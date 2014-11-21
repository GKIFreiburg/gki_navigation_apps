/*#include <ros/ros.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <calibrate_twist/CalibrateAction.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
*/


#include "calibrate_twist/calibrate_twist_server.h"

  CalibrateAction::CalibrateAction(std::string name) :
    as_(nh_, name, boost::bind(&CalibrateAction::executeCB, this, _1), false),
    action_name_(name)
  {
    ros::NodeHandle nhPriv("~");

    // read all necessary parameters
    nhPriv.getParamCached("voronoi_grid_size", voronoi_grid_size);// size of the overall voronoi grid in meters (robot stands in center of that grid)
    nhPriv.getParamCached("voronoi_grid_resolution", voronoi_grid_resolution);// defines the edge length in meters of a voronoi grid cell

    // Initialize of Voronoi stuff
    int size_x = (int) voronoi_grid_size / voronoi_grid_resolution;
    int size_y = (int) voronoi_grid_size / voronoi_grid_resolution;
    //voronoi_.changeMaxDist(100); // change the calculation distance of voronoi
    voronoi_.initializeEmpty(size_x, size_y, true);



    as_.start();
  }

  CalibrateAction::~CalibrateAction(void)
  {
  }

  void CalibrateAction::executeCB(const calibrate_twist::CalibrateGoalConstPtr &goal)
  {
    success = true;

    // read all necessary parameters
    ros::NodeHandle nhPriv("~");
    nhPriv.getParamCached("odo_cache_depths", odo_cache_depths);
    nhPriv.getParamCached("stability_timeout", stability_timeout);
    nhPriv.getParamCached("stability_intervalDuration", stability_intervalDuration);
    nhPriv.getParamCached("stability_xThreshold", stability_xThreshold);
    nhPriv.getParamCached("stability_zThreshold", stability_zThreshold);
    nhPriv.getParamCached("calibration_calc_interval", calibration_calc_interval);
    nhPriv.getParamCached("tfFixedFrame", tfFixedFrame);
    nhPriv.getParamCached("robotFrame", robotFrame);
    nhPriv.getParamCached("minStabilityDuration", minStabilityDuration);
    nhPriv.getParamCached("transforms_interval_size", transforms_interval_size);
    nhPriv.getParamCached("cal_costmap", cal_costmap);

    goal_ = *goal;

    listener = new tf::TransformListener((goal_.duration)*2); // set cache time twice the time of the calibr. run
    cost_map = new costmap_2d::Costmap2DROS(cal_costmap, *listener);

    ros::Subscriber cm_sub = nh_.subscribe("~/move_base/local_costmap/obstacles", 1, CalibrateAction::costmapCB);

    message_filters::Subscriber<nav_msgs::Odometry> sub(nh_, "/odom", 1);
    odo_cache = new message_filters::Cache<nav_msgs::Odometry> (sub, odo_cache_depths);

    marker_pub = nh_.advertise<geometry_msgs::PoseArray>("trajectory_", 10);

    twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    voronoi_pub = nh_.advertise<visualization_msgs::MarkerArray>("voronoi_marker", 10);


    // necessary? how to initialize as empty?
    zero_twist.angular.x = 0;
    zero_twist.angular.y = 0;
    zero_twist.angular.z = 0;
    zero_twist.linear.x = 0;
    zero_twist.linear.y = 0;
    zero_twist.linear.z = 0;

    // publish info to the console for the user
    ROS_INFO("%s: Executing, running calibration run for %f seconds with linear speed %f, angular speed %f", action_name_.c_str(), goal_.duration.toSec(), goal_.twist_goal.linear.x, goal_.twist_goal.angular.z);

//**************************************************
    // bringing the robot to the goal speed
//**************************************************
    bool stability_reached = false;
    stability_reached = bringupGoalSpeed();

    // not stopping robot here because we want to have continuous movement!

    if(!stability_reached)
    {
        twist_pub.publish(zero_twist); // safety first, stop robot
        as_.setAborted();
        ROS_INFO("%s: Aborted. No stability reached within timeout", action_name_.c_str());
        success = false;
    }
//**************************************************
    // starting calibration run for given duration
//**************************************************
    else if(success)
    {
        startCalibrationRun();

    }
    // end of movement, therefore robo is stopped
    twist_pub.publish(zero_twist); // safety first, stop robot

//**************************************************
    // calculating the result
//**************************************************

    calculateResult();

//**************************************************
    // publishing the result
//**************************************************

    if(success)
    {
      result_.calibrated_result = twistWCFromTf;
      result_.odo_result = twistWCFromOdometry;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    // callback finished
  }

  void CalibrateAction::costmapCB(const nav_msgs::GridCells::ConstPtr& msg)
  {
      updateVoronoi(msg);
  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate");

  CalibrateAction calibrate(ros::this_node::getName());

  ROS_INFO("Calibration Server started");

  ros::spin();

  return 0;
}

void CalibrateAction::updateVoronoi(const nav_msgs::GridCells::ConstPtr& msg)
{
    std::vector<IntPoint> newObstacles;
    for(unsigned int i=0; i++; i<msg->cells.size())
    {
        IntPoint temp_point;
        if(worldToGrid(&(msg->cells[i]), &temp_point))
        {
            newObstacles.push_back(temp_point);
        }
    }
    voronoi_.exchangeObstacles(newObstacles);
    voronoi_.update(true);
    visualizeVoronoi();
}


bool CalibrateAction::bringupGoalSpeed()
{
    ros::Rate r(10);
    bool stability_reached = false;
    bool first_stability = false;
    ros::Time first_stability_time;


    ros::Time stability_timeout_start = ros::Time::now();
    while((ros::Time::now().toSec()) < (stability_timeout_start.toSec() + stability_timeout))
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
          twist_pub.publish(zero_twist); // safety first, stop robot
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
          break;
        }
        // driving the robot with the intended parameters
        twist_pub.publish(goal_.twist_goal);

        //**************************************************
            // checking continuity of achieved velocity
        //**************************************************

       stability_reached = checkOdoConsistency(first_stability, first_stability_time);
       if(stability_reached)
       {
           ROS_INFO("Stability reached after %.2f seconds", ros::Time::now().toSec()-stability_timeout_start.toSec() );
           break;
       }
       r.sleep(); // ensure 10Hz for cmd_vel
    }
    return stability_reached;
}

bool CalibrateAction::checkOdoConsistency(bool &first_stability, ros::Time &first_stability_time)
{
    bool stability_reached = false;

    // get a odometry interval out of the cache and check for continuity of the contained twists
    ros::Time continuity_start = odo_cache->getLatestTime(); // results in weird absolute time once cache is filled?!
    // result of getLatestTime is zero in the beginning and crashes if we calculate with the substracted interval duration
    if(!continuity_start.isZero()) // maybe assure here that at least x values are in cache already
    {
        continuity_start = continuity_start - ros::Duration(stability_intervalDuration); // take the correct interval into account
        ros::Time continuity_end = odo_cache->getLatestTime();
        std::vector<nav_msgs::Odometry::ConstPtr> consistency_odo_interval = odo_cache->getInterval(continuity_start,continuity_end);

        // calc the covariance out of the interval
        geometry_msgs::TwistWithCovariance result_twist = calcTwistWithCov(consistency_odo_interval);

        ROS_INFO("%lu values: constance check: %4.3f on x-axis, %4.3f on z-rot", consistency_odo_interval.size(), result_twist.covariance[0],result_twist.covariance[35]);

        //only if both values are lower than the threshold for 3x in a row we assume stability is reached
        if((result_twist.covariance[0] < stability_xThreshold) && (result_twist.covariance[35] < stability_zThreshold))
        {
            if(!first_stability)
            {
                first_stability = true;
                first_stability_time = ros::Time::now();
            }
            else if (ros::Time::now()>(first_stability_time + ros::Duration(minStabilityDuration)))
            {
                stability_reached = true;
            }
        }
        else
        {
            if(first_stability)
            {
                // clear stability criteria so the duration timer gets reset
                first_stability = false;
                ROS_INFO("Stability timer reset!");
            }
        }
    }
    else
    {
        ROS_INFO("Doohh! Invalid Start time: %.2f !", continuity_start.toSec());
    }
    return stability_reached;
}

void CalibrateAction::startCalibrationRun()
{
    ros::Rate r(10);
    calibration_start = ros::Time::now();
    // starting calibration run
    while((ros::Time::now().toSec()) < (calibration_start.toSec() + goal_.duration.toSec()))
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
          twist_pub.publish(zero_twist); // safety first, stop robot
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
          break;
        }
        // driving the robot with the intended parameters
        twist_pub.publish(goal_.twist_goal);

        //voronoi_.exchangeObstacles(cost_map.getPoints());
        //voronoi_.update(true);
        //visualizeVoronoi();

        r.sleep(); // ensure 10Hz for cmd_vel
    }
    calibration_end = ros::Time::now();
}

void CalibrateAction::calculateResult()
{
    // retrieving values from odo cache
    std::vector<nav_msgs::Odometry::ConstPtr> calibration_odo_interval = odo_cache->getInterval(calibration_start,calibration_end);

    // create the transform vector that we fill, using tf
    std::vector<tf::StampedTransform> movement_transforms;

    // wait for 300ms to allow the tf buffer to be filled with enough values
    ros::Duration d = ros::Duration(0.3);
    d.sleep();

    for(ros::Time cTime = calibration_start; cTime < calibration_end; cTime += ros::Duration(calibration_calc_interval) )
    {
        tf::StampedTransform tempTransform;

        // make the tf lookup to get the transformation from robot frame to the fixed frame
        try{
            listener->lookupTransform(tfFixedFrame, robotFrame,
                                        cTime, tempTransform);
          // store calculated transform
          movement_transforms.push_back(tempTransform); // what happens if exception is thrown? not guaranteed that all transforms can be retrieved
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
    }

    // for information if the transform lookup didn't succeed (at least 5 lookups unsuccessful)
    unsigned int interval_steps = goal_.duration.toSec() / calibration_calc_interval;
    if((movement_transforms.size()+5)<interval_steps)
    {
        ROS_INFO("Only %lu transformations from %u steps", movement_transforms.size(), interval_steps);
        ROS_INFO("given time: %2.1f, actual time: %2.1f, actual steps: %2.1f", (goal_.duration).toSec(),ros::Duration(calibration_end - calibration_start).toSec(), ros::Duration(calibration_end - calibration_start).toSec() / calibration_calc_interval);
    }
    else
    {
        //ROS_INFO("Result contains %lu transformations from %u steps", movement_transforms.size(), interval_steps);
        //ROS_INFO("given time: %2.1f, actual time: %2.1f, actual steps: %2.1f", (goal_.duration).toSec(),ros::Duration(calibration_end - calibration_start).toSec(), ros::Duration(calibration_end - calibration_start).toSec() / calibration_calc_interval);
    }

    // calculating odo based result
    twistWCFromOdometry = calcTwistWithCov(calibration_odo_interval);

    // calculating tf based result
    twistWCFromTf = estimateTwWithCovFromTrajectory(movement_transforms);

    // publishing to tf
    geometry_msgs::PoseArray poses;
    for(unsigned int i=0; i<movement_transforms.size();i++)
    {
        geometry_msgs::Pose temp_pose;
        //tf::transformTFToMsg(movement_transforms[i], temp_pose);
        tf::poseTFToMsg(movement_transforms[i], temp_pose);
        poses.poses.push_back(temp_pose);
    }
    poses.header.frame_id = tfFixedFrame;
    marker_pub.publish(poses);
}

geometry_msgs::TwistWithCovariance CalibrateAction::calcTwistWithCov(std::vector<geometry_msgs::Twist> twists)
{
    geometry_msgs::TwistWithCovariance resultTwist;
    int n = twists.size();
    MatrixXd mat(n,6);
    // creating value matrix here
    for (int i = 0; i<n; i++)
    {
        mat(i,0) = twists[i].linear.x;
        mat(i,1) = twists[i].linear.y;
        mat(i,2) = twists[i].linear.z;
        mat(i,3) = twists[i].angular.x;
        mat(i,4) = twists[i].angular.y;
        mat(i,5) = twists[i].angular.z;
    }
    MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    MatrixXd covMat = MatrixXd::Ones(6,6);
    covMat = (centered.adjoint() * centered) / double(mat.rows());
    VectorXd meanVector(6);
    meanVector = mat.colwise().mean();
    // only need linear.x and angular.z but other values could be useful in the future
    resultTwist.twist.linear.x = double(meanVector(0));
    resultTwist.twist.linear.y = double(meanVector(1));
    resultTwist.twist.linear.z = double(meanVector(2));
    resultTwist.twist.angular.x = double(meanVector(3));
    resultTwist.twist.angular.y = double(meanVector(4));
    resultTwist.twist.angular.z = double(meanVector(5));

    double *resultC = &(resultTwist.covariance[0]);

    // arrange resultMatrix as RowMajor as required by TwistWithCovariance type
    Map<Matrix<double,6,6,RowMajor> >(resultC, 6,6) = covMat;

    // print for test only
    //std::cout << "CovMat:\n" << covMat <<"\n";

    return resultTwist;
}

geometry_msgs::TwistWithCovariance CalibrateAction::calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twistsWC)
{
    std::vector<geometry_msgs::Twist> temp_twists;
    for(unsigned int i=0; i<twistsWC.size(); i++)
    {
        temp_twists.push_back(twistsWC[i].twist);
    }

    return calcTwistWithCov(temp_twists);
}

geometry_msgs::TwistWithCovariance CalibrateAction::calcTwistWithCov(std::vector<nav_msgs::Odometry> odos)
{
    std::vector<geometry_msgs::TwistWithCovariance> temp_twistsWC;
    for(unsigned int i=0; i<odos.size(); i++)
    {
        temp_twistsWC.push_back(odos[i].twist);
    }

    return calcTwistWithCov(temp_twistsWC);
}

geometry_msgs::TwistWithCovariance CalibrateAction::calcTwistWithCov(std::vector<nav_msgs::Odometry::ConstPtr> odos_ptr)
{
    std::vector<geometry_msgs::TwistWithCovariance> temp_twistsWC;
    for(unsigned int i=0; i<odos_ptr.size(); i++)
    {
        temp_twistsWC.push_back(odos_ptr[i]->twist);
    }

    return calcTwistWithCov(temp_twistsWC);
}

// estimates a overall mean twist for given vector of stamped transforms
geometry_msgs::TwistWithCovariance CalibrateAction::estimateTwWithCovFromTrajectory(std::vector<tf::StampedTransform> transforms)
{
    std::vector<geometry_msgs::Twist> transform_twists;
    for(unsigned int i=0; i<transforms.size()-transforms_interval_size; i++)
    {
        ros::Duration dur = transforms[i+transforms_interval_size].stamp_ - transforms[i].stamp_;
        geometry_msgs::Twist tempTwist = calcTwistFromTransform((transforms[i].inverseTimes(transforms[i+transforms_interval_size])),dur);
        transform_twists.push_back(tempTwist);
    }
    return calcTwistWithCov(transform_twists);
}

// calculates the linear and roatation speed out of given transform and duration
geometry_msgs::Twist CalibrateAction::calcTwistFromTransform(tf::Transform _transform, ros::Duration _dur)
{
    geometry_msgs::Twist result_twist;

    result_twist.linear.x = _transform.getOrigin().length() / _dur.toSec();
    result_twist.linear.y = 0;
    result_twist.linear.z = 0;

    result_twist.angular.x = 0;
    result_twist.angular.y = 0;
    result_twist.angular.z = (tf::getYaw(_transform.getRotation())) / _dur.toSec();

    return result_twist;
}


void CalibrateAction::visualizeVoronoi()
{
    // nothing to send to no one
    if(voronoi_pub.getNumSubscribers() == 0)
        return;

    visualization_msgs::Marker voronoiMarker;

    //initializes the markers
    voronoiMarker.header.frame_id = robotFrame;
    voronoiMarker.header.stamp = ros::Time::now();
    voronoiMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    voronoiMarker.action = visualization_msgs::Marker::ADD;
    voronoiMarker.scale.x = voronoi_grid_resolution * sqrt(2.0);
    voronoiMarker.scale.y = voronoi_grid_resolution * sqrt(2.0);
    voronoiMarker.scale.z = voronoi_grid_resolution * sqrt(2.0);
    voronoiMarker.frame_locked = false;
    voronoiMarker.color.r = 1.0f;
    voronoiMarker.color.g = 1.0f;
    voronoiMarker.color.b = 1.0f;
    voronoiMarker.color.a = 1.0f;
    voronoiMarker.ns = "voronoi";
    voronoiMarker.id = 2;

    //initialize the cellpoints
    geometry_msgs::Point cellPoint;
    cellPoint.z = - voronoi_grid_resolution * sqrt(2.0)/2.0;
    std_msgs::ColorRGBA cellColor;
    cellColor.a = 1.0;

    for(unsigned int x = 0; x < (voronoi_grid_size / voronoi_grid_resolution); x++) {
        for(unsigned int y = 0; y < (voronoi_grid_size / voronoi_grid_resolution); y++)
        {
            float dist = voronoi_.getDistance(x, y);
            dist *= voronoi_grid_resolution; // now in meters
            IntPoint gridPoint(x,y);
            gridtoWorld(&gridPoint, &cellPoint);
            voronoiMarker.points.push_back(cellPoint);

            if(dist == -INFINITY) {
                cellColor.r = 1.0;
                cellColor.g = 0.0;
                cellColor.b = 1.0;
                cellColor.a = 1.0;
            } else if(dist == INFINITY) {
                cellColor.r = 0.0;
                cellColor.g = 1.0;
                cellColor.b = 0.0;
                cellColor.a = 0.1; // changed alpha for infinity in voronoi
            }
            voronoiMarker.colors.push_back(cellColor);
        }
    }
    //sends the markers
    visualization_msgs::MarkerArray voronoiMarkerArray;
    voronoiMarkerArray.markers.push_back(voronoiMarker);
    voronoi_pub.publish(voronoiMarkerArray);
}

void CalibrateAction::gridtoWorld(IntPoint* ip, geometry_msgs::Point* wp)
{
  // TODO: spend some thoughts if rounding is neccessary here or not
  double half_mapsize = (voronoi_grid_size / 2);
  wp->x = (float) (ip->x * voronoi_grid_resolution - half_mapsize);
  wp->y = (float) (ip->y * voronoi_grid_resolution - half_mapsize);
}

bool CalibrateAction::worldToGrid(const geometry_msgs::Point* wp, IntPoint* ip)
{
  int temp_x, temp_y;
  double half_mapsize = (voronoi_grid_size / 2);
  geometry_msgs::Point point=*wp;
  // start the coordinates from the middle of the grid
  point.x+=half_mapsize;
  point.y+=half_mapsize;

  // use round_int to correctly round to closest integer value
  temp_x = round(point.x / voronoi_grid_resolution);
  temp_y = round(point.y / voronoi_grid_resolution);

  // the absolute value of both coordinates may not be larger than half of the grid size
  if(temp_x<0 || temp_y<0 || temp_x>= (int) voronoi_.getSizeX() || temp_y>= (int) voronoi_.getSizeY())
  {
    return false;
  }
  // values are in bounds so it's ok to set them
  else
  {
    ip->x = temp_x;
    ip->y = temp_y;
    return true;
  }
}
