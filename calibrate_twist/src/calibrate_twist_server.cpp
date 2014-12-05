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
    nhPriv.getParamCached("traj_sim_granularity_", traj_sim_granularity_);
    nhPriv.getParamCached("traj_dist_threshold", traj_dist_threshold);
    nhPriv.getParamCached("accel_max_x", accel_max_x);
    nhPriv.getParamCached("accel_max_y", accel_max_y);
    nhPriv.getParamCached("accel_max_theta", accel_max_theta);

    goal_ = *goal;

    listener = new tf::TransformListener((goal_.duration)*2); // set cache time twice the time of the calibr. run
    cost_map = new costmap_2d::Costmap2DROS(cal_costmap, *listener);

    voronoi_.initializeEmpty(cost_map->getSizeInCellsX(), cost_map->getSizeInCellsY(), true);


    //ros::Subscriber cm_sub = nh_.subscribe("~/move_base/local_costmap/obstacles", 1, CalibrateAction::costmapCB);

    message_filters::Subscriber<nav_msgs::Odometry> sub(nh_, "/odom", 1);
    odo_cache = new message_filters::Cache<nav_msgs::Odometry> (sub, odo_cache_depths);

    estTraj_pub = nh_.advertise<geometry_msgs::PoseArray>("est_traj_", 10);
    calcTraj_pub = nh_.advertise<geometry_msgs::PoseArray>("calc_traj_", 10);

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

    updateVoronoi(); // loads values from costmap and pushes into voronoi
//**************************************************
    // bringing the robot to the goal speed
//**************************************************
    bool stability_reached = false;

    if(checkPath(0, 0, 0, goal_.twist_goal.linear.x, goal_.twist_goal.linear.y, goal_.twist_goal.angular.z, accel_max_x, accel_max_y, accel_max_theta))
    {
        stability_reached = bringupGoalSpeed();
    }

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
        if(checkPath(goal_.twist_goal.linear.x, goal_.twist_goal.linear.y, goal_.twist_goal.angular.z, goal_.duration.toSec(),
                  goal_.twist_goal.linear.x, goal_.twist_goal.linear.y, goal_.twist_goal.angular.z, 0, 0, 0))
        {
            startCalibrationRun();
        }
        else
        {
            twist_pub.publish(zero_twist); // safety first, stop robot
            as_.setAborted();
            ROS_INFO("%s: Aborted. No space to drive planned trajectory", action_name_.c_str());
            success = false;
        }

    }
    // end of movement, therefore robo is stopped
    twist_pub.publish(zero_twist); // safety first, stop robot

//**************************************************
    // calculating the result
//**************************************************

    if(success)
    {
        calculateResult();
    }

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

/*
  void CalibrateAction::costmapCB(const nav_msgs::GridCells::ConstPtr& msg)
  {
      updateVoronoi(msg);
  }
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate");

  CalibrateAction calibrate(ros::this_node::getName());

  ROS_INFO("Calibration Server started");

  ros::spin();

  return 0;
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

        updateVoronoi(); // loads values from costmap and pushes into voronoi
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

        updateVoronoi(); // loads values from costmap and pushes into voronoi

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

    // publishing to rviz
    geometry_msgs::PoseArray poses;
    for(unsigned int i=0; i<movement_transforms.size();i++)
    {
        geometry_msgs::Pose temp_pose;
        //tf::transformTFToMsg(movement_transforms[i], temp_pose);
        tf::poseTFToMsg(movement_transforms[i], temp_pose);
        poses.poses.push_back(temp_pose);
    }
    poses.header.frame_id = tfFixedFrame;
    calcTraj_pub.publish(poses);
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
    bool reverse = false;
    ROS_ASSERT(transforms.size()> transforms_interval_size); // make sure we have a large enough interval of transforms
    // only iterates to the size of the vector minus the given interval for caclulating the difference as a twist
    for(unsigned int i=0; i<transforms.size()-transforms_interval_size; i++)
    {
        ros::Duration dur = transforms[i+transforms_interval_size].stamp_ - transforms[i].stamp_;
        tf::Transform diff_transform = transforms[i].inverseTimes(transforms[i+transforms_interval_size]);

        if(!reverse)
        {
            // calculate by angles if reverse driving is highly likely
            double diff_angle = angles::normalize_angle(atan2(diff_transform.getOrigin().getY(),diff_transform.getOrigin().getX()));
            double angle1 = angles::normalize_angle(tf::getYaw(transforms[i].getRotation())) - diff_angle;
            double angle2 = angles::normalize_angle(tf::getYaw(transforms[i + transforms_interval_size].getRotation())) - diff_angle;
            reverse = (fabs(angle1+angle2)>fabs((M_PI - angle1)+ (M_PI - angle2)));
        }

        geometry_msgs::Twist tempTwist = calcTwistFromTransform(diff_transform,dur, reverse);
        transform_twists.push_back(tempTwist);
    }
    ROS_INFO("Reverse drive: %i", reverse);
    return calcTwistWithCov(transform_twists);
}

// calculates the linear and roatation speed out of given transform and duration and reverse drive information
geometry_msgs::Twist CalibrateAction::calcTwistFromTransform(tf::Transform _transform, ros::Duration _dur, bool reverse_)
{
    geometry_msgs::Twist result_twist;

    result_twist.linear.x = reverse_ ? (_transform.getOrigin().length() / _dur.toSec() *(-1)) : (_transform.getOrigin().length() / _dur.toSec());
    result_twist.linear.y = 0;
    result_twist.linear.z = 0;

    result_twist.angular.x = 0;
    result_twist.angular.y = 0;
    result_twist.angular.z = (tf::getYaw(_transform.getRotation())) / _dur.toSec();

    return result_twist;
}

/*
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
*/

void CalibrateAction::updateVoronoi()
{
    std::vector<IntPoint> obstacles;
    cost_map->getCostmapCopy(costmap_);

    ROS_ASSERT(costmap_.getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_.getSizeInCellsY() == voronoi_.getSizeY());


    for(unsigned int x = 0; x < costmap_.getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_.getSizeInCellsY(); y++) {
            if(costmap_.getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE) { // lethal and unknown
                obstacles.push_back(IntPoint(x, y));
            }
        }
    }
    voronoi_.exchangeObstacles(obstacles);
    voronoi_.update(true);
    visualizeVoronoi();
}

void CalibrateAction::visualizeVoronoi()
{
    double vis_max_dist_ = 1.0; // temporary because no param found
    ROS_ASSERT(costmap_.getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_.getSizeInCellsY() == voronoi_.getSizeY());

    // nothing to send to no one
    if(voronoi_pub.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray channelMarkers;
    visualization_msgs::Marker voronoiMarker;
    voronoiMarker.header.frame_id = cost_map->getGlobalFrameID();
    voronoiMarker.header.stamp = ros::Time(0);
    voronoiMarker.ns = "voronoi";
    voronoiMarker.id = 0;
    voronoiMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    voronoiMarker.action = visualization_msgs::Marker::ADD;
    voronoiMarker.pose.orientation.w = 1.0;
    voronoiMarker.scale.x = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.scale.y = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.scale.z = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.frame_locked = false;
    geometry_msgs::Point cellPoint;
    cellPoint.z = - costmap_.getResolution() * sqrt(2.0)/2.0;
    std_msgs::ColorRGBA cellColor;
    cellColor.a = 1.0;
    for(unsigned int x = 0; x < costmap_.getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_.getSizeInCellsY(); y++) {
            float dist = voronoi_.getDistance(x, y);
            dist *= costmap_.getResolution();   // now in meters
            costmap_.mapToWorld(x, y, cellPoint.x, cellPoint.y);
            voronoiMarker.points.push_back(cellPoint);

            if(dist == -INFINITY) {
                cellColor.r = 1.0;
                cellColor.g = 0.0;
                cellColor.b = 1.0;
            } else if(dist == INFINITY) {
                cellColor.r = 0.0;
                cellColor.g = 1.0;
                cellColor.b = 0.0;
            } else {
                if(dist > vis_max_dist_) {
                    cellColor.r = cellColor.g = cellColor.b = 1.0;
                } else {
                    // make those slightly darker then max dist to distinguish
                    // vis max dist regions
                    cellColor.r = cellColor.g = cellColor.b = 0.9 * dist / vis_max_dist_;
                }
            }

            voronoiMarker.colors.push_back(cellColor);
        }
    }
    channelMarkers.markers.push_back(voronoiMarker);
    voronoi_pub.publish(channelMarkers);
}


bool CalibrateAction::checkTrajectory(Trajectory& traj)
{
    unsigned int size = traj.getPointsSize();
    float smallest_dist = INFINITY;
    bool isValid;
    for(unsigned int i = 0; i< size;i++)
    {
        geometry_msgs::Pose tempPose;
        tf::Pose tfPose;
        traj.getPoint(i,tempPose.position.x, tempPose.position.y, tempPose.orientation.z);
        tf::poseMsgToTF(tempPose, tfPose);
        float dist = getDistanceAtPose(tfPose, &isValid);

        if ((dist < traj_dist_threshold) && (dist != -INFINITY) && (dist != INFINITY)) // prevent negative infinity value from crashing us
        {
            ROS_INFO("Critical distance was %f at point %i, point valid: %i", dist, i, isValid);
            return false;
        }
    }
    ROS_INFO("Trajectory check successful, smallest distance was: %f", smallest_dist);
    return true;

    /*
    unsigned int size = traj.getPointsSize();
   float smallest_dist = INFINITY;
    for(unsigned int i = 0; i< size;i++)
    {
        geometry_msgs::Pose testPose;
        traj.getPoint(i,testPose.position.x, testPose.position.y, testPose.orientation.z);
        float dist = voronoi_.getDistance(testPose.position.x, testPose.position.y); // get closest distance from trajectory point to an obstacle
        dist *= costmap_.getResolution(); // distance now in meters
        if (dist < smallest_dist)
        {
            smallest_dist = dist;
        }
        if ((dist < traj_dist_threshold) && (dist != -INFINITY) && (dist != INFINITY)) // prevent negative infinity value from crashing us
        {
            ROS_INFO("Critical distance was %f at point %i", dist, i);
            return false;
        }
    }
    */
}

void CalibrateAction::visualize_trajectory(Trajectory &traj)
{
    // publishing to rviz
    geometry_msgs::PoseArray poses;
    for(unsigned int i=0; i<traj.getPointsSize();i++)
    {
        geometry_msgs::Pose temp_pose;
        double x_, y_,th_;
        traj.getPoint(i, x_, y_, th_);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(th_),temp_pose.orientation);
        temp_pose.position.x = x_;
        temp_pose.position.y = y_;
        poses.poses.push_back(temp_pose);
        //ROS_INFO("Pose: x: %f y: %f th: %f", x_, y_, th_);
    }
    geometry_msgs::Pose temp_pose, temp_pose2;
    double x_, y_,th_;
    traj.getPoint(0, x_, y_, th_);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(th_),temp_pose.orientation);
    temp_pose.position.x = x_;
    temp_pose.position.y = y_;
    traj.getEndpoint(x_, y_, th_);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(th_),temp_pose2.orientation);
    temp_pose2.position.x = x_;
    temp_pose2.position.y = y_;
    ROS_INFO("Visualize trajectory with %i points\nFirst Point: x: %f y: %f\nLast Point: x: %f y: %f"
             , traj.getPointsSize(),temp_pose.position.x,temp_pose.position.y,temp_pose2.position.x,temp_pose2.position.y);
    poses.header.frame_id = cost_map->getGlobalFrameID();;
    estTraj_pub.publish(poses);
}

// checks if a path is clear from current position for given speed, goal speed, time and accel
bool CalibrateAction::checkPath(double vx, double vy, double vtheta, double  sim_time_, double vx_samp, double vy_samp, double vtheta_samp,
               double acc_x, double acc_y, double acc_theta)
{
    Trajectory traj;
    tf::StampedTransform transform;
    try
    {
        listener->lookupTransform(cost_map->getGlobalFrameID(),robotFrame,ros::Time::now(), transform);
    }
     catch (tf::TransformException ex)
     {
         ROS_ERROR("Nope! %s", ex.what());
     }

    ROS_INFO("gen. Traj with pos x: %f, y: %f, th: %f, vel: x: %f, y:%f, th: %f, dur: %f", transform.getOrigin().getX(), transform.getOrigin().getY(), tf::getYaw(transform.getRotation()), vx, vy, vtheta, sim_time_);

    // generate a trajectory for the given goal speed, rotation and time. No acceleration needed here
    generateTrajectory(transform.getOrigin().getX(), transform.getOrigin().getY(), tf::getYaw(transform.getRotation()),
                       vx, vy, vtheta, sim_time_, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, traj);
    visualize_trajectory(traj);


    return checkTrajectory(traj);
}

// checks path when starting from zero speed with unknown speed up time
bool CalibrateAction::checkPath(double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp,
               double acc_x, double acc_y, double acc_theta)
{
    Trajectory traj;
    tf::StampedTransform transform;
    try
    {
        listener->lookupTransform(cost_map->getGlobalFrameID(),robotFrame,ros::Time::now(), transform);
    }
     catch (tf::TransformException ex)
     {
         ROS_ERROR("Nope! %s", ex.what());
     }



    // getting maximum needed sim_time_, absolute value, as time can't be negative
    double sim_time_x = fabs((vx_samp - vx)/acc_x);
    double sim_time_y = fabs((vy_samp - vy)/acc_y);
    double sim_time_theta = fabs((vtheta_samp - vtheta)/acc_theta);

    double sim_time_ = std::max(sim_time_x,sim_time_y);
            sim_time_ = std::max(sim_time_,sim_time_theta);

    ROS_INFO("gen. Traj with pos x: %f, y: %f, th: %f, vel: x: %f, y:%f, th: %f, dur: %f", transform.getOrigin().getX(), transform.getOrigin().getY(), tf::getYaw(transform.getRotation()), vx, vy, vtheta, sim_time_);

    // generate a trajectory for the given goal speed, rotation and time. No acceleration needed here
    generateTrajectory(transform.getOrigin().getX(), transform.getOrigin().getY(), tf::getYaw(transform.getRotation()),
                       vx, vy, vtheta, sim_time_, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta, traj);
    visualize_trajectory(traj);


    return checkTrajectory(traj);
}

double CalibrateAction::getDistanceAtPose(const tf::Pose & pose, bool* in_bounds) const
{
    // determine current dist
    int pose_x, pose_y;
    costmap_.worldToMapNoBounds(pose.getOrigin().x(), pose.getOrigin().y(),
            pose_x, pose_y);
    //ROS_INFO("pose_x: %i, pose_y: %i", pose_x, pose_y);
    if(pose_x < 0 || pose_y < 0 ||
            pose_x >= (int)voronoi_.getSizeX() || pose_y >= (int)voronoi_.getSizeY()) {
        if(in_bounds == NULL) {
            // only warn if in_bounds isn't checked externally
            ROS_WARN_THROTTLE(1.0, "%s: Pose out of voronoi bounds (%.2f, %.2f) = (%d, %d)", __func__,
                    pose.getOrigin().x(), pose.getOrigin().y(), pose_x, pose_y);
        } else {
            *in_bounds = false;
        }
        return 0.0;
    }
    if(in_bounds)  {
        *in_bounds = true;
    }
    float dist = voronoi_.getDistance(pose_x, pose_y);
    dist *= costmap_.getResolution();
    return dist;
}


/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the WillgetPointow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

/**
* create and score a trajectory given the current pose of the robot and selected velocities
*/
void CalibrateAction::generateTrajectory(
    double x, double y, double theta,
    double vx, double vy, double vtheta, double  sim_time_, double vx_samp, double vy_samp, double vtheta_samp,
        double acc_x, double acc_y, double acc_theta, Trajectory& traj) {

    double x_i = x;
    double y_i = y;
    double theta_i = theta;
    double vx_i, vy_i, vtheta_i;
    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(sim_time_ / traj_sim_granularity_ + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
        num_steps = 1;
    }
    double dt = sim_time_ / num_steps;

    //create a potential trajectory    
    traj.resetPoints();
    traj.xv_ = vx_i;
    traj.yv_ = vy_i;
    traj.thetav_ = vtheta_i;

    for(int i = 0; i < num_steps; ++i){
        //the point is legal... add it to the trajectory
        traj.addPoint(x_i, y_i, theta_i);

        //calculate velocities
        vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
        vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
        vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

        //calculate positions
        x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
        y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
        theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);
    } // end for i < numsteps
}
