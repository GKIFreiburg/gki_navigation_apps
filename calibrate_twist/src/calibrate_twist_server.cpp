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
    //bool cache_flag = false;
    as_.start();
  }

  /*
  void CalibrateAction::odo_cacheCB(const nav_msgs::Odometry::ConstPtr &msg)
  {

      if(cache_flag == false)
      {
          ROS_INFO("X");
          cache_flag = true;
      }
      else
      {
          ROS_INFO("Y");
      }
  }
*/

  CalibrateAction::~CalibrateAction(void)
  {
  }

  void CalibrateAction::executeCB(const calibrate_twist::CalibrateGoalConstPtr &goal)
  {
    bool success = true;
    bool stability_reached = false;
    int stability_counter = 0;

    // read all necessary parameters
    nh_.getParamCached("/calibrate_twist/odo_cache_depths", odo_cache_depths);
    nh_.getParamCached("/calibrate_twist/stability_timeout", stability_timeout);
    nh_.getParamCached("/calibrate_twist/stability_intervalDuration", stability_intervalDuration);
    nh_.getParamCached("/calibrate_twist/stability_xThreshold", stability_xThreshold);
    nh_.getParamCached("/calibrate_twist/stability_zThreshold", stability_zThreshold);
    nh_.getParamCached("/calibrate_twist/calibration_calc_interval", calibration_calc_interval);
    nh_.getParamCached("/calibrate_twist/tfFixedFrame", tfFixedFrame);


    listener = new tf::TransformListener((goal->duration)*2); // set cache time twice the time of the calibr. run

    //ros::Subscriber sub = nh_.subscribe("/odom", odo_cache_depths, someCallback);
    message_filters::Subscriber<nav_msgs::Odometry> sub(nh_, "/odom", 1);
    message_filters::Cache<nav_msgs::Odometry> odo_cache(sub, odo_cache_depths);
    //odo_cache.registerCallback(boost::bind(&CalibrateAction::odo_cacheCB, this, _1));


    twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate r(10);

    geometry_msgs::Twist zero_twist; // how to initialize as zero?
    zero_twist.angular.x = 0; // necessary?
    zero_twist.angular.y = 0; // necessary?
    zero_twist.angular.z = 0; // necessary?
    zero_twist.linear.x = 0; // necessary
    zero_twist.linear.y = 0; // necessary
    zero_twist.linear.z = 0; // necessary

    // publish info to the console for the user
    ROS_INFO("%s: Executing, running calibration run for %f seconds with linear speed %f, angular speed %f", action_name_.c_str(), goal->duration.toSec(), goal->twist_goal.linear.x, goal->twist_goal.angular.z);

//**************************************************
    // bringing the robot to the goal speed
//**************************************************
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
        twist_pub.publish(goal->twist_goal);

//**************************************************
    // checking continuity of achieved velocity
//**************************************************

        // get a odometry interval out of the cache and check for continuity of the contained twists
        ros::Time continuity_start = odo_cache.getLatestTime(); // results in weird absolute time once cache is filled?!
        // result of getLatestTime is zero in the beginning and crashes if we calculate with the substracted interval duration
        if(!continuity_start.isZero()) // maybe assure here that at least x values are in cache already
        {
            continuity_start = continuity_start - ros::Duration(stability_intervalDuration); // take the correct interval into account
            ros::Time continuity_end = odo_cache.getLatestTime();
            std::vector<nav_msgs::Odometry::ConstPtr> consistenty_odo_interval = odo_cache.getInterval(continuity_start,continuity_end);
            std::vector<geometry_msgs::TwistWithCovariance> twist_interval;
            // extract the twists out of the odo values
            for(unsigned int i=0; i<consistenty_odo_interval.size(); i++)
            {
                twist_interval.push_back(consistenty_odo_interval[i]->twist);
            }

            geometry_msgs::TwistWithCovariance result_twist;
            // calc the covariance out of the interval
            calcTwistWithCov(twist_interval,&result_twist);
            ROS_INFO("%lu values: constance check: %4.3f on x-axis, %4.3f on z-rot", consistenty_odo_interval.size(), result_twist.covariance[0],result_twist.covariance[35]);

            //only if both values are lower than the threshold for 3x in a row we assume stability is reached
            if((result_twist.covariance[0] < stability_xThreshold) && (result_twist.covariance[35] < stability_zThreshold))
            {
                stability_counter++;
                if (stability_counter > 2) // make this a parameter?
                {
                    stability_reached = true;
                    ROS_INFO("Stability reached after %f seconds", ros::Time::now().toSec()-stability_timeout_start.toSec() );
                    break;
                }
            }
            else
            {
                // reset counter so we only count three reached criteria in a row
                stability_counter = 0;
            }
            ROS_INFO("start time: %.2f , stability_cnt: %i",continuity_start.toSec(), stability_counter);
        }
        else
        {
            ROS_INFO("Dooohhh! Invalid Start time: %f !", continuity_start.toSec());
        }

        r.sleep();
    }


    if(!stability_reached)
    {
        twist_pub.publish(zero_twist); // safety first, stop robot
        as_.setAborted();
        ROS_INFO("%s: Aborted. No stability reached within timeout", action_name_.c_str());
        success = false;
    }
    else
    {
//**************************************************
    // starting calibration run for given duration
//**************************************************

        calibration_start = ros::Time::now();
        // starting calibration run
        while((ros::Time::now().toSec()) < (calibration_start.toSec() + goal->duration.toSec()))
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
            twist_pub.publish(goal->twist_goal);
            r.sleep();
        }
        calibration_end = ros::Time::now();
    }     
    // end of movement, therefore robo is stopped
    twist_pub.publish(zero_twist); // safety first, stop robot

//**************************************************
    // calculating the result
//**************************************************
    // create the transform that we fill, using tf
    std::vector<tf::StampedTransform> movement_transforms;
    unsigned int interval_steps = goal->duration.toSec() / calibration_calc_interval;
    for(unsigned int i = 1; i<=(interval_steps);i++)
    {
        tf::StampedTransform tempTransform;

        std::vector<nav_msgs::Odometry::ConstPtr> calibration_odo_interval = odo_cache.getInterval(calibration_start,calibration_start+ros::Duration(i*calibration_calc_interval));
        /*
        calibration_odo_interval.front()->header.frame_id;
        calibration_odo_interval.front()->header.stamp;
        calibration_odo_interval.back()->header.frame_id;
        calibration_odo_interval.back)->header.stamp;
        */
        // make the tf lookup to get the transformation from one scan to the other
        if(!calibration_odo_interval.empty())
        {
            try{
              listener->waitForTransform(calibration_odo_interval.front()->header.frame_id, calibration_odo_interval.front()->header.stamp,
                                        calibration_odo_interval.back()->header.frame_id, calibration_odo_interval.back()->header.stamp,
                                        tfFixedFrame, ros::Duration(1.0));


              // uses the two scans and /odom as fixed frame to get a transformation
              // transform naming: from_to (e.g. transformOld_New means from Old to New)

              // calculates the transform from source (scanOld) to target (scanNew)
              listener->lookupTransform(calibration_odo_interval.front()->header.frame_id, calibration_odo_interval.front()->header.stamp,
                                        calibration_odo_interval.back()->header.frame_id, calibration_odo_interval.back()->header.stamp,
                                        tfFixedFrame, tempTransform);
              // store calculated transform
              movement_transforms.push_back(tempTransform); // what happens if exception is thrown? not guaranteed that all transforms can be retrieved
            }
            catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
            }
        }
        else
        {
            ROS_INFO("No Odo Data received from calibration run ");
        }
    }



    geometry_msgs::TwistWithCovariance tw;
    std::vector<geometry_msgs::Twist> twists_vector;
    twists_vector.push_back(goal->twist_goal);
    CalibrateAction::calcTwistWithCov(twists_vector, &tw);

    if(success)
    {
      result_.calibrated_result = tw;
      result_.odo_result = tw;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate");

  CalibrateAction calibrate(ros::this_node::getName());

  ROS_INFO("Calibration Server started");

  ros::spin();

  return 0;
}

void CalibrateAction::calcTwistWithCov(std::vector<geometry_msgs::Twist> twists, geometry_msgs::TwistWithCovariance* resultTwist)
{
    /*
    double angular = 0;
    double linear = 0;
    foreach(geometry_msgs::Twist tw, twists)
    {
        // check if twists are positiv?
        angular += tw.angular.z;
        linear  += tw.linear.x;
    }
    angular = angular / twists.size();
    linear  = linear  / twists.size();

    geometry_msgs::TwistWithCovariance a;
*/

/*
    int n = twists.size();
    MatrixXd mat(n,2);
    // creating value matrix here
    for (int i = 0; i<n; i++)
    {
        mat(i,0) = twists[i]->angular.z;
        mat(i,1) = twists[i]->linear.x;
    }
    MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    *cov = (centered.adjoint() * centered) / double(mat.rows());
    Vector2d meanVector = mat.colwise().mean();


    resultTwist.twist.angular.z = double(meanVector(0));
    resultTwist.twist.linear.x = double(meanVector(1));

    resultTwist.covariance = ;
*/
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
    resultTwist->twist.linear.x = double(meanVector(0));
    resultTwist->twist.angular.z = double(meanVector(5));

    double *resultC = &(resultTwist->covariance[0]);

    //Map<MatrixXd>( resultC, covMat.rows(), covMat.cols() ) = covMat;
    Map<Matrix<double,6,6,RowMajor> >(resultC, 6,6) = covMat;

    // print for test only
    std::cout << "CovMat:\n" << covMat <<"\n";
}

void CalibrateAction::calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twists, geometry_msgs::TwistWithCovariance* resultTwist)
{
    int n = twists.size();
    MatrixXd mat(n,6);
    // creating value matrix here
    for (int i = 0; i<n; i++)
    {
        mat(i,0) = twists[i].twist.linear.x;
        mat(i,1) = twists[i].twist.linear.y;
        mat(i,2) = twists[i].twist.linear.z;
        mat(i,3) = twists[i].twist.angular.x;
        mat(i,4) = twists[i].twist.angular.y;
        mat(i,5) = twists[i].twist.angular.z;
    }
    MatrixXd centered = mat.rowwise() - mat.colwise().mean();
    MatrixXd covMat = MatrixXd::Ones(6,6);
    covMat = (centered.adjoint() * centered) / double(mat.rows());
    VectorXd meanVector(6);
    meanVector = mat.colwise().mean();
    resultTwist->twist.linear.x = double(meanVector(0));
    resultTwist->twist.angular.z = double(meanVector(5));

    double *resultC = &(resultTwist->covariance[0]);

    //Map<MatrixXd>( resultC, covMat.rows(), covMat.cols() ) = covMat;

    // arrange resultMatrix as RowMajor as required by TwistWithCovariance type
    Map<Matrix<double,6,6,RowMajor> >(resultC, 6,6) = covMat; // unclear if it works as it's supposed to

    // print for test only
    std::cout << "CovMat:\n" << covMat <<"\n";
    //std::cout << "CovMat:\n" << Map<Matrix<double,6,6,RowMajor> >(resultC, 6,6) <<"\n"; // test only
}




