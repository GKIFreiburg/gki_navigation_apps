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
    bool success = true;
    bool stability_reached = false;
    bool first_stability = false;
    ros::Time first_stability_time;

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

            // calc the covariance out of the interval
            geometry_msgs::TwistWithCovariance result_twist = calcTwistWithCov(consistenty_odo_interval);

            ROS_INFO("%lu values: constance check: %4.3f on x-axis, %4.3f on z-rot", consistenty_odo_interval.size(), result_twist.covariance[0],result_twist.covariance[35]);

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
                    ROS_INFO("Stability reached after %.2f seconds", ros::Time::now().toSec()-stability_timeout_start.toSec() );
                    break;
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
    // retrieving values from odo cache
    std::vector<nav_msgs::Odometry::ConstPtr> calibration_odo_interval = odo_cache.getInterval(calibration_start,calibration_end);

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
            listener->lookupTransform(robotFrame, tfFixedFrame,
                                        cTime, tempTransform);
          // store calculated transform
          movement_transforms.push_back(tempTransform); // what happens if exception is thrown? not guaranteed that all transforms can be retrieved
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
    }

    // for information if the transform lookup didn't succeed (at least 5 lookups unsuccessful)
    unsigned int interval_steps = goal->duration.toSec() / calibration_calc_interval;
    if((movement_transforms.size()+5)<interval_steps)
    {
        ROS_INFO("Only %lu transformations from %u steps", movement_transforms.size(), interval_steps);
        ROS_INFO("given time: %2.1f, actual time: %2.1f, actual steps: %2.1f", (goal->duration).toSec(),ros::Duration(calibration_end - calibration_start).toSec(), ros::Duration(calibration_end - calibration_start).toSec() / calibration_calc_interval);
    }
    else
    {
        //ROS_INFO("Result contains %lu transformations from %u steps", movement_transforms.size(), interval_steps);
        //ROS_INFO("given time: %2.1f, actual time: %2.1f, actual steps: %2.1f", (goal->duration).toSec(),ros::Duration(calibration_end - calibration_start).toSec(), ros::Duration(calibration_end - calibration_start).toSec() / calibration_calc_interval);
    }

    // calculating odo based result
    geometry_msgs::TwistWithCovariance twistFromOdometry = calcTwistWithCov(calibration_odo_interval);

    // calculating tf based result
    geometry_msgs::TwistWithCovariance twistFromTf = estimateTwWithCovFromTrajectory(movement_transforms);



//**************************************************
    // publishing the result
//**************************************************

    if(success)
    {
      result_.calibrated_result = twistFromTf;
      result_.odo_result = twistFromOdometry;
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

geometry_msgs::TwistWithCovariance CalibrateAction::estimateTwWithCovFromTrajectory(std::vector<tf::StampedTransform> transforms)
{
    std::vector<geometry_msgs::Twist> transform_twists;
    for(unsigned int i=0; i<transforms.size()-transforms_interval_size; i++)
    {
        transform_twists.push_back(getTwistFromTransforms(transforms[i],transforms[i+transforms_interval_size]));
    }

    return calcTwistWithCov(transform_twists);
}


geometry_msgs::Twist CalibrateAction::getTwistFromTransforms(tf::StampedTransform trans1, tf::StampedTransform trans2)
{
    geometry_msgs::Twist result_twist;
    result_twist.linear = trans2.getOrigin() - trans1.getOrigin();
    result_twist.angular.z = tf::getYaw(trans2.getRotation() - trans1.getRotation());
    result_twist.angular.x = 0;
    result_twist.angular.y = 0;

    return result_twist;
}

