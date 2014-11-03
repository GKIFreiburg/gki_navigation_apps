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

    // read all necessary parameters
    nh_.getParamCached("/calibrate_twist/odo_cache_depths", odo_cache_depths);
    nh_.getParamCached("/calibrate_twist/stability_timeout", stability_timeout);
    nh_.getParamCached("/calibrate_twist/stability_intervalDuration", stability_intervalDuration);


    //ros::Subscriber sub = nh_.subscribe("/odom", odo_cache_depths, someCallback);
    message_filters::Subscriber<nav_msgs::Odometry> sub(nh_, "/odom", 1);
    message_filters::Cache<nav_msgs::Odometry> odo_cache(sub, odo_cache_depths);

    twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate r(10);

    geometry_msgs::Twist zero_twist; // how to initialize as zero?
    zero_twist.angular.x = 0; // necessary?
    zero_twist.angular.y = 0; // necessary?
    zero_twist.angular.z = 0; // necessary?
    zero_twist.linear.x = 0; // necessary
    zero_twist.linear.y = 0; // necessary
    zero_twist.linear.z = 0; // necessary

    ros::Time start = ros::Time::now();

    // publish info to the console for the user
    ROS_INFO("%s: Executing, running calibration run for %f seconds with linear speed %f, angular speed %f", action_name_.c_str(), goal->duration.toSec(), goal->twist_goal.linear.x, goal->twist_goal.angular.z);

    // bringing the robot to the goal speed
    while((ros::Time::now().toSec()) < (start.toSec() + stability_timeout))
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

        //test only
        nav_msgs::Odometry::ConstPtr odo_ptr = odo_cache.getElemBeforeTime(odo_cache.getLatestTime());
        if(odo_ptr)
            ROS_INFO("constance check: %f on x-axis, %f on z-rot", odo_ptr->twist.twist.linear.x,odo_ptr->twist.covariance[35]);


        // get a odometry interval out of the cache and check for constance of the contained twists
        //std::vector<nav_msgs::Odometry> odo_interval = odo_cache.getInterval((odo_cache.getLatestTime()-ros::Duration(stability_intervalDuration)),odo_cache.getLatestTime());
        //std::vector<geometry_msgs::TwistWithCovariance> twist_interval;
        //for(unsigned int i=0; i<odo_interval.size(); i++)
        //{
//            twist_interval.push_back(odo_interval[i].twist);
//        }
        //geometry_msgs::TwistWithCovariance result_twist;
//        calcTwistWithCov(twist_interval,&result_twist);
        //ROS_INFO("constance check: %f on x-axis, %f on z-rot", result_twist.covariance[0],result_twist.covariance[35]);

        r.sleep();
    }
    if(!stability_reached)
    {
        twist_pub.publish(zero_twist); // safety first, stop robot
        as_.setAborted();
        success = false;
    }

    start = ros::Time::now();

    if(stability_reached)
    {
    // starting calibration run
    while((ros::Time::now().toSec()) < (start.toSec() + goal->duration.toSec()))
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
    }

    // checking constance of achieved velocity

    // starting calibration run for given duration

    // calculating the result


    geometry_msgs::TwistWithCovariance tw;
    std::vector<geometry_msgs::Twist> twists_vector;
    twists_vector.push_back(goal->twist_goal);
    CalibrateAction::calcTwistWithCov(twists_vector, &tw);

    if(success)
    {
      twist_pub.publish(zero_twist); // safety first, stop robot
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
    Map<Matrix<double,6,6,RowMajor> >(resultC, 6,6) = covMat;
}




