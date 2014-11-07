#ifndef _CALIBRATE_TWIST_H_
#define _CALIBRATE_TWIST_H_


#include <ros/ros.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <calibrate_twist/CalibrateAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include <tf/transform_listener.h>

using namespace Eigen;

class CalibrateAction
{
protected:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub;    
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<calibrate_twist::CalibrateAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  calibrate_twist::CalibrateFeedback feedback_;
  calibrate_twist::CalibrateResult result_;

  ros::Time calibration_start;
  ros::Time calibration_end;

  tf::TransformListener* listener;

  //geometry_msgs::TwistWithCovariance CalibrateAction::calcTwistWithCov(std::vector<geometry_msgs::Twist> twists);
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<geometry_msgs::Twist> twists);
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twistsWC );
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<nav_msgs::Odometry> odos);
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<nav_msgs::Odometry::ConstPtr> odos_ptr);

  geometry_msgs::TwistWithCovariance estimateTwWithCovFromTrajectory(std::vector<tf::StampedTransform> transforms);
  geometry_msgs::Twist calcTwistFromTransform(tf::Transform _transform, ros::Duration _dur);

public:

  CalibrateAction(std::string name);

  ~CalibrateAction(void);

  void executeCB(const calibrate_twist::CalibrateGoalConstPtr &goal);


};


//------------ Parameter Variables--------------

double odo_cache_depths; // how many values of the odometry are gonna be stored
double stability_timeout; // maximum time in seconds until a stability in speed has to be reached
double stability_intervalDuration; // defines in seconds how long in the past the interval of do values reaches, to check stability
double stability_xThreshold; // defines the maximal covariance on x lin-axis which is tolerated as stable
double stability_zThreshold; // defines the maximal covariance on z rot-axis which is tolerated as stable
double calibration_calc_interval; // value in seconds how long one calculated interval of the calibration should be
std::string tfFixedFrame; // defines the fixed frame for the transform lookup call
std::string robotFrame; // defines the robot frame for the transform lookup call
double minStabilityDuration; // time in seconds for how long stability criteria must be met
int transforms_interval_size;

#endif

