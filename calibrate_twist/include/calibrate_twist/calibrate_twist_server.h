#ifndef _CALIBRATE_TWIST_H_
#define _CALIBRATE_TWIST_H_


#include <ros/ros.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <calibrate_twist/CalibrateAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <nav_msgs/Odometry.h>

#include <calibrate_twist/eigen/Eigenvalues>
#include <calibrate_twist/eigen/Dense>

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

  tf::TransformListener* listener;

  //bool cache_flag;

  void calcTwistWithCov(std::vector<geometry_msgs::Twist> twists, geometry_msgs::TwistWithCovariance* resultTwist);
  void calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twists, geometry_msgs::TwistWithCovariance* resultTwist);
  //void odo_cacheCB(const nav_msgs::Odometry::ConstPtr &msg);

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

#endif

