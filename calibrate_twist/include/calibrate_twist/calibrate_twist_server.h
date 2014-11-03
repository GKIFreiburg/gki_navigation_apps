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

  void calcTwistWithCov(std::vector<geometry_msgs::Twist> twists, geometry_msgs::TwistWithCovariance* resultTwist);
  void calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twists, geometry_msgs::TwistWithCovariance* resultTwist);
public:

  CalibrateAction(std::string name);

  ~CalibrateAction(void);

  void executeCB(const calibrate_twist::CalibrateGoalConstPtr &goal);

};


//------------ Parameter Variables--------------

// size of the overall voronoi grid
double odo_cache_depths;
double stability_timeout;
double stability_intervalDuration;



#endif

