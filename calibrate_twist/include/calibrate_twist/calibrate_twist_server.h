#ifndef _CALIBRATE_TWIST_H_
#define _CALIBRATE_TWIST_H_


#include <ros/ros.h>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <calibrate_twist/CalibrateAction.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "dynamicvoronoi/dynamicvoronoi.h"

#include "trajectory.h"

#include <angles/angles.h>

using namespace Eigen;


class CalibrateAction
{
protected:

  ros::NodeHandle nh_;
  ros::Publisher twist_pub;    
  ros::Publisher calcTraj_pub;
  ros::Publisher estTraj_pub;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<calibrate_twist::CalibrateAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  calibrate_twist::CalibrateFeedback feedback_;
  calibrate_twist::CalibrateResult result_;
  calibrate_twist::CalibrateGoal goal_;

  ros::Time calibration_start;
  ros::Time calibration_end;
  std::vector<tf::StampedTransform> movement_transforms;   // the transform vector that we fill, using tf during the calibration run
  geometry_msgs::Twist zero_twist;
  geometry_msgs::TwistWithCovariance twistWCFromOdometry; // temporary result before copied to action result
  geometry_msgs::TwistWithCovariance twistWCFromTf; // temporary result before copied to action result

  bool success;

  message_filters::Cache<nav_msgs::Odometry>* odo_cache;
  tf::TransformListener* listener;
  costmap_2d::Costmap2DROS* cost_map;

  costmap_2d::Costmap2D costmap_; // element for voronoi updating

  ros::Publisher voronoi_pub;
  DynamicVoronoi voronoi_;

  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<geometry_msgs::Twist> twists);
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<geometry_msgs::TwistWithCovariance> twistsWC );
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<nav_msgs::Odometry> odos);
  geometry_msgs::TwistWithCovariance calcTwistWithCov(std::vector<nav_msgs::Odometry::ConstPtr> odos_ptr);

  geometry_msgs::TwistWithCovariance estimateTwWithCovFromTrajectory(std::vector<tf::StampedTransform> transforms);
  geometry_msgs::Twist calcTwistFromTransform(tf::Transform _transform, ros::Duration _dur);

  bool bringupGoalSpeed();
  bool checkOdoConsistency(bool &first_stability, ros::Time &first_stability_time);
  bool startCalibrationRun();
  void calculateResult();

  void visualizeVoronoi();

  //void updateVoronoi(const nav_msgs::GridCells::ConstPtr& msg); // function to use with costmapCB
  void updateVoronoi();

  bool checkTrajectory(Trajectory& traj);

  void visualize_trajectory(Trajectory& traj);

  // checks if a path is clear from current position for given speed, goal speed, time and accel
  bool checkPath(double vx, double vy, double vtheta, double  sim_time_, double vx_samp, double vy_samp, double vtheta_samp,
                 double acc_x, double acc_y, double acc_theta);

  // checks if a path is clear from a given position for given speed, goal speed, time and accel
  bool checkPath(double xPos, double yPos, double thetaPos, double vx, double vy, double vtheta, double  sim_time_, double vx_samp, double vy_samp, double vtheta_samp,
                 double acc_x, double acc_y, double acc_theta);

  // checks path when starting from zero speed with unknown speed up time
  bool checkPath(double vx, double vy, double vtheta, double vx_samp, double vy_samp, double vtheta_samp,
                 double acc_x, double acc_y, double acc_theta, Trajectory *resultTraj = NULL);

  double getDistanceAtPose(const tf::Pose & pose, bool* in_bounds) const;

  bool estimateFullPathIsClear(double vx_samp, double vy_samp, double vtheta_samp, double  sim_time_,
                               double acc_x, double acc_y, double acc_theta);


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
  * * Neither the name of the Willow Garage nor the names of its
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

  void generateTrajectory(
          double x, double y, double theta,
          double vx, double vy, double vtheta, double  sim_time_, double vx_samp, double vy_samp, double vtheta_samp,
          double acc_x, double acc_y, double acc_theta, Trajectory& traj);

  /**
 * @brief Compute x position based on velocity
 * @param xi The current x position
 * @param vx The current x velocity
 * @param vy The current y velocity
 * @param theta The current orientation
 * @param dt The timestep to take
 * @return The new x position
 */
 inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
 return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
 }
 /**
 * @brief Compute y position based on velocity
 * @param yi The current y position
 * @param vx The current x velocity
 * @param vy The current y velocity
 * @param theta The current orientation
 * @param dt The timestep to take
 * @return The new y position
 */
 inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
 return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
 }
 /**
 * @brief Compute orientation based on velocity
 * @param thetai The current orientation
 * @param vth The current theta velocity
 * @param dt The timestep to take
 * @return The new orientation
 */
 inline double computeNewThetaPosition(double thetai, double vth, double dt){
 return thetai + vth * dt;
 }

 //compute velocity based on acceleration
 /**
  * @brief  Compute velocity based on acceleration
  * @param vg The desired velocity, what we're accelerating up to
  * @param vi The current velocity
  * @param a_max An acceleration limit
  * @param  dt The timestep to take
  * @return The new velocity
  */
 inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
   if((vg - vi) >= 0)
     return std::min(vg, vi + a_max * dt);
   return std::max(vg, vi - a_max * dt);
 }


public:

  CalibrateAction(std::string name);

  ~CalibrateAction(void);

  void executeCB(const calibrate_twist::CalibrateGoalConstPtr &goal);

  //void costmapCB(const nav_msgs::GridCells::ConstPtr& msg); // not used as for now unclear why callback isn't able to be registered
};

//------------ Parameter Variables--------------

// How to deal with parameters in class structure? private? public?

double odo_cache_depths; // how many values of the odometry are gonna be stored
double stability_timeout; // maximum time in seconds until a stability in speed has to be reached
double stability_intervalDuration; // defines in seconds how long in the past the interval of do values reaches, to check stability
double stability_xThreshold; // defines the maximal covariance on x lin-axis which is tolerated as stable
double stability_zThreshold; // defines the maximal covariance on z rot-axis which is tolerated as stable
double calibration_calc_interval; // value in seconds how long one calculated interval of the calibration should be
std::string cmdVelTopic; // the topic the command values are being published to
std::string tfFixedFrame; // defines the fixed frame for the transform lookup call
std::string robotFrame; // defines the robot frame for the transform lookup call
double minStabilityDuration; // time in seconds for how long stability criteria must be met
int transforms_interval_size;
std::string cal_costmap; // the name of the local costmap used for avoiding crashes in calibration runs

double traj_sim_granularity_; // the distance between trajectory simulation points // "The granularity with which to check for collisions along each trajectory in meters"
double traj_dist_threshold; // threshold in meters how close the robot may get to an obstacle

double accel_max_x; //maximum acceleration values of the robot
double accel_max_y;
double accel_max_theta;

double min_time_clear; // time in seconds that the trajectory is checked further than it's actually driven

#endif

