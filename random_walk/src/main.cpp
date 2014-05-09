#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include <angles/angles.h>

// global parameter
double speed_angle_min = 0.2;
double speed_angle_max = 0.5;
double speed_vel_min = 0.1;
double speed_vel_max = 0.4;
double speed_angle_free = 0.25;
double distance_drive_min = 0.5;
double distance_drive_max = 1;
double distance_angle_min = 0.15;
double distance_angle_max = 2.09;
double distance_obstacle_front = 0.4;
double distance_obstacle_side = 0.3;
double distance_robot_offset = 0.1;
double threshold_angle = 0.05;

//global variables
tf::TransformListener *listener;
ros::Time last_laser_msg;
bool angle_reached;
bool distance_reached;
bool obstacle;
bool drive_configured;
bool rotate_configured;

//globals for states
double distance_to_drive;
double angle_to_rotate;
double cmd;
tf::StampedTransform start_robot_pose;


ros::Publisher pub;
ros::Publisher led_pub;
ros::Publisher sound_pub;


void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
	std::vector<tf::Stamped<tf::Point> > points;

	for (int i = 0; i < scan_in->ranges.size(); i++) {
		tf::Point point;

		if(scan_in->ranges.at(i) <= scan_in->range_min || scan_in->ranges.at(i) >= scan_in->range_max){
            continue;
        }
        point.setX(scan_in->ranges.at(i)* cos(scan_in->angle_min+ (i * scan_in->angle_increment)));
        point.setY(scan_in->ranges.at(i)* sin(scan_in->angle_min+ (i * scan_in->angle_increment)));
        tf::Stamped<tf::Point> stamped_point_in = tf::Stamped<tf::Point>(point,
                scan_in->header.stamp, scan_in->header.frame_id);

        tf::Stamped<tf::Point> stamped_point_out = tf::Stamped<tf::Point>();

        try {
            listener->transformPoint("base_link",stamped_point_in,stamped_point_out);
        }
        catch (tf::TransformException  & ex) {
            ROS_ERROR_THROTTLE(3.0, "\n%s: Error: %s \n", __func__, ex.what());
            return;
        }

        if(stamped_point_out.getX() >= distance_robot_offset)
            points.push_back(stamped_point_out);
    }

    if(points.empty()) {
        ROS_ERROR_THROTTLE(3.0, "No transformable points in scan");
        return;
    }

	last_laser_msg = ros::Time::now();

	obstacle = false;

	for (int i = 0; i < points.size(); i++){
        //ROS_INFO("check pt at %f %f", points.at(i).getX(), points.at(i).getY());
		if(fabs(points.at(i).getY()) <  distance_obstacle_side){
			if(points.at(i).getX() < distance_obstacle_front){
				obstacle = true;
			}
		}
	}
	if(obstacle) {
		kobuki_msgs::Sound sound;
		sound.value = sound.BUTTON;
		sound_pub.publish(sound);
	}
}

void get_parameter() {
	ros::NodeHandle privNh("~");

	if (!privNh.hasParam("speed_angle_min"))
		ROS_WARN("did not find speed_angle_min parameter, assuming %f\n",
				speed_angle_min);
	else
		privNh.getParam("speed_angle_min", speed_angle_min);

	if (!privNh.hasParam("speed_angle_max"))
		ROS_WARN("did not find speed_angle_max parameter, assuming %f\n",
				speed_angle_max);
	else
		privNh.getParam("speed_angle_max", speed_angle_max);

	if (!privNh.hasParam("speed_vel_min"))
		ROS_WARN("did not find speed_vel_min parameter, assuming %f\n",
				speed_vel_min);
	else
		privNh.getParam("speed_vel_min", speed_vel_min);

	if (!privNh.hasParam("speed_angle_free"))
		ROS_WARN("did not find speed_angle_free parameter, assuming %f\n",
				speed_angle_free);
	else
		privNh.getParam("speed_angle_free", speed_angle_free);

	if (!privNh.hasParam("speed_vel_max"))
		ROS_WARN("did not find speed_vel_max parameter, assuming %f\n",
				speed_vel_max);
	else
		privNh.getParam("speed_vel_max", speed_vel_max);

	if (!privNh.hasParam("distance_drive_min"))
		ROS_WARN("did not find distance_drive_min parameter, assuming %f\n",
				distance_drive_min);
	else
		privNh.getParam("distance_drive_min", distance_drive_min);

	if (!privNh.hasParam("distance_drive_max"))
			ROS_WARN("did not find distance_drive_max parameter, assuming %f\n",
					distance_drive_max);
	else
		privNh.getParam("distance_drive_max", distance_drive_max);

	if (!privNh.hasParam("distance_angle_min"))
				ROS_WARN("did not find distance_angle_min parameter, assuming %f\n",
						distance_angle_min);
	else
		privNh.getParam("distance_angle_min", distance_angle_min);

	if (!privNh.hasParam("distance_angle_max"))
				ROS_WARN("did not find distance_angle_max parameter, assuming %f\n",
						distance_angle_max);
	else
		privNh.getParam("distance_angle_max", distance_angle_max);

	if (!privNh.hasParam("distance_robot_offset"))
				ROS_WARN("did not find distance_robot_offset parameter, assuming %f\n",
						distance_robot_offset);
	else
		privNh.getParam("distance_robot_offset", distance_robot_offset);

	if (!privNh.hasParam("distance_obstacle_front"))
		ROS_WARN(
				"did not find distance_obstacle_front parameter, assuming %f\n",
				distance_obstacle_front);
	else
		privNh.getParam("distance_obstacle_front", distance_obstacle_front);

	if (!privNh.hasParam("distance_obstacle_side"))
		ROS_WARN("did not find distance_obstacle_side parameter, assuming %f\n",
				distance_obstacle_side);
	else
		privNh.getParam("distance_obstacle_side", distance_obstacle_side);

	if (!privNh.hasParam("threshold_angle"))
				ROS_WARN("did not find threshold_angle parameter, assuming %f\n",
						threshold_angle);
	else
		privNh.getParam("threshold_angle", threshold_angle);
}


double fRand(double fMin, double fMax)
{
    // TODO drand48, srand48
    srand48(time(NULL));
    return fMin + drand48() * (fMax- fMin);
	
}


tf::StampedTransform get_robot_pose(){
    tf::StampedTransform current_robot_pose;
    listener->waitForTransform("odom","base_link",ros::Time::now(),ros::Duration(0));
    try {
        listener->lookupTransform("odom","base_link",ros::Time(0),current_robot_pose);
    }catch (tf::TransformException & ex){
        ROS_ERROR("\nError: %s \n",ex.what());
        return current_robot_pose;
    }
    return current_robot_pose;
}

void drive(){
    tf::Transform relative_odo;
    relative_odo = start_robot_pose.inverse() * get_robot_pose();
	double driven_distance = relative_odo.getOrigin().length();
    ROS_DEBUG_THROTTLE(0.5, "driven_distance: %f", driven_distance);
	
    kobuki_msgs::Led led;
    led.value = led.GREEN;
    led_pub.publish(led);


    geometry_msgs::Twist drive_msg;
	drive_msg.linear.x = cmd;
    drive_msg.angular.z = 0;

    pub.publish(drive_msg);


    if(driven_distance >= distance_to_drive){
        distance_reached = true;
        drive_configured=false;

    } else {
        //reached goal
        distance_reached = false;
        drive_configured = true;
    }
}


void rotate(){
    
    tf::StampedTransform current_robot_pose = get_robot_pose();

	geometry_msgs::Twist drive_msg;

    tf::Transform relative_odo;
    relative_odo = start_robot_pose.inverse() * current_robot_pose;
	double angle_difference = angles::normalize_angle(tf::getYaw(relative_odo.getRotation()) - angle_to_rotate);


    if(angle_difference > 0 && cmd > 0){
        cmd = -cmd;
    }
    //ROS_INFO("angle_difference %f   speed: %f", angle_difference,cmd);

    kobuki_msgs::Led led;
    led.value = led.ORANGE;
    led_pub.publish(led);

    drive_msg.linear.x = 0;
    drive_msg.angular.z = cmd;
    pub.publish(drive_msg);


    if(fabs(angle_difference) < threshold_angle ){
        angle_reached = true;
        rotate_configured = false;
    } else {
        angle_reached = false;
        rotate_configured = true;
    }
}

void get_drive_config(){
	distance_to_drive =  fRand(distance_drive_min, distance_drive_max);
	cmd = fRand(speed_vel_min,speed_vel_max);

    start_robot_pose = get_robot_pose();
    ROS_INFO("Config drive dist: %f vel: %f", distance_to_drive, cmd);
    drive_configured = true;
    distance_reached=false;
}

void get_rotate_config(){

	angle_to_rotate = fRand(-distance_angle_max,distance_angle_max);
    if(fabs(angle_to_rotate) < distance_angle_min){
        if(angle_to_rotate < 0 ){
            angle_to_rotate = -distance_angle_min;
        } else{ 
            angle_to_rotate = distance_angle_min;

        }
    }

    cmd = fRand(speed_angle_min,speed_angle_max);

    start_robot_pose = get_robot_pose();
    angle_to_rotate = angles::normalize_angle( tf::getYaw(start_robot_pose.getRotation()) + angle_to_rotate);

    rotate_configured = true;
    angle_reached=false;
    ROS_INFO("Config rotate angle: %f vel: %f", angle_to_rotate, cmd);

}

void drive_free(){
    geometry_msgs::Twist drive_msg;
    drive_msg.linear.x = 0;
    drive_msg.angular.z = speed_angle_free;
    pub.publish(drive_msg);
    if(!obstacle){
        drive_configured = false;
        rotate_configured = false;
    }
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "random_walk");
	ros::NodeHandle nh;

    ROS_INFO("random_walk started.");

	// get parameters
	get_parameter();

	listener = new tf::TransformListener();

	ros::Subscriber sub_laser = nh.subscribe("base_scan_filtered", 1,
			scan_callback);
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	led_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led2",1);
	sound_pub = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound",1);

	// init transitions
    distance_reached = false;
    angle_reached = false;
    drive_configured = false;
    rotate_configured = false;

    // define states
    enum State {DRIVE=1,GET_DRIVE_CONFIG,ROTATE,GET_ROTATE_CONFIG,DRIVE_FREE}; 
    State state = DRIVE_FREE;

    ros::Rate loop_rate(20);
    while(ros::ok()) {
		ros::spinOnce();
		ros::Time now = ros::Time::now();

		if(	(now - last_laser_msg) > ros::Duration(0.2)){
			ROS_ERROR_THROTTLE(5.0, "Laser msgs too slow");
			kobuki_msgs::Led led;
			led.value = led.RED;
			led_pub.publish(led);
		} else {
            switch(state){
                case DRIVE:
                    drive();
            break;
                case GET_DRIVE_CONFIG:
                    get_drive_config();
            break;
                case ROTATE:
                    rotate();
            break;
                case GET_ROTATE_CONFIG:
                    get_rotate_config();
            break;
                case DRIVE_FREE:
                    drive_free();
            break;
            default:
                //invalid state
                ROS_ERROR("Reached invalid state - should never happen!");
            }


            // transitions
            if(state == GET_DRIVE_CONFIG && drive_configured && !obstacle && !distance_reached){
                state = DRIVE;
            }else if(state == DRIVE && !rotate_configured && !obstacle && distance_reached){
                state = GET_ROTATE_CONFIG;
            }else if((state == DRIVE || state == ROTATE || state == GET_ROTATE_CONFIG || state == GET_DRIVE_CONFIG || state == DRIVE_FREE) && obstacle){
                state = DRIVE_FREE;
            }else if(state == GET_ROTATE_CONFIG && rotate_configured && !obstacle && !angle_reached){
                state = ROTATE;
            }else if(state == ROTATE && !drive_configured && !obstacle && angle_reached){
                state = GET_DRIVE_CONFIG;
            }else if(state == DRIVE_FREE  && !obstacle){
                state = GET_DRIVE_CONFIG;
            }else {
                //stay in state
            }
    
           // ROS_INFO_THROTTLE(0.5,"Current State: %d Transition: drive_configured %d distance_reached %d rotate_configured %d angle_reached %d obstacle %d", state, drive_configured, distance_reached, rotate_configured, angle_reached, obstacle);


		}
        loop_rate.sleep();
	}

	return 0;
}
