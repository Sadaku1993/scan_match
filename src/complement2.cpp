//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <time_util/stopwatch.h>
#include <Eigen/Core>
#include "ceres_msgs/AMU_data.h"


using namespace std;

class odomPublisher
{
	ros::NodeHandle n;
	ros::Subscriber sub_wheel;
	ros::Subscriber sub_gyro;
	ros::Publisher pub_odom;
	tf::TransformBroadcaster odom_broadcaster;
	double x;
	double y;
	double yaw;
	double vel;
	double dyaw;
	double drift_dyaw;
	// Stopwatch sw;
	ros::Time current_time, last_time;

	public:
	odomPublisher();
	void wheelCallback(const nav_msgs::Odometry::ConstPtr&);
	// void gyroCallback(const ceres_msgs::AMU_data::ConstPtr&);
	void gyroCallback(const sensor_msgs::Imu::ConstPtr&);
	void complement();
	void publisher();
};

odomPublisher::odomPublisher()
	: x(0.0), y(0.0), yaw(0.0), vel(0.0), dyaw(0.0)
{
	// sub_wheel = n.subscribe<nav_msgs::Odometry>("/odom", 1, &odomPublisher::wheelCallback, this);
	sub_wheel = n.subscribe<nav_msgs::Odometry>("/odom", 1, &odomPublisher::wheelCallback, this);
	sub_gyro = n.subscribe<sensor_msgs::Imu>("/imu/data", 1, &odomPublisher::gyroCallback, this);
	pub_odom = n.advertise<nav_msgs::Odometry>("/odom/complement", 1);

	double drift = 0.0;
	n.getParam("/dyaw/drift", drift);
	if(drift){
		drift_dyaw = drift;
	}else{
		drift_dyaw = 0.269380006;
	}
	cout << "drift_dyaw: " << drift_dyaw << endl;
	// sw.start();
	current_time = ros::Time::now();
	last_time= ros::Time::now();
}

void odomPublisher::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vel = msg->twist.twist.linear.x;
}

void odomPublisher::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	// dyaw = -(msg->dyaw - 0.295030001) * M_PI / 180; //left
	// dyaw = -(msg->dyaw - 0.275030001) * M_PI / 180; //org
	// dyaw = -(msg->dyaw - 0.259380006) * M_PI / 180; //right
	dyaw = msg->angular_velocity.z - drift_dyaw;
	// dyaw = msg->dyaw * M_PI / 180;
	// yaw = (msg->yaw + 0.275030001) * M_PI / 180;
	// yaw = msg->yaw * M_PI / 180;
}

void odomPublisher::complement()
{
	// float dt = sw.getTime();
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = ros::Time::now();

	double dist = vel * dt;
	// vel = 0.0;

	yaw += dyaw * dt;
	// dyaw /= 1.01;

	// while(yaw > M_PI) yaw -= 2*M_PI;
	// while(yaw < -M_PI) yaw += 2*M_PI;
	x += dist * cos(yaw);
	y += dist * sin(yaw);

	// sw.reset();
}

void odomPublisher::publisher()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	// geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = ros::Time::now();

	// odom_trans.header.frame_id = "map";
	// odom_trans.child_frame_id = "matching_base_link";

	// odom_trans.transform.translation.x = x;
	// odom_trans.transform.translation.y = y;
	// odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation = odom_quat;
    //
	// odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	// odom.header.frame_id = "map";
	odom.header.frame_id = "odom";

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	pub_odom.publish(odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "complement");

	odomPublisher op;

	ros::Rate loop_rate(60);

	while(ros::ok()){
		op.complement();
		op.publisher();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

