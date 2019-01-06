//
// wheelのv とimuのyawからodomをpublish
//
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <time_util/stopwatch.h>
#include <Eigen/Core>


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
	double init_yaw;
	double offset_dyaw;
	double vel;
	double dyaw;
	double qr;
	double qp;
	double qy;
	// Stopwatch sw;
	ros::Time current_time, last_time;

	public:
	odomPublisher();
	void wheelCallback(const nav_msgs::Odometry::ConstPtr&);
	void gyroCallback(const sensor_msgs::Imu::ConstPtr&);
	void complement();
	void publisher();
};

odomPublisher::odomPublisher()
	: x(0.0), y(0.0), yaw(0.0), init_yaw(0.0), offset_dyaw(0.0001) //sq2/d_kan_indoor->0.00125
{
	sub_wheel = n.subscribe<nav_msgs::Odometry>("/odom", 1, &odomPublisher::wheelCallback, this);
	// sub_gyro = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &odomPublisher::gyroCallback, this);
	sub_gyro = n.subscribe<sensor_msgs::Imu>("/imu/data/filtered", 1, &odomPublisher::gyroCallback, this);
	pub_odom = n.advertise<nav_msgs::Odometry>("/odom/complement/filtered", 1);
	// sw.start();
	current_time = ros::Time::now();
	last_time= ros::Time::now();
}

void odomPublisher::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vel = msg->twist.twist.linear.x;
	// cout<<"vel="<<vel<<endl;
}

void odomPublisher::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3(quat).getRPY(qr, qp, qy);
	// dyaw = msg->angular_velocity.x;
	dyaw = msg->angular_velocity.z - offset_dyaw;
	// cout<<"dyaw="<<dyaw<<endl;
	// cout<<"qy="<<qy<<endl;
}

void odomPublisher::complement()
{
	// float dt = sw.getTime();
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = ros::Time::now();

	double dist = vel * dt;

	// yaw += dyaw * dt;
	yaw = qy - init_yaw;
	x += dist * cos(yaw);
	y += dist * sin(yaw);

	// cout<<"x="<<x<<endl;
	// cout<<"y="<<y<<endl;
	// sw.reset();
}

void odomPublisher::publisher()
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	// geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = ros::Time::now();
    //
	// odom_trans.header.frame_id = "map";
	// // odom_trans.header.frame_id = "odom";
	// // odom_trans.child_frame_id = "matching_base_link";
	// odom_trans.child_frame_id = "base_link";
    //
	// odom_trans.transform.translation.x = x;
	// odom_trans.transform.translation.y = y;
	// odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation = odom_quat;
    //
	// odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.frame_id = "/map";
	/* odom.header.frame_id = "odom"; */
	/* odom.child_frame_id = "base_link"; */

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	pub_odom.publish(odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "complement_filtered");

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

