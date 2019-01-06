#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <AMU_data.h>
#include <sensor_msgs/PointCloud.h>
#include <time_util/stopwatch.h>
#include <Eigen/Core>

using namespace std;

float x = 0.0;
float y = 0.0;
float yaw = 0.0;
float odom_v = 0.0;
float yawrate = 0.0;
float pitchrate = 0.0;
float rollrate = 0.0;
float amu_roll = 0.0;
float amu_pitch = 0.0;
float amu_yaw = 0.0;
double dist = 0.0;
bool point_flag = false;

// Stopwatch t;
// t.start();
// ros::Time current_time;
// ros::Time last_time;
// current_time = ros::Time::now();
// last_time = ros::Time::now();

void position_callback(sensor_msgs::PointCloud::ConstPtr msg)
{
	x = msg->points[0].x;
	y = msg->points[0].y;
	point_flag = true;
}

void amu_callback(ceres_msgs::AMU_data::ConstPtr msg)
{
	//rate
	rollrate  =  M_PI * msg->droll / 180.0;
	pitchrate =  M_PI * msg->dpitch / 180.0;
	yawrate   =  M_PI * (-msg->dyaw + 0.275030001) / 180.0;
	// yawrate   = - M_PI * msg->dyaw / 180.0 + OFFSET;
	// yawrate *= 0.54;

	//absolute
	amu_roll = M_PI * msg->roll / 180.0 + M_PI;
	amu_pitch = M_PI * msg->pitch / 180.0;
	amu_yaw = M_PI * msg->yaw / 180.0;

	if(amu_roll >  M_PI)amu_roll -= 2.0*M_PI;
	if(amu_roll < -M_PI)amu_roll += 2.0*M_PI;
	// amu_flag = true;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg)
{
	odom_v = msg->twist.twist.linear.x;
	//	cout << "odom_v = " << odom_v << endl;
}

void complement(float dt)
{
	cout<<"DT="<<dt<<endl;
	yaw += yawrate * dt;
	if(!point_flag){
		dist = odom_v * dt;
		x += dist * cos(yaw);
		y += dist * sin(yaw);
	}
	// t.reset();
	point_flag = false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "position_publisher");
	ros::NodeHandle n;
	ros::Rate r(100);

  ros::Subscriber amu_sub = n.subscribe("/AMU_data", 1, amu_callback);
	ros::Subscriber odm_sub = n.subscribe("/tinypower/odom", 1, odom_callback);
	ros::Subscriber point_sub = n.subscribe("/now_position", 1, position_callback);

	ros::Publisher pub_pose = n.advertise<geometry_msgs::Point>("position_comp",10);
	ros::Publisher pub_pose2 = n.advertise<sensor_msgs::PointCloud>("position_comp_vis",10);

	geometry_msgs::Point32 pc;
	sensor_msgs::PointCloud pc_now;
	pc_now.header.frame_id = "/map";
	// tm.start();
	Stopwatch tm;
	tm.start();

	while(ros::ok()){

		float dt = tm.getTime();
		cout<<"dt="<<dt<<endl;
		complement(dt);
		pc.x = x;
		pc.y = y;
		pc.z = 0.0;
		pc_now.points.push_back(pc);
		// pc_now.x = x;
		// pc_now.y = y;
		// pc_now.z = 0.0;
		pub_pose.publish(pc);
		pub_pose2.publish(pc_now);
		pc_now.points.clear();
		tm.reset();

		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
