
#ifndef DRIFT_CALCULATOR_H
#define DRIFT_CALCULATOR_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "ceres_msgs/AMU_data.h"

class driftCalculator
{
	ros::NodeHandle n;
	ros::Subscriber sub_wheel;
	ros::Subscriber sub_gyro;
	ros::Publisher pub_drift;
	double vel;
	double dyaw;
	double dyaw_ave;
	int cnt_dyaw;

	void calc_dyaw_ave();

	public:
	driftCalculator();
	void wheelCallback(const nav_msgs::Odometry::ConstPtr&);
	// void gyroCallback(const ceres_msgs::AMU_data::ConstPtr&);
	void gyroCallback(const sensor_msgs::Imu::ConstPtr&);
	double driftErrorGetter();
	double countGetter();
	void publisher();
};

#endif

