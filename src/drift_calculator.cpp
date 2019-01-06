//
// amui dyawのdrift誤差を計算
//
//
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "ceres_msgs/AMU_data.h"
#include "ceres_msgs/AMU_data.h"
// #include "complement/drift_calculator.h"
#include "drift_calculator.h"

driftCalculator::driftCalculator()
	: vel(0.0), dyaw(0.0), dyaw_ave(0.0), cnt_dyaw(0)
{
	sub_wheel = n.subscribe<nav_msgs::Odometry>("/odom", 1, &driftCalculator::wheelCallback, this);
	sub_gyro = n.subscribe<sensor_msgs::Imu>("/imu/data", 1, &driftCalculator::gyroCallback, this);
	pub_drift = n.advertise<std_msgs::Float64>("/AMU/drift_error/dyaw", 1);
}

void driftCalculator::wheelCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vel = msg->twist.twist.linear.x;
}

void driftCalculator::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	// dyaw = msg->dyaw * M_PI / 180;
	dyaw = msg->angular_velocity.z;
	calc_dyaw_ave();
}

void driftCalculator::calc_dyaw_ave()
{
	// if(fabs(vel) < 1e-8 && fabs(dyaw) < 0.5){
	if(fabs(dyaw) < 0.5){
		dyaw_ave = (dyaw_ave * cnt_dyaw + dyaw) / (cnt_dyaw + 1);
		cnt_dyaw++;
	}else{
		dyaw_ave = 0.0;
		cnt_dyaw = 0;
	}
}

double driftCalculator::driftErrorGetter(){ return dyaw_ave; }

double driftCalculator::countGetter(){ return cnt_dyaw; }

void driftCalculator::publisher()
{
	std_msgs::Float64 drift_error;

	drift_error.data = dyaw_ave;
	pub_drift.publish(drift_error);
}

