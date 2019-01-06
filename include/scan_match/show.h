#ifndef _SHOW_H_

#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

int ask(const vector<string> &dat, const string &qry){
	for(size_t i=0;i<dat.size();i++){
		if(dat[i] == qry) return i;
	}
	return -1;
}

// void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointNormal> &cloud){
void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZINormal> &cloud){
//###void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud){
	static vector<string> dat;
	static vector<ros::Publisher> pub;
	int index = ask(dat, topic);
	if(index == -1){
		ros::Publisher tmp_pub = n.advertise<sensor_msgs::PointCloud2>(topic,1);
		pub.push_back(tmp_pub);
		dat.push_back(topic);
		index = (int)pub.size()-1;
	}
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(cloud, pc);
	pc.header.frame_id = "/map";
	pc.header.stamp = ros::Time::now();
	pub[index].publish(pc);
}

void showXYZ(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud){
//###void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud){
	static vector<string> dat;
	static vector<ros::Publisher> pub;
	int index = ask(dat, topic);
	if(index == -1){
		ros::Publisher tmp_pub = n.advertise<sensor_msgs::PointCloud2>(topic,1);
		pub.push_back(tmp_pub);
		dat.push_back(topic);
		index = (int)pub.size()-1;
	}
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(cloud, pc);
	pc.header.frame_id = "/map";
	pc.header.stamp = ros::Time::now();
	pub[index].publish(pc);
}
/*
void show(ros::NodeHandle n, const string topic, sensor_msgs::PointCloud &cloud){
	static vector<string> dat;
	static vector<ros::Publisher> pub;
	int index = ask(dat, topic);
	if(index == -1){
		ros::Publisher tmp_pub = n.advertise<sensor_msgs::PointCloud>(topic,1);
		pub.push_back(tmp_pub);
		dat.push_back(topic);
		index = (int)pub.size()-1;
	}
	cloud.header.frame_id = "/velodyne";
	cloud.header.stamp = ros::Time::now();
	pub[index].publish(cloud);
}
*/
//
// void show(ros::NodeHandle n, const string topic, sensor_msgs::PointCloud &pc){
// 	static ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>(topic,1);
// 	pc.header.frame_id = "/map";
// 	pc.header.stamp = ros::Time::now();
// 	pub.publish(pc);
// }
/*
class Classname{
private:

public:
	Classname();
	~Classname();
};
*/
//#include "impl/xxx.hpp"
#endif

