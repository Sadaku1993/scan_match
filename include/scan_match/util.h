#ifndef _SM2_UTIL_H_
#define _SM2_UTIL_H_

#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>//ark...tuika
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
//#include <usr_msgs/Edge.h>
#include <Edge.h>


using namespace std;
using namespace Eigen;

int ask(const vector<string> &dat, const string &qry);

// void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointNormal> &cloud);
void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZINormal> &cloud);
void showXYZ(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud);
//###void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud);


// void show(ros::NodeHandle n, const string topic, sensor_msgs::PointCloud &pc);

// pcl::PointCloud<pcl::PointNormal> transformer(const pcl::PointCloud<pcl::PointNormal> &cloud_org, Eigen::Matrix4f m);
//###pcl::PointCloud<pcl::PointXYZ> transformer(const pcl::PointCloud<pcl::PointXYZ> &cloud_org, Eigen::Matrix4f m);
pcl::PointCloud<pcl::PointXYZINormal> transformer(const pcl::PointCloud<pcl::PointXYZINormal> &cloud_org, Eigen::Matrix4f m);
pcl::PointCloud<pcl::PointXYZ> transformerXYZ(const pcl::PointCloud<pcl::PointXYZ> &cloud_org, Eigen::Matrix4f m);


inline double SIGN(double x) {return (x >= 0.0) ? +1.0 : -1.0;}
inline double NORM(double a, double b, double c, double d) {return sqrt(a * a + b * b + c * c + d * d);}

Eigen::Vector4d mat2quat(Eigen::Matrix3d mat);
Eigen::Vector4f mat2quat_f(Eigen::Matrix3f mat);

void mat2rpy(Matrix4f m, float &r, float &p, float &y);

Eigen::Matrix3d rpy2mat(double roll, double pitch, double yaw);

inline Matrix4f calcMat(float roll, float pitch, float yaw);

void tf_bro(Eigen::Matrix4f mat, string child, string parent);

Matrix3f quat2mat(float x,float y,float z,float w);




#include "impl/util.cpp"

#endif

