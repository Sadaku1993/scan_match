/*
 * src: loop_detector.cpp
 *
 * Author: s.shimizu
 * created   : 2015.1.10
 * lastupdate: 20xx.xx.xx
 *
 * memo: NodeとEdgeの繋がりをRvizに表示するsrc. 
 *
 */


#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <string>

#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point32.h>

#include <Eigen/Core>

//user defined
//#include <usr_msgs/Edge.h>
//#include <scan_match2/util.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/point_types.h>


const size_t DIST = 20;
const size_t MAX_NODE = 10000;
const char filename[] = "aft.csv";
const size_t M=10000;

//namespaces
using namespace std;	



void load(sensor_msgs::PointCloud &pc, const ros::Publisher &pub_pc, visualization_msgs::Marker &mk, const ros::Publisher &pub_mk, visualization_msgs::Marker &mk2, const ros::Publisher &pub_mk2, visualization_msgs::MarkerArray &mk_a, const ros::Publisher &pub_mk_a){
	FILE *fp;
	fp = fopen(filename,"r");
	if(fp == NULL){
		cout << "could not find file" << endl;
	}else{
		cout << "found." << endl;
	}

	pc.points.clear();
	mk.points.clear();
	mk2.points.clear();
	while(1){
		float data[7] = {0};
		char type[1000];
		if(fscanf(fp,"%s",type) == EOF)break;
		if(strcmp(type,"VERTEX_SE3:QUAT")==0){
			if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;

			geometry_msgs::Point32 p;
			p.x = data[0];
			p.y = data[1];
			p.z = data[2]+0.05;
			pc.points.push_back(p);
		}
		else if(strcmp(type,"EDGE_SE3:QUAT")==0){
			int num1 = 0, num2 = 0;
			if(fscanf(fp,"%d %d %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f", &num1, &num2) != 2)break;
			if(num1 == 0)continue;
			//	if(num1 == 1025)continue;
			if(abs(num2 - num1) < 2){
				geometry_msgs::Point p1, p2;
				p1.x = pc.points[num1].x;
				p1.y = pc.points[num1].y;
				p1.z = pc.points[num1].z;
				p2.x = pc.points[num2].x;
				p2.y = pc.points[num2].y;
				p2.z = pc.points[num2].z;
				mk2.points.push_back(p1);
				mk2.points.push_back(p2);
			}else{
				geometry_msgs::Point p1, p2;
				p1.x = pc.points[num1].x;
				p1.y = pc.points[num1].y;
				p1.z = pc.points[num1].z;
				p2.x = pc.points[num2].x;
				p2.y = pc.points[num2].y;
				p2.z = pc.points[num2].z;
				mk.points.push_back(p1);
				mk.points.push_back(p2);
			}
		}
	}

	for(size_t i=0;i<M;i++){
		char hoge[100];
		sprintf(hoge,"%5d",i);
		mk_a.markers[i].text = hoge;
		if(i<pc.points.size()){
			float height = 0.6;
			mk_a.markers[i].pose.position.x = pc.points[i].x;
			mk_a.markers[i].pose.position.y = pc.points[i].y;
			mk_a.markers[i].pose.position.z = pc.points[i].z + height;
		}else{
			mk_a.markers[i].pose.position.x = 0.0;
			mk_a.markers[i].pose.position.y = 0.0;
			mk_a.markers[i].pose.position.z = 0.0;
		}

	}


	fclose(fp);

	pub_pc.publish(pc);
	pub_mk.publish(mk);
	pub_mk2.publish(mk2);
	pub_mk_a.publish(mk_a);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "GRAPH_VIEWER"); //define node name.
	ros::NodeHandle n;
	ros::Rate roop(1);          //Set Rate[Hz].
	ros::Publisher pub_pc = n.advertise<sensor_msgs::PointCloud>("path",1);
	ros::Publisher pub_mk = n.advertise<visualization_msgs::Marker>("pair",1);
	ros::Publisher pub_mk_a = n.advertise<visualization_msgs::MarkerArray>("label",1);
	ros::Publisher pub_mk2 = n.advertise<visualization_msgs::Marker>("pair2",1);
	sensor_msgs::PointCloud pc;
	visualization_msgs::Marker mk;
	visualization_msgs::MarkerArray mk_a;
	visualization_msgs::Marker mk2;
	pc.header.frame_id = "/map";//"map"
	mk.header.frame_id = "/map";//"map"
	mk2.header.frame_id = "/map";//"map"

	mk.ns = "namespace";
	mk.id = 0;
	mk.type = visualization_msgs::Marker::LINE_LIST;
	mk.action = visualization_msgs::Marker::ADD;
	mk.scale.x = 0.2;//0.15
	mk.scale.y = 0.2;//0.1
	mk.scale.z = 0.2;//0.1
	mk.color.a = 1.0;
	mk.color.r = 1.0;
	mk.color.g = 0.0;
	mk.color.b = 1.0;

	mk2.ns = "namespace";
	mk2.id = 0;
	mk2.type = visualization_msgs::Marker::LINE_LIST;
	mk2.action = visualization_msgs::Marker::ADD;
	mk2.scale.x = 0.2;//0.15
	mk2.scale.y = 0.2;//0.1
	mk2.scale.z = 0.2;//0.1
	mk2.color.a = 1.0;
	mk2.color.r = 0.0;
	mk2.color.g = 1.0;
	mk2.color.b = 1.0;

	mk_a.markers.resize(M);
	for(size_t i=0;i<M;i++){	
		mk_a.markers[i].ns = "namespace";
		mk_a.markers[i].id = i;
		mk_a.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		mk_a.markers[i].action = visualization_msgs::Marker::ADD;
		mk_a.markers[i].color.a = 1.0;
		mk_a.markers[i].color.r = 0.0;
		mk_a.markers[i].color.g = 1.0;
		mk_a.markers[i].color.b = 0.0;
		mk_a.markers[i].scale.z = 0.75;
		mk_a.markers[i].header.frame_id = "/map";//"map"
	}

	cout << "check1" << endl;

	load(pc, pub_pc, mk, pub_mk, mk2, pub_mk2, mk_a,pub_mk_a);

	cout << "check2" << endl;


	while(ros::ok()){
		pc.header.stamp = ros::Time::now();
		mk.header.stamp = ros::Time::now();
		mk2.header.stamp = ros::Time::now();
		for(size_t i=0;i<M;i++)
			mk_a.markers[i].header.stamp = ros::Time::now();
		pub_pc.publish(pc);
		pub_mk.publish(mk);
		pub_mk2.publish(mk2);
		pub_mk_a.publish(mk_a);
		roop.sleep();
		ros::spinOnce();
	}

	return 0;
}

