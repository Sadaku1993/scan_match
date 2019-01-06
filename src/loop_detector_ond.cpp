/*
 * src: loop_detector.cpp
 * Author: s.shimizu
 *
 * pkg       : scan_match2
 * created   : 2015.1.10
 * lastupdate: 20xx.xx.xx
 *
 * memo: 再訪判定の時に使用. 任意のNodeから一定範囲内にある別のNodeを探し,
 *		 二つのNodeの位置情報と点群情報を【gicp_for_loop】に向けてPublishするsrc.
 */	

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>

#include <Eigen/Core>

//user defined
#include <Edge.h>
#include <scan_match/util.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>//追加..rosのPointCloud, pclのPointCloud変換
#include <sensor_msgs/point_cloud_conversion.h>//追加..
#include <tf/transform_broadcaster.h>

const size_t DIST = 20;
const size_t MAX_NODE = 10000;
const char filename[] = "aft_top.csv";

double DISTANCE = 5.0;

//namespaces
using namespace std;	



void load(size_t &cnt, Matrix4f poses[MAX_NODE], sensor_msgs::PointCloud &pc, const ros::Publisher &pub_pc){
	FILE *fp;
	fp = fopen(filename,"r");
	if(fp == NULL){
		cout << "could not find file" << endl;
	}

	cnt = 0;
	while(1){
		float data[7] = {0};
		char type[1000];
		if(fscanf(fp,"%s",type) == EOF)break;
		if(strcmp(type,"VERTEX_SE3:QUAT")==0){
			if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
			Matrix4f pose = Matrix4f::Identity();
			pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
			pose(0,3) = data[0];
			pose(1,3) = data[1];
			pose(2,3) = data[2];
			poses[cnt] = pose;

			cnt ++;
		}
	}

	fclose(fp);

	//LOOP DETECTION

	pc.points.resize(0);
	for(size_t i=0;i<cnt;i++){
		geometry_msgs::Point32 p;
		p.x = poses[i](0,3);
		p.y = poses[i](1,3);
		p.z = poses[i](2,3);
		pc.points.push_back(p);
	}
	pc.header.stamp = ros::Time::now();
	pub_pc.publish(pc);
}




int main (int argc, char** argv)
{
	cout << "sleep = ";
	double slp = 0.0;
	cin >> slp;
	cout << "DISTANCE = " ;
	cin >> DISTANCE;
	cout << "start -> ";
	size_t start = 0;
	cin >> start;
	ros::init(argc, argv, "LOOPDETECTOR"); //define node name.
	ros::NodeHandle n;
	ros::Rate roop(100);          //Set Rate[Hz].
	ros::Publisher pub_pc = n.advertise<sensor_msgs::PointCloud>("path",1);
	ros::Publisher pub_pc_loop = n.advertise<sensor_msgs::PointCloud>("loop",1);
	ros::Publisher pub_mk = n.advertise<visualization_msgs::Marker>("pair",1);
	ros::Publisher pub_edge = n.advertise<usr_msgs::Edge>("/from_loop_detection",1);
	sensor_msgs::PointCloud pc;
	sensor_msgs::PointCloud pc_loop;
	visualization_msgs::Marker mk;
	pc.header.frame_id = "map";
	pc_loop.header.frame_id = "map";
	mk.header.frame_id = "map";
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	mk.ns = "namespace";
	mk.id = 0;
	mk.type = visualization_msgs::Marker::LINE_LIST;
	mk.action = visualization_msgs::Marker::ADD;
	mk.scale.x = 1.0;
	mk.scale.y = 0.1;
	mk.scale.z = 0.1;
	mk.color.a = 1.0;
	mk.color.r = 0.0;
	mk.color.g = 1.0;
	mk.color.b = 0.0;

	Matrix4f poses[MAX_NODE];
	/*
	   FILE *fp;
	   fp = fopen("aft.csv","r");
	   if(fp == NULL){
	   cout << "could not find file" << endl;
	   return -1;
	   }

	   size_t cnt = 0;
	   while(1){
	   float data[7] = {0};
	   char type[1000];
	   if(fscanf(fp,"%s",type) == EOF)break;
	   if(strcmp(type,"VERTEX_SE3:QUAT")==0){
	   if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
	   Matrix4f pose = Matrix4f::Identity();
	   pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
	   pose(0,3) = data[0];
	   pose(1,3) = data[1];
	   pose(2,3) = data[2];
	   poses[cnt] = pose;

	   cnt ++;
	   }
	   }

	   fclose(fp);

	//LOOP DETECTION

	for(size_t i=0;i<cnt;i++){
	geometry_msgs::Point32 p;
	p.x = poses[i](0,3);
	p.y = poses[i](1,3);
	p.z = poses[i](2,3);
	pc.points.push_back(p);
	}
	 */
	size_t cnt = 0;
	load(cnt, poses, pc, pub_pc);

	double pos[3] = {0.0};
	double pos_star[3] = {0.0};
	for(size_t i=DIST+start;i<cnt;i++){
		if(i > 5370) continue;
		// if(i > 1370) continue;
		size_t hit = 0;
		float min_dist = 10.0;
		for(size_t j=i-DIST;j!=0;j--){
			if(j > 5370) continue;
			float delta_x = poses[i](0,3) - poses[j](0,3);
			float delta_y = poses[i](1,3) - poses[j](1,3);
			float delta_z = poses[i](2,3) - poses[j](2,3);
			float dist = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
			// if(1){
			if(dist < min_dist){
				hit = j;
				min_dist = dist;
			}
		}
		// if(1){
		if(min_dist < DISTANCE){
			// if(i == 71 || i == 72 ){
			// i = 71;
			// hit = 443;
			cout << i << " " << hit << " " << min_dist << endl;
			geometry_msgs::Point p1, p2;
			p1.x = poses[i](0,3);
			p1.y = poses[i](1,3);
			p1.z = poses[i](2,3);
			p2.x = poses[hit](0,3);
			p2.y = poses[hit](1,3);
			p2.z = poses[hit](2,3);
			//mk.points.push_back(p1);
			//mk.points.push_back(p2);

			pc_loop.points.resize(1);
			pc_loop.points[0].x = p1.x;
			pc_loop.points[0].y = p1.y;
			pc_loop.points[0].z = p1.z;
			pub_pc_loop.publish(pc_loop);
			mk.points.resize(2);
			mk.points[0] = p1;
			mk.points[1] = p2;
			transform.setOrigin(tf::Vector3(p1.x,p1.y,p1.z));
			transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "hogehoge"));

			mk.header.stamp = ros::Time::now();
			pub_mk.publish(mk);

			Matrix4f T = poses[hit].inverse() * poses[i];
			cout << "T = " << endl << T << endl;
			Vector4f q = mat2quat_f(T.block(0,0,3,3));
			Vector3f t;
			t << T(0,3), T(1,3) ,T(2,3);
			usr_msgs::Edge e;
			e.header.frame_id = "target";
			e.header.stamp = ros::Time::now();
			e.r_pose.pose.position.x = t(0);
			e.r_pose.pose.position.y = t(1);
			e.r_pose.pose.position.z = t(2);
			e.r_pose.pose.orientation.x = q(1);
			e.r_pose.pose.orientation.y = q(2);
			e.r_pose.pose.orientation.z = q(3);
			e.r_pose.pose.orientation.w = q(0);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
			//### XYZINomal version!! ###
			//###pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);
			//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt_star (new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
			//### XYZINormal version!! ###
			//###pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
			//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src_star (new pcl::PointCloud<pcl::PointNormal>);

			char file_tgt[500];
			char file_src[500];
			sprintf(file_tgt,"clouds/cloud_%d.pcd",(int)hit);
			sprintf(file_src,"clouds/cloud_%d.pcd",(int)i);
			pcl::io::loadPCDFile(file_tgt, *cloud_tgt);
			pcl::io::loadPCDFile(file_src, *cloud_src);



			pcl::toROSMsg(*cloud_tgt, e.cloud_tgt);
			pcl::toROSMsg(*cloud_src, e.cloud_src);


			//	sensor_msgs::PointCloud2::Ptr tmp = &e.cloud_tgt;
			/*downsampling_start*/
			/*			pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
						sor.setInputCloud (cloud_tgt);
						sor.setLeafSize (0.1,0.1,0.1);
						sor.filter (e.cloud_tgt);
						sor.setInputCloud (e.cloud_src);
						sor.setLeafSize (0.1,0.1,0.1);
						sor.filter (e.cloud_src);
			 */
			e.idx_tgt = hit;
			e.idx_src = i;
			pub_edge.publish(e);
			cout << "published." << endl;
			sleep(slp);
			system("mv aft.csv bfr.csv");
			// system("graph_slam3d"); // bfr.csv -> aft.csv
			// system("/home/ubuntu/AMSL_ros_pkg/mapping/my_g2o/bin/graph_slam3d"); // bfr.csv -> aft.csv
			system("/home/amsl/AMSL_ros_pkg/mapping/my_g2o/bin/graph_slam3d"); // bfr.csv -> aft.csv
			load(cnt,poses,pc,pub_pc);
			if(!ros::ok())return 1;
		}
		// break;
		}
		for(size_t i=0;i<mk.points.size();i++){
			cout << mk.points[i].x << endl;
		}
		cout << mk.points.size() << endl;

		while(ros::ok()){
			pc.header.stamp = ros::Time::now();
			mk.header.stamp = ros::Time::now();
			pub_pc.publish(pc);
			pub_mk.publish(mk);

			double param = 0.01;
			pos_star[0] -= param * (pos_star[0] - pos[0]);
			pos_star[1] -= param * (pos_star[1] - pos[1]);
			pos_star[2] -= param * (pos_star[2] - pos[2]);
			roop.sleep();
			ros::spinOnce();
		}

		return 0;
		}
