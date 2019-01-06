/*
 * src: gicp_for_loop.cpp
 *
 * memo: 再訪判定の時に利用. 【loop_detector】から二つのNodeの位置情報と点群を受け取り, 
 *		 ICP処理(GICP)を施すsrc.
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>//rosのPointCloud, pclのPointCloud変換
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <time_util/stopwatch.h>
#include <AMU_data.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Edge.h>
#include <scan_match/util.h>

using namespace Eigen;

// typedef pcl::PointNormal PN;
// typedef pcl::PointCloud<PN> CloudN;
// typedef CloudN::Ptr CloudNPtr;
// typedef CloudN::ConstPtr CloudNConstPtr;
// ##### PointXYZINormal version ############## 
typedef pcl::PointXYZINormal PIN;
typedef pcl::PointCloud<PIN> CloudIN;
typedef CloudIN::Ptr CloudINPtr;
typedef CloudIN::ConstPtr CloudINConstPtr;
// ##############################################

bool is_new = false;
double MaxCorrespondenceDistance = 10.0;
double MaximumIterations = 20;
double TransformationEpsilon = 1e-8;
double EuclideanFitnessEpsilon = 1e-8;

usr_msgs::Edge edge;

Matrix4f S = Matrix4f::Identity();

Stopwatch sw;

bool first = true;

void callback(usr_msgs::Edge::ConstPtr msg){
	// cout << "=============get================"<<endl;
	edge = *msg;
	is_new = true;
	// cout << "idx_tgt = " << edge.idx_tgt << endl;
	// cout << "idx_src = " << edge.idx_src << endl;
}

void check(void)
{
	static int num = 0;
	cout << "check " << num << endl;
	num ++;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "GICP"); //define node name.
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Subscriber sub = n.subscribe("/from_loop_detection",1,callback);

	sw.start();
	//	getParams(n);

	sensor_msgs::PointCloud path;


	// pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp; 
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp; 
	gicp.setMaxCorrespondenceDistance (2.0);//0.3// 対応距離の最大値
	// gicp.setMaxCorrespondenceDistance (10.0);//0.3// 対応距離の最大値
	gicp.setMaximumIterations (100);        //20 // ICPの最大繰り返し回数
	gicp.setTransformationEpsilon (1e-8);   // 変換パラメータ値
	gicp.setEuclideanFitnessEpsilon (1e-8);
   
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointNormal>);
	// ######## PonintXYZINormal version ##############################################
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZINormal>);
    // ######################################################################################
	
    while(ros::ok()){
		if(is_new){
			Matrix4f S = Matrix4f::Identity();
			S.block(0,0,3,3) = quat2mat(edge.r_pose.pose.orientation.x, edge.r_pose.pose.orientation.y, edge.r_pose.pose.orientation.z, edge.r_pose.pose.orientation.w );
			S(0,3) = edge.r_pose.pose.position.x;
			S(1,3) = edge.r_pose.pose.position.y;
			S(2,3) = edge.r_pose.pose.position.z;
			fromROSMsg(edge.cloud_tgt, *cloud_tgt);
			fromROSMsg(edge.cloud_src, *cloud_src);

			*cloud_tmp = transformer(*cloud_src,S);

			gicp.setInputSource(cloud_tmp); 
			// gicp.setInputCloud(cloud_tmp); 
			gicp.setInputTarget(cloud_tgt); 
			// pcl::PointCloud<pcl::PointNormal> Final; 
			// pcl::PointCloud<pcl::PointXYZ> Final; 
			//XYZINormal version ### 
			pcl::PointCloud<pcl::PointXYZINormal> Final; 
			gicp.align(Final);

			Eigen::Matrix4f F = gicp.getFinalTransformation();


	//			std::cout << "GICP has converged:" << gicp.hasConverged() << " score: " << 
	//			gicp.getFitnessScore() << std::endl; 
	//			std::cout << gicp.getFinalTransformation() << std::endl; 
			Eigen::Matrix4f tmp;
			Eigen::Matrix4f idn = Eigen::Matrix4f::Identity();
			float jeje = ((F-idn).block(0,0,3,3)).norm();
			cout << "JEJE = " << jeje << endl;
			if(jeje < 0.2){
		//	if(gicp.getFitnessScore() < 2.0){//2.0
				tmp = F * S;
			}else{
				cout << "====CONTINUE====" << endl << endl;
				is_new = false;
				continue;
			}
			cout << "S = " << S << endl << "solution = " << endl << F << endl << "final = " << endl << tmp << endl << endl;
			Eigen::Vector4f quat;
			quat = mat2quat_f( tmp.block(0,0,3,3) );
	FILE *fp=fopen("aft.csv","a");
			fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					"EDGE_SE3:QUAT", (int)edge.idx_tgt, (int)edge.idx_src, 
					tmp(0,3), tmp(1,3), tmp(2,3),quat(1), quat(2), quat(3), quat(0),
					0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.05, 0.0, 0.0, 0.0, 0.0,
					0.05, 0.0, 0.0, 0.0,
					0.05, 0.0, 0.0,
					0.05, 0.0,
					0.05);
			
	fclose(fp);
			Final = transformer(*cloud_tmp, F);

			show(n,"source", *cloud_src);
			show(n,"target", *cloud_tgt);
			show(n,"tmp", *cloud_tmp);
			show(n,"final", Final);
			// show(n,"map", *cloud_map); // /map上に定義される、大きな地図。


			//	cout << "-------------------" << endl;
			is_new = false;
		}

		r.sleep();
		ros::spinOnce();
	}


	return 0;
}
