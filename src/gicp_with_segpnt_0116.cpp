/*
 * src: gicp_with_zed.cpp
 * last_update: '16.09.09
 * 
 * memo:
 *		bufferからの2点群にGICPをかけるsrc.
 *		GICPの結果をLocalizationに利用(/now_position)
 *		laserのsensor_msgs取得時のcameraのrgb/depth画像を保存．
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>//追加..rosのPointCloud, pclのPointCloud変換

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <usr_msgs/Edge.h>//ark:Edge情報含有(usr_msgs/cpp/include/usr_msgs/Edge.h;pclの型は無い模様
#include <scan_match2/util.h>
#include <ceres_msgs/AMU_data.h>
#include <time_util/stopwatch.h>
#include <tf/transform_broadcaster.h>

// #include "jsk_rviz_plugins/Pictogram.h"
// #include "jsk_rviz_plugins/PictogramArray.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Eigen;

typedef pcl::PointNormal PN;
typedef pcl::PointCloud<PN> CloudN;
typedef CloudN::Ptr CloudNPtr;
typedef CloudN::ConstPtr CloudNConstPtr;
/*###ark### pcl::PointXYZINormal version #########
typedef pcl::PointXYZINormal PIN;
typedef pcl::PointCloud<PIN> CloudIN;
typedef CloudIN::Ptr CloudINPtr;
typedef CloudIN::ConstPtr CloudINConstPtr;
##################################################*/

const double OFFSET = 0.0040410296;
float DT = 2.0;//1.0;//0.2;
float hosei = 1.0;

float odom_v    = 0.0;
float yawrate   = 0.0;
float pitchrate = 0.0;
float rollrate  = 0.0;
float amu_roll  = 0.0;
float amu_pitch = 0.0;
float amu_yaw   = 0.0;

bool is_new = false;
//bool camera_info_flag = false;

double MaxCorrespondenceDistance = 0.3;
double MaximumIterations = 20;
double TransformationEpsilon   = 1e-8;
double EuclideanFitnessEpsilon = 1e-8;

const float Velodyne_height = 1.0;

usr_msgs::Edge edge;
geometry_msgs::PoseStamped camera_info_in;

jsk_rviz_plugins::PictogramArray landmark;
//landmark.header.frame_id = "/map";
const string icon_name = "fa-flag";
ros::Publisher pub_landmark;

Matrix4f S = Matrix4f::Identity();
Stopwatch sw;

//--- for segmented point ---//
bool seg_pnt_flag = false;
sensor_msgs::PointCloud2 seg_pnt_;


void edge_callback(usr_msgs::Edge::ConstPtr msg)
{
	edge = *msg;
	is_new = true;
	//cout << "idx_tgt = " << msg->idx_tgt << endl;
	//cout << "idx_src = " << msg->idx_src << endl;
}

void segpnt_callback(const sensor_msgs::PointCloud2 &msg)
{
	seg_pnt_ = msg;
	seg_pnt_flag = true;
}

/*void camera_callback(const geometry_msgs::PoseStamped &msg)
{
	camera_info_in = msg;
	camera_info_flag = true;
}*/

void check(void)
{
	static int num = 0;
//	cout << "check " << num << endl;
	num ++;
}
/*
void pubMarker(const float x, const float y)
{			
	jsk_rviz_plugins::Pictogram tmp;
	tmp.pose.position.x = x;
	tmp.pose.position.y = y;
	tmp.pose.position.z = -Velodyne_height;

	tmp.pose.orientation.w =  0.7; // 0.7
	tmp.pose.orientation.x =  0.0;
	tmp.pose.orientation.y = -0.7; //-0.7
	tmp.pose.orientation.z =  0.0;

	tmp.header.frame_id = "map";
	tmp.action = 4; 	// 
	tmp.mode   = 0; 	// 
	tmp.speed  = 0.2; 	// 
	tmp.size   = 2;
	tmp.character = icon_name;
	tmp.color.r = 25/ 255.0;
	tmp.color.g = 255/ 255.0;
	tmp.color.b = 240/ 255.0;
	tmp.color.a = 1.0;

	landmark.pictograms.push_back(tmp);

	landmark.header.frame_id = "/map";
	landmark.header.stamp = ros::Time::now();
	pub_landmark.publish(landmark);
	cout<<"landmark num: " << landmark.pictograms.size() <<endl;
}

void describeCameraInfo( FILE *fp_l, const size_t &ref_node_num, const Eigen::Matrix4f &sum, const Eigen::Vector4d &quat, const geometry_msgs::PoseStamped &camera_info )
{
	//----------------------------------------------------------------------------
	//本来ならカメラ情報はrobot中心の為，sum行列に相対位置(方位)をかける必要がある
	//Eigen::Matrix3d sum_l; // for landmark
	//----------------------------------------------------------------------------
	
	static int l_num = 0;
	float delt_x = camera_info.pose.position.x - sum(0,3);
	float delt_y = camera_info.pose.position.y - sum(1,3);
	//float delt_z = camera_info.pose.position.z - sum(2,3);
	//double yaw = atan( delt_x / delt_y ); // map座標系でradの方向に見えるか??(atanは符号を考慮していない..)
	double yaw = atan2( delt_x, delt_y );

	pubMarker(camera_info.pose.position.x, camera_info.pose.position.y);

	double roll_r, pitch_r, yaw_r;  // ロボットの姿勢(map座標系)
	double qx = (double)quat(1), qy = (double)quat(2), qz = (double)quat(3), qw = (double)quat(0);
	tf::Quaternion q(qx, qy, qz, qw);
	tf::Matrix3x3 m(q);
	m.getRPY(roll_r, pitch_r, yaw_r);

	double final_yaw = yaw - yaw_r; // ロボットからの相対角度
	
	//tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, final_yaw); // RPY --> Quat
	
	float q1, q2, q3, w;
//	q1 =	cos(z_deg*0.5)*cos(y_deg*0.5)*sin(x_deg*0.5) - sin(z_deg*0.5)*sin(y_deg*0.5)*cos(x_deg*0.5); 
//	q2 =    sin(z_deg*0.5)*cos(y_deg*0.5)*sin(x_deg*0.5) + cos(z_deg*0.5)*sin(y_deg*0.5)*cos(x_deg*0.5); 
//	q3 = -1*cos(z_deg*0.5)*sin(y_deg*0.5)*sin(x_deg*0.5) + sin(z_deg*0.5)*cos(y_deg*0.5)*cos(x_deg*0.5); 
//	w  = 	cos(z_deg*0.5)*cos(y_deg*0.5)*cos(x_deg*0.5) + sin(z_deg*0.5)*sin(y_deg*0.5)*sin(x_deg*0.5);

	q1 =      cos(final_yaw*0.5)*cos(0.0*0.5)*sin(0.0*0.5) - sin(final_yaw*0.5)*sin(0.0*0.5)*cos(0.0*0.5); 
	q2 =      sin(final_yaw*0.5)*cos(0.0*0.5)*sin(0.0*0.5) + cos(final_yaw*0.5)*sin(0.0*0.5)*cos(0.0*0.5); 
	q3 = -1 * cos(final_yaw*0.5)*sin(0.0*0.5)*sin(0.0*0.5) + sin(final_yaw*0.5)*cos(0.0*0.5)*cos(0.0*0.5); 
	w  =      cos(final_yaw*0.5)*cos(0.0*0.5)*cos(0.0*0.5) + sin(final_yaw*0.5)*sin(0.0*0.5)*sin(0.0*0.5); 
	
	//--- Landmarkの位置，方位 ---//
	fprintf(fp_l, "%s %d %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", (int)ref_node_num, l_num,
												  //sum(0,3)+camera_info.pose.position.x, sum(1,3)+camera_info.pose.position.y, sum(2,3)+camera_info.pose.position.z,
												  delt_x, delt_y, 
												  //sum(2,3)//0.0,
												  q1, q2, q3, w );
//cout << "Landmark num ==> %d" << l_num + 1 <<endl;
	l_num++;
}*/



int main(int argc, char** argv)
{

	FILE *fp_b = NULL;
	//FILE *fp_l = NULL;
	if( (fp_b=fopen("bfr.csv", "w"))==NULL/* || (fp_l=fopen("landmark_bfr.csv", "w"))==NULL */){
		printf("File Open Error!!\n");
		exit(1);
	}

	fprintf(fp_b,"%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	//printf("%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	ros::init(argc, argv, "GICP_SEMAP"); //define node name.
	ros::NodeHandle n;
	ros::Rate r(20); // 100
	
	//--- Subscriber ---//
	ros::Subscriber edge_sub   = n.subscribe("/from_buffer", 5, edge_callback);
	//ros::Subscriber camera_sub = n.subscribe("/camera_info", 5, camera_callback);
	ros::Subscriber segpnt_sub = n.subscribe("/seg_img_from_buffer", 5, segpnt_callback);
	
		
	//--- Publisher ---//
    //ros::Publisher smatch_pose_pub = n.advertise<geometry_msgs::Pose>("/scanmatch_pose",10);
    ros::Publisher pub_now = n.advertise<sensor_msgs::PointCloud>("/now_position", 1);
    pub_landmark = n.advertise<jsk_rviz_plugins::PictogramArray>("/landmarks", 1);
	
	sensor_msgs::PointCloud robo_now;
    robo_now.header.frame_id = "/map";

	//--- TF ---//
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	sw.start();
	//getParams(n);

	sensor_msgs::PointCloud path;

	Eigen::Matrix4f sum = Eigen::Matrix4f::Identity();
	
	/*以下は何に使ってるんだ??
	Eigen::Matrix3d m_clb;
	Eigen::Vector4d q_clb;
	m_clb = rpy2mat(0.0, -0.81, 0.0);
	q_clb = mat2quat(m_clb);*/

    //#################  ark: GICP Parameter  ###############################################
	pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp; 
    //#ark#pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp; 
	gicp.setMaxCorrespondenceDistance (0.3);//#ark_memo# 対応距離の最大値
	gicp.setMaximumIterations (20);         //#ark_memo# ICPの最大繰り返し回数
	gicp.setTransformationEpsilon (1e-8);   //#ark_memo# 変換パラメータ値
 	gicp.setEuclideanFitnessEpsilon (1e-8); //#ark_memo# ??

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src_v (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt_v (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp_v (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointNormal>);
  /*pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tmp_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZINormal>);
  */
    //###########################################################################################
	
	double pos[3] = {0.0};
	double pos_star[3] = {0.0};
	
    
    while(ros::ok() ){
		
		if(is_new){ // get edge information

			if(seg_pnt_flag){ // get segmented point information 
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_c (new pcl::PointCloud<pcl::PointXYZRGB>);
				{
					fromROSMsg(seg_pnt_, *pcl_pc_c);
				}	
				char seg_pnt_file[100];
				sprintf(seg_pnt_file, "seg_points/seg_pnt_%d.pcd", (int)edge.idx_src);
				pcl::io::savePCDFileBinary(seg_pnt_file, *pcl_pc_c);
			}
			
			//--- set VERTEX EDGE information ---//
			Matrix4f S = Matrix4f::Identity();
			Matrix3f e = quat2mat( edge.r_pose.pose.orientation.x, edge.r_pose.pose.orientation.y, edge.r_pose.pose.orientation.z, edge.r_pose.pose.orientation.w );
			for(size_t i=0; i<3; i++)for(size_t j=0; j<3; j++)	S(i,j) = e(i,j);
			S(0,3) = edge.r_pose.pose.position.x;
			S(1,3) = edge.r_pose.pose.position.y;
			S(2,3) = edge.r_pose.pose.position.z;
			fromROSMsg(edge.cloud_tgt, *cloud_tgt);
			fromROSMsg(edge.cloud_src, *cloud_src);
			
			/*--- Camera information ---//
			geometry_msgs::PoseStamped camera_info;
			{
				camera_info = camera_info_in;
			}*/

			static bool is_first = true;
			if(is_first){
				fprintf(fp_b,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			   //printf("%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
						"EDGE_SE3:QUAT", 0, (int)edge.idx_tgt, 
						0.0,0.0,0.0,0.0,0.0,0.0,1.0,
						1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
						1.0, 0.0, 0.0, 0.0, 0.0,
						1.0, 0.0, 0.0, 0.0,
						1.0, 0.0, 0.0,
						1.0, 0.0,
						1.0);
				char pcd_file[100];
				sprintf(pcd_file,"clouds/cloud_%d.pcd",(int)edge.idx_tgt);
				pcl::io::savePCDFileBinary(pcd_file, *cloud_tgt);

				is_first = false;
			}
            
            //ark_memo: transformer <=== #include<scan_match2/util.h> <== impl/util.cpp 
			*cloud_tmp   = transformer(*cloud_src, S);
			*cloud_tmp_v = transformer(*cloud_tmp, sum);
			*cloud_src_v = transformer(*cloud_src, sum);

			gicp.setInputSource(cloud_tmp);
			gicp.setInputTarget(cloud_tgt);
			pcl::PointCloud<pcl::PointNormal> Final; 
			//###ark###pcl::PointCloud<pcl::PointXYZINormal> Final; 
			gicp.align(Final);


			Eigen::Matrix4f F;
			F = gicp.getFinalTransformation();//#ark_memo#: GICPで求めた変換行列を表示

			Eigen::Matrix4f hantei;
			Eigen::Matrix4f idn = Eigen::Matrix4f::Identity();
			hantei = F;
			hantei = hantei - idn;

			//	std::cout << "GICP has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl; 
			// [ark]: Use FitnessScore !?!?!?
			//	std::cout << gicp.getFinalTransformation() << std::endl; 
			Eigen::Matrix4f tmp;
			//cout << hantei.norm() << endl;
			
            //---ark_memo:GICPから算出した変換行列を使用するかどうかの判定----
            if(hantei.norm() < 3.0){//0.2
				tmp = F * S;
			}else{
				tmp = S;
				cout << "xxxxxxxxxxxxxxxx matching miss!!!! xxxxxxxxxxxxxxxxxxxx" << endl;
			}
            //---ark_memo: sumの更新!!!---
			sum = sum * tmp;
			//std::cout << icp.getFinalTransformation() << std::endl;

			//edge.r_pose = trans(S);


			Eigen::Matrix3d mat;
			Eigen::Vector4d quat;	// for node
			Eigen::Vector4d quat_e; // for edge

			//--- 各頂点の位置姿勢を記述(pointcloud) ---//
			mat << sum(0,0), sum(0,1), sum(0,2), sum(1,0), sum(1,1), sum(1,2), sum(2,0), sum(2,1), sum(2,2);
			quat = mat2quat(mat);
			fprintf(fp_b,"%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", (int)edge.idx_src, sum(0,3), sum(1,3), sum(2,3), quat(1), quat(2), quat(3), quat(0));
			//printf("%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", (int)edge.idx_src, sum(0,3), sum(1,3), sum(2,3), quat(1), quat(2), quat(3), quat(0));
			//edges.csv
			mat << tmp(0,0), tmp(0,1), tmp(0,2), tmp(1,0), tmp(1,1), tmp(1,2), tmp(2,0), tmp(2,1), tmp(2,2);
			quat_e = mat2quat(mat);


			//--- 連続するフレーム間の相対位置姿勢の記述(pointcloud) ---//
			fprintf(fp_b,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			//printf("%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					"EDGE_SE3:QUAT", (int)edge.idx_tgt, (int)edge.idx_src, 
					tmp(0,3), tmp(1,3), tmp(2,3), quat_e(1), quat_e(2), quat_e(3), quat_e(0),
					1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0,
					1.0, 0.0,
					1.0);
			
			char pcd_file[100];
			sprintf(pcd_file,"clouds/cloud_%d.pcd",(int)edge.idx_src);
			pcl::io::savePCDFileBinary(pcd_file, *cloud_src);
	
			//--- for landmark ---//
			/*if(camera_info_flag){	
				geometry_msgs::PoseStamped camera_info_;
				{
				camera_info_ = camera_info_in;
				}
				//--- Describe Landmark infomation ---//
			//if( !((camera_info.pose.position.x == 0.0) && (camera_info.pose.position.y == 0.0) && (camera_info.pose.position.z == 0.0)) ){ // Landmark情報の記述
				describeCameraInfo( fp_l, edge.idx_src, sum, quat, camera_info_ );
				camera_info_flag = false;
			}*/
			
			//--- GICPによるrobotの現在地 ---//
			robo_now.points.resize(1);
			robo_now.points[0].x = sum(0,3);
			robo_now.points[0].y = sum(1,3);
			robo_now.points[0].z = sum(2,3);
			robo_now.header.stamp = ros::Time::now();
			pub_now.publish(robo_now);
			robo_now.points.clear();


			//-------------------------------------------------------------------
			//--- for visualize ---
			//-------------------------------------------------------------------
			//Final = transformer(*cloud_src, gicp.getFinalTransformation() * S);
			Final = transformer(*cloud_src, sum);
			*cloud_map = transformer(*cloud_src, sum);

			//*cloud_src_v = transformer(*cloud_tmp, sum);
			*cloud_tgt_v = transformer(*cloud_tgt, sum*tmp.inverse());

			show(n,"source", *cloud_src_v);
			show(n,"target", *cloud_tgt_v);
			show(n,"tmp", *cloud_tmp_v);
			show(n,"final", Final);
			static int v_cnt = 0;
			if((v_cnt % 1)==0) show(n,"map", *cloud_map); // /map上に定義される、大きな地図。
			v_cnt ++;

			pos[0] = sum(0,3);
			pos[1] = sum(1,3);
			pos[2] = sum(2,3);

			is_new 		 = false;
			seg_pnt_flag = false;
		}
		
		double param = 0.0025;
		pos_star[0] -= param * (pos_star[0] - pos[0]);
		pos_star[1] -= param * (pos_star[1] - pos[1]);
		pos_star[2] -= param * (pos_star[2] - pos[2]);
		transform.setOrigin(tf::Vector3(pos_star[0], pos_star[1], pos_star[2]));
		transform.setRotation(tf::Quaternion(0.0, 0.0, 0.02));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "hogehoge"));
	//	cout << pos_star[0] << " " << pos_star[1] << endl;
		
		r.sleep();
		ros::spinOnce();
	}

	fclose(fp_b); // close bfr.csv..
//	fclose(fp_l); // close landmark.csv..
//	cout << "save bfr.csv landmark.csv ..\n";

	return 0;
}
