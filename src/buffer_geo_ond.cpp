/*
 * src: buffer.cpp
 * 
 * memo: 一定距離毎に点群を2セット貯めて, ICP処理を行う【gicp】にPublishするsrc
 * 	    	   
*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>//追加..rosのPointCloud, pclのPointCloud変換
//#include <scan_match2/show.h>
#include <scan_match/util.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <time_util/stopwatch.h>
//#include <ceres_msgs/AMU_data.h>
#include <AMU_data.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <Eigen/Core>
//#include <usr_msgs/Edge.h>
#include <Edge.h>

using namespace Eigen;

//cloud_src : 時刻tの点群。
// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
//cloud_tgt : 時刻t-1の点群。
// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);

const string frame_name = "/map";

const double LEAF = 0.075;
// const double LEAF = 0.175;
const double OFFSET = 0.0040410296;
// const double DIST = 1.5;
double DIST = 0.5;
bool strt_flg = true; //開始場所での推定のため added by machinaka 20160902

size_t offset = 0;

float DT = 2.0;//1.0;//0.2;
float hosei = 1.0;

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

bool is_new = false;
bool clicked_flag = false;
bool seg_img_flag = false;
bool amu_flag = false;

sensor_msgs::PointCloud2 seg_img_;
double MaxCorrespondenceDistance = 0.3;
double MaximumIterations = 250;
// double MaximumIterations = 20;
double TransformationEpsilon = 1e-8;
double EuclideanFitnessEpsilon = 1e-8;
Matrix4f S = Matrix4f::Identity();

Stopwatch sw;
Stopwatch atm;

//Eigen::Matrix3d rpy2mat(double roll, double pitch, double yaw);

void amu_callback(ceres_msgs::AMU_data::ConstPtr msg){
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
	amu_flag = true;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg){
	odom_v = msg->twist.twist.linear.x;
	//	cout << "odom_v = " << odom_v << endl;
}

void seg_img_callback(const sensor_msgs::PointCloud2 &msg)
{
	seg_img_ = msg;
	seg_img_flag = true;
	// cout<<"in seg_img_callback"<<endl;
}

//---Set cloud_tgt, cloud_src---//
void callback(const pcl::PCLPointCloud2ConstPtr msg){ //Set type of message.
	if(strt_flg){
		dist = DIST + 1;
		strt_flg = false;
	}
	//過密な推定を抑える処置。
	// if((dist > DIST )){
	// if((dist > DIST )&&(fabs(yawrate)<0.05)){
	if((dist > DIST )&&(fabs(yawrate)<0.08)){
		if(!cloud_src->points.empty()) *cloud_tgt = *cloud_src;
		
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		pcl::PCLPointCloud2::Ptr tmp (new pcl::PCLPointCloud2);
		
        sor.setInputCloud (msg);
		sor.setLeafSize (LEAF,LEAF,LEAF);
		sor.filter (*tmp);
		pcl::fromPCLPointCloud2(*tmp, *cloud_src);//PCLPointCloud2 -> PointCloud<T>

		//if(!cloud_src->empty() && cloud_tgt->empty())
		//	pcl::io::savePCDFileBinary("clouds/cloud_0.pcd", *cloud_src);
		
		if(!cloud_tgt->points.empty() && !cloud_src->points.empty()){
			is_new = true;
		}
		sw.reset();
	}
}

void input(usr_msgs::Edge &edge){
	edge.r_pose.pose.position.x = S(0,3);
	edge.r_pose.pose.position.y = S(1,3);
	edge.r_pose.pose.position.z = S(2,3);
	Matrix3d mat;
	mat << S(0,0), S(0,1), S(0,2), S(1,0), S(1,1), S(1,2), S(2,0), S(2,1), S(2,2);
	Vector4d q = mat2quat(mat);
	edge.r_pose.pose.orientation.x = q(1);
	edge.r_pose.pose.orientation.y = q(2);
	edge.r_pose.pose.orientation.z = q(3);
	edge.r_pose.pose.orientation.w = q(0);
	static size_t cnt = 0;
	edge.idx_tgt = cnt		+offset;// offset始まりの数字
	edge.idx_src = cnt+1	+offset;
	cnt ++;
}



int main (int argc, char** argv)
{

	cout << "idx offset = ";
	cin >> offset;
//	cout << "bag replay speed = ";
//	cin >> hosei;
	cout << "DIST = " ;
	cin >> DIST;
	ros::init(argc, argv, "BUFFER_azm"); //define node name.
	ros::NodeHandle n;
//	ros::Rate r(100*hosei);
	ros::Rate r(100);
	
    //--- Subscriber ---//
    //ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal",1,callback);
	ros::Subscriber sub = n.subscribe("/rm_cluster/removed_points",1,callback);//人を除いた点群
	//ros::Subscriber sub = n.subscribe("/velodyne_points",1,callback);//matchingのvisualize確認用
    ros::Subscriber amu_sub = n.subscribe("/AMU_data", 1, amu_callback);
	ros::Subscriber odm_sub = n.subscribe("/tinypower/odom", 1, odom_callback);
    //ros::Subscriber azm_sub = n.subscribe("/perfect_velodyne/filtered_pose", 1, azm_callback);
    
   	ros::Subscriber seg_pnt_sub = n.subscribe("/segmented_image/converted/points", 1, seg_img_callback);
	//--- Publisher ---//
    ros::Publisher pub_edge = n.advertise<usr_msgs::Edge>("/from_buffer",5);

	ros::Publisher pub_seg_pnt = n.advertise<sensor_msgs::PointCloud2>("/seg_img_from_buffer", 5);	

	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("/odom/complement", 1);	

	usr_msgs::Edge edge;

	sw.start();
	atm.start();
	ros::Time current_time, last_time;

	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZINormal>);


	Stopwatch tm;
	tm.start();

	Matrix4f oS = Matrix4f::Identity();
	Matrix4f aS = Matrix4f::Identity();
   

	Eigen::Matrix3d m_clb;
	Eigen::Vector4d q_clb;
	m_clb = rpy2mat(0.0, -0.81, 0.0);
	q_clb = mat2quat(m_clb);

	while(ros::ok()){
		amu_flag = true;
		if(amu_flag){
		//if(is_new){
			seg_img_flag = true;
			if(is_new && seg_img_flag){
				input(edge);	//相対位置情報を与える。
				toROSMsg(*cloud_tgt, edge.cloud_tgt);
				toROSMsg(*cloud_src, edge.cloud_src);
				edge.header.stamp = ros::Time::now();
				pub_edge.publish(edge);

				//if(seg_img_flag){
				sensor_msgs::PointCloud2 seg_img;
				{
					seg_img = seg_img_;
				}
				seg_img.header.stamp    = ros::Time::now();
				seg_img.header.frame_id = frame_name;
				pub_seg_pnt.publish(seg_img);
				seg_img_flag = false;
				//}
				cout << "S=" << endl << S << endl;
				
				S = Matrix4f::Identity();
				is_new = false;
				dist = 0.0;
			}
	//		float dt = tm.getTime()*hosei;//ark: hosei = bag_speed
	//		dist += fabs(odom_v) * dt;
	//		oS(1,3) = odom_v * dt;

			current_time = ros::Time::now();
			float dt = tm.getTime();
			last_time = ros::Time::now();
			dist += fabs(odom_v) * dt;
			// oS(1,3) = odom_v * dt;
			oS(0,3) = odom_v * dt; //x方向に進む
			// oS(1,3) = odom_v * dt * 0.9;
			
			//aS = calcMat(pitchrate*dt, rollrate*dt, azm_yaw*dt);//ark:calcMat <== #include<scan_match2/util.h> <--use rpy2mat() 
			aS = calcMat(0.0, 0.0, yawrate*dt); 
			tm.reset();
			S = S * oS * aS;	//oS:odom, aS:AMU
			amu_flag = false;


			double dis = odom_v * dt; 

			yaw += yawrate * dt; 
			x += dis * cos(yaw);
			y += dis * sin(yaw);

			static int cnt = 0 ;
			const int sparse = 80;

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
			nav_msgs::Odometry odom;
			odom.header.frame_id = "map";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			if(cnt == sparse){
				pub_odom.publish(odom);
			}
			cnt++;                                                                                                                    
			cnt %= sparse + 1;
		}
		//cout << "S=" << endl << S << endl;
		r.sleep();
		ros::spinOnce();
	}
	return (0);
}

