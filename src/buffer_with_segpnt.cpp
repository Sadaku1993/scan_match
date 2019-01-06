/*
 * src: buffer_with_segpnt.cpp
 * last_update: '16.09.09
 *
 * 
 * memo:
 *		rm_human点群とsegmentした三次元点群を受け取るsrc．
 *		
 *
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>//追加..rosのPointCloud, pclのPointCloud変換
//#include <scan_match2/show.h>
#include <scan_match2/util.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <time_util/stopwatch.h>
#include <ceres_msgs/AMU_data.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <usr_msgs/Edge.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace Eigen;

//cloud_tgt : 時刻tの点群。
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
//###pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
//cloud_src : 時刻t-1の点群。
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
//###pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);

const string frame_name = "/map";

const double LEAF = 0.05; //0.075
//const double OFFSET = 0.0040410296;
const double OFFSET =  0.0041196016 - 0.00025 + 0.000171428;
//const double DIST = 1.5;
double DIST = 0.5;

size_t offset = 0;

float DT = 1.5;//1.0;//0.2;
float hosei = 1.0;

float odom_v = 0.0;
float yawrate = 0.0;
float pitchrate = 0.0;
float rollrate = 0.0;
float amu_roll = 0.0;
float amu_pitch = 0.0;
float amu_yaw = 0.0;

//### ark add ###//
float azm_yaw = 0.0;
//###############//

double dist = 0.0;
double dist_tmp = 0.0;

bool is_new 	  = false;
bool clicked_flag = false;
bool seg_img_flag = false;

sensor_msgs::PointCloud2 seg_img_;

/*
double MaxCorrespondenceDistance = 0.3;
double MaximumIterations = 20;
double TransformationEpsilon = 1e-8;
double EuclideanFitnessEpsilon = 1e-8;
*/
Matrix4f S = Matrix4f::Identity();

Stopwatch sw;

geometry_msgs::PoseStamped clicked_pnt_in;

//Eigen::Matrix3d rpy2mat(double roll, double pitch, double yaw);


void amu_callback(ceres_msgs::AMU_data::ConstPtr msg)
{
	//rate
	rollrate  =   M_PI * msg->droll  / 180.0;
	pitchrate =   M_PI * msg->dpitch / 180.0;
	yawrate   = - M_PI * msg->dyaw   / 180.0 + OFFSET;

	//lcl[1].w += 0.0041196016 - 0.00025 + 0.000171428; //correction
	//absolute
	amu_roll  = M_PI * msg->roll  / 180.0 + M_PI;
	amu_pitch = M_PI * msg->pitch / 180.0;
	amu_yaw   = M_PI * msg->yaw   / 180.0;

	if(amu_roll >  M_PI)	 amu_roll -= 2.0*M_PI;
	else if(amu_roll < -M_PI)amu_roll += 2.0*M_PI;
}

void odom_callback(nav_msgs::Odometry::ConstPtr msg)
{
	odom_v = msg->twist.twist.linear.x;
	//	cout << "odom_v = " << odom_v << endl;
}

void azm_callback(sensor_msgs::Imu::ConstPtr msg)
{
    azm_yaw = M_PI * msg->orientation.z/ 180.0;
}

/*void cam_callback(const geometry_msgs::PoseStamped &msg)
{
	clicked_pnt_in = msg;
	clicked_flag = true;
}*/

void seg_img_callback(const sensor_msgs::PointCloud2 &msg)
{
	seg_img_ = msg;
	seg_img_flag = true;
}
	
//--- Set cloud_tgt, cloud_src ---//
void points_callback(const pcl::PCLPointCloud2ConstPtr msg){ //Set type of message.
	
	//過密な推定を抑える処置。
	if((dist > DIST ) && (fabs(yawrate)<0.05) ){
		if(!cloud_src->points.empty()) *cloud_tgt = *cloud_src;
		
		//--- Downsampling ---//
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//ark...
		pcl::PCLPointCloud2::Ptr tmp (new pcl::PCLPointCloud2);//ark...
			
        sor.setInputCloud (msg);
		sor.setLeafSize (LEAF,LEAF,LEAF);
		sor.filter (*tmp);
		pcl::fromPCLPointCloud2(*tmp, *cloud_src);//ark...PCLPointCloud2-> PointCloud<T>

		//if(!cloud_src->empty() && cloud_tgt->empty())
		//	pcl::io::savePCDFileBinary("clouds/cloud_0.pcd", *cloud_src);
		if(!cloud_tgt->points.empty() && !cloud_src->points.empty()){
			is_new = true;
		}
		sw.reset();
	}
}


void input(usr_msgs::Edge &edge)
{
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
	edge.idx_tgt = cnt		+offset;//ark: offset始まりの数字
	edge.idx_src = cnt+1	+offset;
	cnt ++;
}



int main (int argc, char** argv)
{

	cout << "idx offset = ";//tougosuru taisyono node number
	cin >> offset;
	cout << "DIST = " ;
	cin >> DIST;
	ros::init(argc, argv, "BUFFER_WITH_ZED"); //define node name.
	ros::NodeHandle n;
	//ros::Rate r(100*hosei); //rosparam set use_sim_time true -> rosbag play -r xxx 使用の為
	ros::Rate r(20);
	
    //--- Subscriber ---//
    //ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal",1,callback);
	ros::Subscriber sub = n.subscribe("/rm_cluster/removed_points", 1, points_callback);  //人を除いた点群
    //ros::Subscriber azm_sub = n.subscribe("/perfect_velodyne/filtered_pose", 1, azm_callback); //azm_estimationの結果を利用
	//ros::Subscriber pnt_sub = n.subscribe("/velodyne_points", 1, points_callback); //velodyneの生点群
	ros::Subscriber amu_sub = n.subscribe("/AMU_data", 100, amu_callback);
	ros::Subscriber odm_sub = n.subscribe("/tinypower/odom", 1, odom_callback);
	//ros::Subscriber cam_sub = n.subscribe("/landmark", 1, cam_callback);
   	ros::Subscriber seg_pnt_sub = n.subscribe("/segmented_image/converted/points", 1, seg_img_callback);

	//--- Publisher ---//
    ros::Publisher pub_edge    = n.advertise<usr_msgs::Edge>("/from_buffer", 5);
	//ros::Publisher pub_caminfo = n.advertise<geometry_msgs::PoseStamped>("/camera_info", 5); // for landmark
	ros::Publisher pub_seg_pnt = n.advertise<sensor_msgs::PointCloud2>("/seg_img_from_buffer", 5);	

	//--- Edge (user defined.)
	usr_msgs::Edge edge;

	sw.start();

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointNormal>);
	//###pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZINormal>);

	Stopwatch tm;
	tm.start();

	Matrix4f oS = Matrix4f::Identity();
	Matrix4f aS = Matrix4f::Identity();
//	Matrix4f SSS = Matrix4f::Identity();//for visualize
//  double YAW = 0.0;					// 〃
	
	/*  
	//ark: roll, pithch, yaw情報からquaternion作成!! ただこれはどこで使われてる?単なる確認用??
	Eigen::Matrix3d m_clb;
	Eigen::Vector4d q_clb;
	m_clb = rpy2mat(0.0, -0.81, 0.0);//ark: rpy2mat() <== #include <scan_match2/util.h> 
	q_clb = mat2quat(m_clb);         //ark: mat2quat() <==       〃
	*/
	
	while(ros::ok()){
		
		if(is_new){		 // get src&tgt pointcloud
			input(edge); // 相対位置情報を与える。
			toROSMsg(*cloud_tgt, edge.cloud_tgt);
			toROSMsg(*cloud_src, edge.cloud_src);
			edge.header.stamp = ros::Time::now();
			
			pub_edge.publish(edge);
			
			//--- set camara info ---//
			/*if(clicked_flag){
				geometry_msgs::PoseStamped clicked_pnt;
				{
					clicked_pnt = clicked_pnt_in;
				}
				clicked_pnt.header.stamp    = ros::Time::now();
				clicked_pnt.header.frame_id = frame_name;

				pub_caminfo.publish(clicked_pnt);
				clicked_flag = false;
			}*/
			
			//--- set segmented point ---//	
			if(seg_img_flag){
				sensor_msgs::PointCloud2 seg_img;
				{
					seg_img = seg_img_;
				}
				seg_img.header.stamp    = ros::Time::now();
				seg_img.header.frame_id = frame_name;
				pub_seg_pnt.publish(seg_img);
				seg_img_flag = false;
			}

			cout << "S=" << endl << S << endl;
            S = Matrix4f::Identity();
			is_new = false;
			dist = 0.0;
		}

		double dt = tm.getTime();
		tm.reset();

		dist += fabs(odom_v) * dt;
//		dist_tmp = dist;
		oS(1,3) = odom_v * dt;

        //--- angle matrix ---//
        //aS = calcMat(pitchrate*dt, rollrate*dt, yawrate*dt);//[org] ark:calcMat <== #include<scan_match2/util.h> <--use rpy2mat() 
        //aS = calcMat(pitchrate*dt, rollrate*dt, azm_yaw*dt);
        double delt_yaw = yawrate * dt;
		//yawrate = 0.0;
		aS = calcMat(0.0, 0.0, delt_yaw);
//		YAW += delt_yaw;
		
		/*if(YAW>M_PI) 	   YAW -= 2*M_PI;
		else if(YAW<-M_PI) YAW += 2*M_PI;*/

		//tm.reset();
		S = S * oS * aS;	//oS:odom, aS:AMU
//		SSS = SSS * oS * aS;
		//cout << "S=" << endl << S << endl;
		
		
		/////////////////////////
		// --- TF ---
		/*static tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;

		transform.setOrigin(tf::Vector3(SSS(0,3), SSS(1,3), 0.0));
		q.setRPY(0, 0, YAW);
		cout<<"YAW=="<<YAW<<endl;
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "matching_base_link"));*/

		r.sleep();
		ros::spinOnce();
	}
	return (0);
}
