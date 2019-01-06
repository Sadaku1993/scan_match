/*
 * src: gicp.cpp
 * 
 * memo:
 *		【buffer】から二点群をSubscribeして，ICP処理(GICP)により相対位置・方位を算出し,
 *		 NodeとEdgeの位置情報をbfr.csvとして保存するsrc
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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <time_util/stopwatch.h>
//#include <ceres_msgs/AMU_data.h>
#include <Eigen/Core>
//#include <usr_msgs/Edge.h>//ark:Edge情報含有(usr_msgs/cpp/include/usr_msgs/Edge.h;pclの型は無い模様
#include <scan_match/util.h>
#include <tf/transform_broadcaster.h>
#include <AMU_data.h>
#include <Edge.h>

using namespace Eigen;

// typedef pcl::PointNormal PN;
typedef pcl::PointXYZINormal PN;
typedef pcl::PointCloud<PN> CloudN;
typedef CloudN::Ptr CloudNPtr;
typedef CloudN::ConstPtr CloudNConstPtr;
//###### pcl::PointXYZINormal version #########
// typedef pcl::PointXYZINormal PIN;
// typedef pcl::PointCloud<PIN> CloudIN;
// typedef CloudIN::Ptr CloudINPtr;
// typedef CloudIN::ConstPtr CloudINConstPtr;
// ###############################################

const double OFFSET = 0.0040410296;
float DT = 2.0;//1.0;//0.2;
float hosei = 1.0;

float odom_v = 0.0;
float yawrate = 0.0;
float pitchrate = 0.0;
float rollrate = 0.0;
float amu_roll = 0.0;
float amu_pitch = 0.0;
float amu_yaw = 0.0;

bool is_new = false;
double MaxCorrespondenceDistance = 0.3;
// double MaximumIterations = 250;
double MaximumIterations = 20;
double TransformationEpsilon = 1e-8;
double EuclideanFitnessEpsilon = 1e-8;

usr_msgs::Edge edge;

Matrix4f S = Matrix4f::Identity();

Stopwatch sw;

bool seg_pnt_flag = false;
sensor_msgs::PointCloud2 seg_pnt_;

//src_cloud と tgt_cloudの相対初期位置と両者の点群情報取得
void callback(usr_msgs::Edge::ConstPtr msg)
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
void check(void)
{
	static int num = 0;
	cout << "check " << num << endl;
	num ++;
}


int main (int argc, char** argv)
{

	//FILE *fp=fopen("bfr.csv","w");

	//fprintf(fp,"%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	printf("%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	ros::init(argc, argv, "GICP_azm"); //define node name.
	ros::NodeHandle n;
	ros::Rate r(100);

	//---Subscriber---//
	ros::Subscriber sub = n.subscribe("/from_buffer",5,callback);
	ros::Subscriber segpnt_sub = n.subscribe("/seg_img_from_buffer",5,segpnt_callback);
	//---Publisher---//
	// ros::Publisher smatch_pose_pub = n.advertise<geometry_msgs::Pose>("/scanmatch_pose",10);
    ros::Publisher pub_pc = n.advertise<sensor_msgs::PointCloud2>("/final",10);
	ros::Publisher pub_now = n.advertise<sensor_msgs::PointCloud>("/now_position",1);

	sensor_msgs::PointCloud pc_now;
	pc_now.header.frame_id = "/map";

	//---TF---
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	sw.start();
	//	getParams(n);

	//sensor_msgs::PointCloud path;
	sensor_msgs::PointCloud2 final_pc;

	Eigen::Matrix4f sum = Eigen::Matrix4f::Identity();

	Eigen::Matrix3d m_clb;
	Eigen::Vector4d q_clb;
	m_clb = rpy2mat(0.0, -0.81, 0.0);
	q_clb = mat2quat(m_clb);

	// pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp; 
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp; 
	// gicp.setMaxCorrespondenceDistance (1.5);//対応距離の最大値
	gicp.setMaxCorrespondenceDistance (0.5);//対応距離の最大値
	// gicp.setMaximumIterations (200);        //ICPの最大繰り返し回数
	gicp.setMaximumIterations (100);        //ICPの最大繰り返し回数
	gicp.setTransformationEpsilon (1e-8);   //変換パラメータ値
	gicp.setEuclideanFitnessEpsilon (1e-8); //..

	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src_v (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt_v (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tmp_v (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointNormal>);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_src_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tgt_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_tmp_v (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZINormal>);

	double pos[3] = {0.0};
	double pos_star[3] = {0.0};


	while(ros::ok() ){
		//if(is_new){
		seg_pnt_flag = true;
		if(is_new && seg_pnt_flag){
			// if(seg_pnt_flag){ // get segmented point information 	
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc_c (new pcl::PointCloud<pcl::PointXYZRGB>);
			{
				fromROSMsg(seg_pnt_, *pcl_pc_c);
			}	
			char seg_pnt_file[100];
			//sprintf(seg_pnt_file, "seg_points/seg_pnt_%d.pcd", (int)edge.idx_src);
			//pcl::io::savePCDFileBinary(seg_pnt_file, *pcl_pc_c);
			// }
			Matrix4f S = Matrix4f::Identity();
			Matrix3f e = quat2mat(edge.r_pose.pose.orientation.x, edge.r_pose.pose.orientation.y, edge.r_pose.pose.orientation.z, edge.r_pose.pose.orientation.w );
			for(size_t i=0;i<3;i++)for(size_t j=0;j<3;j++)	S(i,j) = e(i,j);
			S(0,3) = edge.r_pose.pose.position.x;
			S(1,3) = edge.r_pose.pose.position.y;
			S(2,3) = edge.r_pose.pose.position.z;
			fromROSMsg(edge.cloud_tgt, *cloud_tgt);
			fromROSMsg(edge.cloud_src, *cloud_src);



			static bool is_first = true;
			if(is_first){
				//fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
				printf("%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
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

			*cloud_tmp   = transformer(*cloud_src, S);
			*cloud_tmp_v = transformer(*cloud_tmp, sum); //いらない 
			*cloud_src_v = transformer(*cloud_src, sum); //いらない

			gicp.setInputSource(cloud_tmp); 
			// gicp.setInputCloud(cloud_tmp); 
			gicp.setInputTarget(cloud_tgt);
			// pcl::PointCloud<pcl::PointNormal> Final; 
			pcl::PointCloud<pcl::PointXYZINormal> Final; 
			gicp.align(Final);


			Eigen::Matrix4f F;
			F = gicp.getFinalTransformation(); // GICPで求めた変換行列

			Eigen::Matrix4f hantei;
			Eigen::Matrix4f idn = Eigen::Matrix4f::Identity();
			hantei = F;
			hantei = hantei - idn;

			//	std::cout << "GICP has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl; 
			//	std::cout << gicp.getFinalTransformation() << std::endl; 
			Eigen::Matrix4f tmp;
			//cout << hantei.norm() << endl;

			//---GICPから算出した変換行列を使用するかどうかの判定----
			if(hantei.norm() < 3.0){//0.2
				// if(hantei.norm() < 9.2){//0.2
				tmp = F * S;
			}else{
				tmp = S;
				cout << "xxxxxxxxxxxxxxxx matching miss!!!! xxxxxxxxxxxxxxxxxxxx" << endl;
			}
			sum = sum * tmp;
			//std::cout << icp.getFinalTransformation() << std::endl;
			//edge.r_pose = trans(S);


			Eigen::Matrix3d mat;
			Eigen::Vector4d quat;

			// ↓ nodes.csv 各頂点の位置姿勢を記述
			mat << sum(0,0), sum(0,1), sum(0,2), sum(1,0), sum(1,1), sum(1,2), sum(2,0), sum(2,1), sum(2,2);
			quat = mat2quat(mat);
			//fprintf(fp,"%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", (int)edge.idx_src, sum(0,3), sum(1,3), sum(2,3), quat(1), quat(2), quat(3), quat(0));
			printf("%s %d %f %f %f %f %f %f %f\n","VERTEX_SE3:QUAT", (int)edge.idx_src, sum(0,3), sum(1,3), sum(2,3), quat(1), quat(2), quat(3), quat(0));
			//edges.csv
			mat << tmp(0,0), tmp(0,1), tmp(0,2), tmp(1,0), tmp(1,1), tmp(1,2), tmp(2,0), tmp(2,1), tmp(2,2);
			quat = mat2quat(mat);


			//連続するフレーム間の相対位置姿勢の記述
			//fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			printf("%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					"EDGE_SE3:QUAT", (int)edge.idx_tgt, (int)edge.idx_src, 
					tmp(0,3), tmp(1,3), tmp(2,3),quat(1), quat(2), quat(3), quat(0),
					1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0, 0.0,
					1.0, 0.0, 0.0,
					1.0, 0.0,
					1.0);

			//fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			/*		printf("%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
					"EDGE_SE3:QUAT", 0, (int)edge.idx_src, 
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0,
					0.005, 0.0, 0.0, 0.0,
					0.01, 0.0, 0.0,
					0.01, 0.0,
					0.0);
			 */
			char pcd_file[100];
			sprintf(pcd_file,"clouds/cloud_%d.pcd",(int)edge.idx_src);
			pcl::io::savePCDFileBinary(pcd_file, *cloud_src);

			// Final = transformer(*cloud_src, gicp.getFinalTransformation() * S);
			Final = transformer(*cloud_src, sum);
			*cloud_map = transformer(*cloud_src, sum);

			// *cloud_src_v = transformer(*cloud_tmp, sum);
			*cloud_tgt_v = transformer(*cloud_tgt, sum*tmp.inverse());

			toROSMsg(Final, final_pc);
			geometry_msgs::Point32 p_tmp;
			p_tmp.x = sum(0,3);
			p_tmp.y = sum(1,3);
			p_tmp.z = sum(2,3);
			pc_now.points.push_back(p_tmp);
			// pc_now.points[0].x = sum(0,3);
			// pc_now.points[0].y = sum(1,3);
			// pc_now.points[0].z = sum(2,3);

			pub_now.publish(pc_now);
			pub_pc.publish(final_pc);

			pc_now.points.clear();

			show(n,"source", *cloud_src_v);
			show(n,"target", *cloud_tgt_v);
			show(n,"tmp", *cloud_tmp_v);
			show(n,"final", Final);
			static int v_cnt = 0;
			if((v_cnt % 1)==0)
				show(n,"map", *cloud_map); 
			v_cnt ++;

			pos[0] = sum(0,3);
			pos[1] = sum(1,3);
			pos[2] = sum(2,3);

			//cout << "-------------------" << endl;
			is_new = false;
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

		//fclose(fp);

		return 0;
	}
