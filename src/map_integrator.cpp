/*
 * src: map_integrator .cpp
 * Author: s.shimizu
 * created   : 20xx.xx.xx
 * lastupdate: 20xx.xx.xx
 *
 * memo: csvファイルに保存されているNodeの位置情報を基に, cloudsディレクトリに保存されている点群を
 *		 座標変換し, 形状地図(.pcd)を作成するsrc
 *
 */

#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//sensor_msgs
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Imu.h>

//nav_msgs
//#include <nav_msgs/Odometry.h>

//geometry_msgs

//user defined
//#include <ceres_msgs/AMU_data.h>
//#include<>


//namespaces
using namespace std;	
typedef pcl::PointCloud<pcl::PointNormal> CloudN;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr CloudNPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> CloudIN;
typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr CloudINPtr;
typedef vector<Eigen::Matrix4f> Nodes;
typedef vector<pcl::PointCloud<pcl::PointNormal> > CloudNs;

const string OUTPUT_PATH = "map/map_0.pcd";
//const string OUTPUT_PATH = "refined/map_0.pcd";

//CloudN whole_map;
CloudIN whole_map;

CloudIN shift(Eigen::Matrix4f m, CloudIN cloud){
	CloudIN rsl;
	//	cout << "mat=" << endl <<  m << endl << endl;
	for(size_t i=0;i<cloud.points.size();i++){
		Eigen::Vector4f p;
		Eigen::Vector4f n;
		p << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, 1.0;
		n << cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z, 0.0;
		p = m*p;
		n = m*n;
		pcl::PointXYZINormal pnt;
		pnt.x = p(0);
		pnt.y = p(1);
		pnt.z = p(2);
		pnt.normal_x = n(0);
		pnt.normal_y = n(1);
		pnt.normal_z = n(2);
		pnt.curvature = cloud.points[i].curvature;
		pnt.intensity = cloud.points[i].intensity;
		rsl.push_back(pnt);
	}
	return rsl;
}

//void addCloud(const CloudN &cloud){
void addCloud(const CloudIN &cloud){
	for(size_t i=0;i<cloud.points.size();i++){
		whole_map.push_back(cloud.points[i]);
	}
}

// void merge(const Nodes &nodes, const CloudNs &clouds){
// 	for(size_t i=0;i<nodes.size();i++){
// 		addCloud( shift(nodes[i],clouds[i]) );
// 	}
// }

Eigen::Matrix4f trans(float dat[7]){
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f r;
	/*
	   m(0,0) = 1.0f - 2.0f * dat[4] * dat[4] - 2.0f * dat[5] * dat[5];
	   m(0,1) = 2.0f * dat[3] * dat[4] + 2.0f * dat[6] * dat[5];
	   m(0,2) = 2.0f * dat[3] * dat[5] - 2.0f * dat[6] * dat[4];

	   m(1,0) = 2.0f * dat[3] * dat[4] - 2.0f * dat[6] * dat[5];
	   m(1,1) = 1.0f - 2.0f * dat[3] * dat[3] - 2.0f * dat[5] * dat[5];
	   m(1,2) = 2.0f * dat[4] * dat[5] + 2.0f * dat[6] * dat[3];

	   m(2,0) = 2.0f * dat[3] * dat[5] + 2.0f * dat[6] * dat[4];
	   m(2,1) = 2.0f * dat[4] * dat[5] - 2.0f * dat[6] * dat[3];
	   m(2,2) = 1.0f - 2.0f * dat[3] * dat[3] - 2.0f * dat[4] * dat[4];
	 */////////////
	m(0,0) = 1.0f - 2.0f * dat[4] * dat[4] - 2.0f * dat[5] * dat[5];
	m(1,0) = 2.0f * dat[3] * dat[4] + 2.0f * dat[6] * dat[5];
	m(2,0) = 2.0f * dat[3] * dat[5] - 2.0f * dat[6] * dat[4];

	m(0,1) = 2.0f * dat[3] * dat[4] - 2.0f * dat[6] * dat[5];
	m(1,1) = 1.0f - 2.0f * dat[3] * dat[3] - 2.0f * dat[5] * dat[5];
	m(2,1) = 2.0f * dat[4] * dat[5] + 2.0f * dat[6] * dat[3];

	m(0,2) = 2.0f * dat[3] * dat[5] + 2.0f * dat[6] * dat[4];
	m(1,2) = 2.0f * dat[4] * dat[5] - 2.0f * dat[6] * dat[3];
	m(2,2) = 1.0f - 2.0f * dat[3] * dat[3] - 2.0f * dat[4] * dat[4];
	////////////////
	m(0,3) = dat[0];
	m(1,3) = dat[1];
	m(2,3) = dat[2];

	return m;
	//return m.inverse();
}



int main (int argc, char** argv)
{

	cout << "output_path = " << OUTPUT_PATH << endl;
	size_t max_index = 0;
	cout << "max index = ";
	cin >> max_index ;
	cout << "skip = ";
	size_t skip = 0;
	cin >> skip;

	ros::init(argc, argv, "MAP_INTEGRATOR"); //define node name.
	ros::NodeHandle n;
	ros::Rate roop(100);          //Set Rate[Hz].
	//	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>(topic,1);
	//	ros::Subscriber sub = n.subscribe("/sub_topic",1,callback);
	//	sensor_msgs::PointCloud pc;
	//	pc.header.frame_id = frame;


	/////graphファイルの読み込み
	FILE *fp;
	fp = fopen("aft.csv","r");
	//fp = fopen("bfr.csv","r");
	if(fp == NULL){
		cerr << "could not load aft.csv" << endl;
		return 1;
	}
	char s[100];
	size_t cnt = 0;
	while(fscanf(fp,"%s",s) != EOF){
		float dat[7];
		int num;
		if (strcmp(s, "VERTEX_SE3:QUAT") == 0){

			if(fscanf(fp,"%d %f %f %f %f %f %f %f",&num, &dat[0], &dat[1], &dat[2], &dat[3], &dat[4], &dat[5], &dat[6]) != 8)break;
			// if(num == 0)continue;
			if(num % skip)continue;
			Eigen::Matrix4f node;
			node = trans(dat);

			cout << "num = " << num << endl;
			char filename[100];
			sprintf(filename,"clouds/cloud_%d.pcd",num);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
			// pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
			if (pcl::io::loadPCDFile<pcl::PointXYZINormal> (filename, *cloud) == -1){
			// if (pcl::io::loadPCDFile<pcl::PointNormal> (filename, *cloud) == -1){
				cout << "cloud_" << num << "not found." << endl;
				//break;
			}


			addCloud( shift(node, *cloud) );
			cout << cnt << " / " << max_index << endl << endl;
			if(cnt == max_index)break;
			cnt += skip;
		}
	}

	pcl::io::savePCDFileBinary(OUTPUT_PATH, whole_map);
	cout << "num = " << cnt << endl;
	return 0;
}

