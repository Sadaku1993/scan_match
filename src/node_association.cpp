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

#include <Eigen/Core>

//user defined
#include <scan_match2/util.h>
#include <tf/transform_broadcaster.h>

const size_t MAX_NODE = 10000;
const char filename1[] = "/home/amsl/3Dmap/infant/onda_test/aft.csv";
const char filename2[] = "/home/amsl/topological_node.csv";
const char outputfile[] = "/home/amsl/3Dmap/infant/onda_test/aft_top.csv";

//namespaces
using namespace std;	

typedef struct{
		int tgt;
		int src;
}EDGE_PAIR;

vector<EDGE_PAIR> p;

void load(size_t &r_cnt, size_t &e_cnt, size_t &g_cnt, size_t &t_cnt, Matrix4f r_poses[MAX_NODE], Matrix4f e_poses[MAX_NODE], Matrix4f g_poses[MAX_NODE], Matrix4f t_poses[MAX_NODE])
{
		FILE *fp;
		fp = fopen(filename1,"r");
		if(fp == NULL){
				cout << "could not find file 'aft.csv'" << endl;
		}
		g_cnt = 0;
		while(1){
				float data[7] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF)break;
				if(strcmp(type,"VERTEX_SE3:QUAT")==0){
						if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
						Matrix4f g_pose = Matrix4f::Identity();
						g_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						g_pose(0,3) = data[0];
						g_pose(1,3) = data[1];
						g_pose(2,3) = data[2];
						g_poses[g_cnt] = g_pose;
						g_cnt ++;
				}
		}
		fclose(fp);

		fp = fopen(filename2,"r");
		if(fp == NULL){
				cout << "could not find file 'topological_node.csv'" << endl;
		}

		t_cnt = 0;
		while(1){
				float data[7] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF)break;
				if(strcmp(type,"INTERSECTION")==0){
						if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
						Matrix4f t_pose = Matrix4f::Identity();
						t_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						t_pose(0,3) = data[0];
						t_pose(1,3) = data[1];
						t_pose(2,3) = data[2];
						t_poses[t_cnt] = t_pose;
						t_cnt ++;
				}
		}

		fclose(fp);

		fp = fopen(filename2,"r");
		if(fp == NULL){
				cout << "could not find file 'topological_node.csv'" << endl;
		}

		r_cnt = 0;
		while(1){
				float data[7] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF)break;
				if(strcmp(type,"ROOM")==0){
						if(fscanf(fp,"%*d %*s %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
						Matrix4f r_pose = Matrix4f::Identity();
						r_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						r_pose(0,3) = data[0];
						r_pose(1,3) = data[1];
						r_pose(2,3) = data[2];
						r_poses[r_cnt] = r_pose;
						r_cnt ++;
				}
		}
		fclose(fp);
}

void nearest_point_detector(size_t &r_cnt, size_t &e_cnt, size_t &g_cnt, size_t &t_cnt, Matrix4f r_poses[MAX_NODE], Matrix4f e_poses[MAX_NODE], Matrix4f g_poses[MAX_NODE], Matrix4f t_poses[MAX_NODE])
{
		for(size_t i=0; i<t_cnt; i++){
				double min = 100;
				double dx = 0.0;
				double dy = 0.0;
				double dz = 0.0;
				double yaw = 0.0;
				int pair_num1 = 0;
				int pair_num2 = 0;
				for(size_t j=0; j<g_cnt; j++){
						double dist = sqrt(pow((double)(t_poses[i](0,3)-g_poses[j](0,3)), 2) + pow((double)(t_poses[i](1,3)-g_poses[j](1,3)), 2));
						if(dist<min){
								pair_num2 = pair_num1;
								min = dist;
								pair_num1 = (int)j;
						}	
				}
				Matrix4f pose = Matrix4f::Identity();
				// dx = t_poses[i](0,3)-g_poses[pair_num1](0,3);
				// dy = t_poses[i](1,3)-g_poses[pair_num1](1,3);
				// dz = t_poses[i](2,3)-g_poses[pair_num](2,3);
				dx = g_poses[pair_num1](0,3)-t_poses[i](0,3);
				dy = g_poses[pair_num1](1,3)-t_poses[i](1,3);
				// cout<<"t="<<i<<"pair="<<pair_num1<<endl;
				// cout<<"dx="<<dx<<" dy="<<dy<<endl;
				// dz = g_poses[pair_num1](2,3)-t_poses[i](2,3);
				// theta = atan2(t_poses[i](1,3)-g_poses[pair_num](1,3), t_poses[i](0,3)-g_poses[pair_num](0,3));
				yaw = atan2(dy, dx);
				// cout<<"yaw="<<yaw<<endl;
				// tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
				Eigen::Matrix3d m_clb;
				Eigen::Vector4d q_clb;
				// m_clb = rpy2mat(0.0, 0.0, yaw);
				m_clb = rpy2mat(0.0, 0.0, 0.0);
				q_clb = mat2quat(m_clb);
				// cout<<q_clb<<endl;

				pose.block(0,0,3,3) = quat2mat(q_clb(1),q_clb(2),q_clb(3),q_clb(0));
				// pose.block(0,0,3,3) = m_clb;
				pose(0,3) = dx;
				pose(1,3) = dy;
				pose(2,3) = dz;
				e_poses[e_cnt] = pose; 
				// EDGE_PAIR a = {pair_num, ((int)i+(int)g_cnt)};
				EDGE_PAIR a = {((int)i+(int)g_cnt), pair_num1};
				// cout<<"top_pair1 "<<a.tgt<<" "<<a.src<<endl;
				p.push_back(a);
				e_cnt ++;
		}
		for(size_t i=0; i<r_cnt; i++){
				double min = 100;
				double dx = 0.0;
				double dy = 0.0;
				double dz = 0.0;
				double yaw = 0.0;
				int pair_num1 = 0;
				int pair_num2 = 0;
				for(size_t j=0; j<t_cnt; j++){
						double dist = sqrt(pow((double)(r_poses[i](0,3)-t_poses[j](0,3)), 2) + pow((double)(r_poses[i](1,3)-t_poses[j](1,3)), 2));
						if(dist<min){
								pair_num2 = pair_num1;
								min = dist;
								pair_num1 = (int)j;
						}	
				}
				Matrix4f pose = Matrix4f::Identity();
				// dx = t_poses[i](0,3)-g_poses[pair_num1](0,3);
				// dy = t_poses[i](1,3)-g_poses[pair_num1](1,3);
				// dz = t_poses[i](2,3)-g_poses[pair_num](2,3);
				dx = fabs(t_poses[pair_num1](0,3)-r_poses[i](0,3));
				dy = fabs(t_poses[pair_num1](1,3)-r_poses[i](1,3));
				cout<<"dx="<<dx<<" dy="<<dy<<endl;
				// dz = g_poses[pair_num1](2,3)-t_poses[i](2,3);
				// theta = atan2(t_poses[i](1,3)-g_poses[pair_num](1,3), t_poses[i](0,3)-g_poses[pair_num](0,3));
				yaw = atan2(dy, dx);
				// cout<<"yaw="<<yaw<<endl;
				// tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
				Eigen::Matrix3d m_clb;
				Eigen::Vector4d q_clb;
				// m_clb = rpy2mat(0.0, 0.0, yaw);
				m_clb = rpy2mat(0.0, 0.0, 0.0);
				q_clb = mat2quat(m_clb);
				// cout<<q_clb<<endl;

				pose.block(0,0,3,3) = quat2mat(q_clb(1),q_clb(2),q_clb(3),q_clb(0));
				// pose.block(0,0,3,3) = m_clb;
				pose(0,3) = dx;
				pose(1,3) = dy;
				pose(2,3) = dz;
				e_poses[e_cnt] = pose; 
				// EDGE_PAIR a = {pair_num1+(int)g_cnt, ((int)i+(int)g_cnt+(int)t_cnt)};
				EDGE_PAIR a = {((int)i+(int)g_cnt+(int)t_cnt), pair_num1+(int)g_cnt};
				// cout<<"room_pair1 "<<a.tgt<<" "<<a.src<<endl;
				p.push_back(a);
				e_cnt ++;
		}

}

void output(size_t &r_cnt, size_t &e_cnt, size_t &g_cnt, size_t &t_cnt, Matrix4f r_poses[MAX_NODE], Matrix4f e_poses[MAX_NODE], Matrix4f g_poses[MAX_NODE], Matrix4f t_poses[MAX_NODE])
{
		FILE *fp;
		if((fp=fopen(outputfile, "w")) == NULL){
				cout<<"can't open file!!\n"<<endl;
				exit(1);
		}
		for(size_t i=0; i<t_cnt; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << t_poses[i](0,0), t_poses[i](0,1), t_poses[i](0,2), t_poses[i](1,0), t_poses[i](1,1), t_poses[i](1,2), t_poses[i](2,0), t_poses[i](2,1), t_poses[i](2,2);
				quat = mat2quat(mat);

				fprintf(fp,"%s %d %f %f %f %f %f %f %f\n", "VERTEX_SE3:QUAT", (int)i+(int)g_cnt, t_poses[i](0,3), t_poses[i](1,3), t_poses[i](2,3), quat(1), quat(2), quat(3), quat(0)); 
				// cout<<"TOP_NODE"<<(int)i+(int)g_cnt+1<<":"<<t_poses[i](0,3)<<" "<<t_poses[i](1,3)<<" "<<t_poses[i](2,3)<<endl;
		}
		for(size_t i=0; i<r_cnt; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << r_poses[i](0,0), r_poses[i](0,1), r_poses[i](0,2), r_poses[i](1,0), r_poses[i](1,1), r_poses[i](1,2), r_poses[i](2,0), r_poses[i](2,1), r_poses[i](2,2);
				quat = mat2quat(mat);

				fprintf(fp,"%s %d %f %f %f %f %f %f %f\n", "VERTEX_SE3:QUAT", (int)i+(int)g_cnt+(int)t_cnt, r_poses[i](0,3), r_poses[i](1,3), r_poses[i](2,3), quat(1), quat(2), quat(3), quat(0)); 
				// cout<<"TOP_NODE"<<(int)i+(int)g_cnt+1<<":"<<t_poses[i](0,3)<<" "<<t_poses[i](1,3)<<" "<<t_poses[i](2,3)<<endl;
		}
		for(size_t i=0; i<e_cnt; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << e_poses[i](0,0), e_poses[i](0,1), e_poses[i](0,2), e_poses[i](1,0), e_poses[i](1,1), e_poses[i](1,2), e_poses[i](2,0), e_poses[i](2,1), e_poses[i](2,2);
				quat = mat2quat(mat);
				// fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
				// 				"EDGE_SE3:QUAT", p[i].tgt, p[i].src, 
				// 				e_poses[i](0,3), e_poses[i](1,3), e_poses[i](2,3),quat(1), quat(2), quat(3), quat(0),
				// 				1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0,
				// 				1.0, 0.0,
				// 				1.0);
				fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
								"EDGE_SE3:QUAT", p[i].tgt, p[i].src, 
								e_poses[i](0,3), e_poses[i](1,3), e_poses[i](2,3),quat(1), quat(2), quat(3), quat(0),
								0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0,
								0.05, 0.0,
								0.05);
				// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses[i](0,3)<<" "<<e_poses[i](1,3)<<" "<<e_poses[i](2,3)<<endl;
		}
		fclose(fp);
}


int main (int argc, char** argv)
{
		Matrix4f r_poses[MAX_NODE];
		Matrix4f e_poses[MAX_NODE];
		Matrix4f g_poses[MAX_NODE];
		Matrix4f t_poses[MAX_NODE];

		size_t r_cnt = 0;
		size_t e_cnt = 0;
		size_t g_cnt = 0;
		size_t t_cnt = 0;

		load(r_cnt, e_cnt, g_cnt, t_cnt, r_poses, e_poses, g_poses, t_poses);
		nearest_point_detector(r_cnt, e_cnt, g_cnt, t_cnt, r_poses, e_poses, g_poses, t_poses);
		output(r_cnt, e_cnt, g_cnt, t_cnt, r_poses, e_poses, g_poses, t_poses);

		// for(size_t i=0; i<g_cnt; i++){
		// 		// cout<<"NODE"<<i<<":"<<g_poses[i](0,3)<<" "<<g_poses[i](1,3)<<" "<<g_poses[i](2,3)<<endl;
		// 		cout<<"NODE"<<i<<":"<<g_poses[i]<<endl;
		// }
		// for(size_t i=0; i<e_cnt; i++){
		// 		// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses[i](0,3)<<" "<<e_poses[i](1,3)<<" "<<e_poses[i](2,3)<<endl;
		// 		cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses[i]<<endl;
		// }
		// for(size_t i=0; i<t_cnt; i++){
		// 		cout<<"TOP_NODE"<<i<<":"<<t_poses[i](0,3)<<" "<<t_poses[i](1,3)<<" "<<t_poses[i](2,3)<<endl;
		// }

		return 0;
}
