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
#include <scan_match2/util.h>
#include <tf/transform_broadcaster.h>

const size_t MAX_NODE = 10000;
const char filename1[] = "/home/amsl/3Dmap/infant/onda_test/aft.csv";
const char filename2[] = "/home/amsl/3Dmap/infant/onda_test/aft_top.csv";
// const char outputfile[] = "/home/amsl/3Dmap/infant/onda_test/aft_tmp.csv";
const char outputfile[] = "/home/amsl/3Dmap/infant/onda_test/bfr.csv";

//namespaces
using namespace std;	

typedef struct{
		int tgt;
		int src;
}EDGE_PAIR;

vector<EDGE_PAIR> p;

void load(size_t &e_cnt, size_t &e_cnt2, size_t &g_cnt, size_t &t_cnt, Matrix4f e_poses[MAX_NODE], Matrix4f e_poses2[MAX_NODE], Matrix4f g_poses[MAX_NODE], Matrix4f t_poses[MAX_NODE])
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

		fp = fopen(filename1,"r");
		if(fp == NULL){
				cout << "could not find file 'aft.csv'" << endl;
		}
		e_cnt = 0;
		while(1){
				// cout<<"EDGE read!!!"<<endl;
				float data[7] = {0};
				int tgt = 0;
				int src = 0;
				// float data[28] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF){
						// cout<<"No data..."<<endl;
						break;
				}
				if(strcmp(type,"EDGE_SE3:QUAT")==0){
						if(fscanf(fp,"%d %d %f %f %f %f %f %f %f", &tgt, &src, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 9)break;
						// if(fscanf(fp,"%*d %*d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7] &data[8], &data[9], &data[10], &data[11], &data[12], &data[13], &data[14], &data[15], &data[15], &data[16], &data[17], &data[18], &data[19], &data[20], &data[21], &data[22], &data[23], &data[24], &data[25], &data[26], &data[27]) != 28) break;

						Matrix4f e_pose = Matrix4f::Identity();
						e_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						e_pose(0,3) = data[0];
						e_pose(1,3) = data[1];
						e_pose(2,3) = data[2];
						e_poses[e_cnt] = e_pose;
						if(e_cnt == 0){
								EDGE_PAIR a = {0, 0};
								p.push_back(a);
						}
						else{
								EDGE_PAIR a = {tgt, src};
								p.push_back(a);
						}
						e_cnt ++;
				}
		}

		fclose(fp);

		fp = fopen(filename2,"r");
		if(fp == NULL){
				cout << "could not find file 'aft_top.csv'" << endl;
		}

		t_cnt = 0;
		while(1){
				float data[7] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF)break;
				if(strcmp(type,"VERTEX_SE3:QUAT")==0){
						if(fscanf(fp,"%*d %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 7)break;
						Matrix4f t_pose = Matrix4f::Identity();
						t_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						t_pose(0,3) = data[0];
						t_pose(1,3) = data[1];
						t_pose(2,3) = data[2];
						t_poses[t_cnt] = t_pose;
						// cout<<"NODE"<<g_cnt+t_cnt<<" "<<t_poses[t_cnt]<<endl;

						t_cnt ++;
				}
		}

		fclose(fp);

		fp = fopen(filename2,"r");
		if(fp == NULL){
				cout << "could not find file 'aft_top.csv'" << endl;
		}
		e_cnt2 = 0;
		while(1){
				// cout<<"EDGE read!!!"<<endl;
				float data[7] = {0};
				int tgt = 0;
				int src = 0;
				// float data[28] = {0};
				char type[1000];
				if(fscanf(fp,"%s",type) == EOF){
						// cout<<"No data..."<<endl;
						break;
				}
				if(strcmp(type,"EDGE_SE3:QUAT")==0){
						if(fscanf(fp,"%d %d %f %f %f %f %f %f %f", &tgt, &src, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6]) != 9)break;
						// if(fscanf(fp,"%*d %*d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7] &data[8], &data[9], &data[10], &data[11], &data[12], &data[13], &data[14], &data[15], &data[15], &data[16], &data[17], &data[18], &data[19], &data[20], &data[21], &data[22], &data[23], &data[24], &data[25], &data[26], &data[27]) != 28) break;

						Matrix4f e_pose = Matrix4f::Identity();
						e_pose.block(0,0,3,3) = quat2mat(data[3],data[4],data[5],data[6]);
						e_pose(0,3) = data[0];
						e_pose(1,3) = data[1];
						e_pose(2,3) = data[2];
						e_poses2[e_cnt2] = e_pose;
						EDGE_PAIR a = {tgt, src};
						p.push_back(a);
						
						e_cnt2 ++;
				}
		}

		fclose(fp);
}

void output(size_t &e_cnt, size_t &e_cnt2, size_t &g_cnt, size_t &t_cnt, Matrix4f e_poses[MAX_NODE], Matrix4f e_poses2[MAX_NODE], Matrix4f g_poses[MAX_NODE], Matrix4f t_poses[MAX_NODE])
{
		FILE *fp;
		if((fp=fopen(outputfile, "w")) == NULL){
				cout<<"can't open file!!\n"<<endl;
				exit(1);
		}
		for(size_t i=0; i<g_cnt; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << g_poses[i](0,0), g_poses[i](0,1), g_poses[i](0,2), g_poses[i](1,0), g_poses[i](1,1), g_poses[i](1,2), g_poses[i](2,0), g_poses[i](2,1), g_poses[i](2,2);
				quat = mat2quat(mat);

				fprintf(fp,"%s %d %f %f %f %f %f %f %f\n", "VERTEX_SE3:QUAT", (int)i, g_poses[i](0,3), g_poses[i](1,3), g_poses[i](2,3), quat(1), quat(2), quat(3), quat(0));
				if(i==0){
						fprintf(fp,"%s\n", "FIX 0");
				}
				// cout<<"NODE"<<i<<":"<<g_poses[i](0,3)<<" "<<g_poses[i](1,3)<<" "<<g_poses[i](2,3)<<endl;
				// cout<<"NODE"<<i<<":"<<g_poses[i]<<endl;
		}
		// for(size_t i=0; i<t_cnt; i++){
		// 		Eigen::Matrix3d mat;
		// 		Eigen::Vector4d quat;
        //
		// 		mat << t_poses[i](0,0), t_poses[i](0,1), t_poses[i](0,2), t_poses[i](1,0), t_poses[i](1,1), t_poses[i](1,2), t_poses[i](2,0), t_poses[i](2,1), t_poses[i](2,2);
		// 		quat = mat2quat(mat);
        //
		// 		fprintf(fp,"%s %d %f %f %f %f %f %f %f\n", "VERTEX_SE3:QUAT", (int)i+(int)g_cnt, t_poses[i](0,3), t_poses[i](1,3), t_poses[i](2,3), quat(1), quat(2), quat(3), quat(0)); 
		// 		// cout<<"NODE"<<i<<":"<<g_poses[i](0,3)<<" "<<g_poses[i](1,3)<<" "<<g_poses[i](2,3)<<endl;
		// 		// cout<<"NODE"<<i<<":"<<g_poses[i]<<endl;
		// }
		for(size_t i=0; i<e_cnt; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << e_poses[i](0,0), e_poses[i](0,1), e_poses[i](0,2), e_poses[i](1,0), e_poses[i](1,1), e_poses[i](1,2), e_poses[i](2,0), e_poses[i](2,1), e_poses[i](2,2);
				quat = mat2quat(mat);
				if(i<g_cnt){
						fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
								"EDGE_SE3:QUAT", p[i].tgt, p[i].src, 
								e_poses[i](0,3), e_poses[i](1,3), e_poses[i](2,3),quat(1), quat(2), quat(3), quat(0),
								1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								1.0, 0.0, 0.0, 0.0, 0.0,
								1.0, 0.0, 0.0, 0.0,
								1.0, 0.0, 0.0,
								1.0, 0.0,
								1.0);
				}
				else{
						fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
								"EDGE_SE3:QUAT", p[i].tgt, p[i].src, 
								e_poses[i](0,3), e_poses[i](1,3), e_poses[i](2,3),quat(1), quat(2), quat(3), quat(0),
								0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0,
								0.05, 0.0,
								0.05);
				}

				// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses[i](0,3)<<" "<<e_poses[i](1,3)<<" "<<e_poses[i](2,3)<<endl;
				// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses[i]<<endl;
		}
		for(size_t i=0; i<e_cnt2; i++){
				Eigen::Matrix3d mat;
				Eigen::Vector4d quat;

				mat << e_poses2[i](0,0), e_poses2[i](0,1), e_poses2[i](0,2), e_poses2[i](1,0), e_poses2[i](1,1), e_poses2[i](1,2), e_poses2[i](2,0), e_poses2[i](2,1), e_poses2[i](2,2);
				quat = mat2quat(mat);
				// fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
				// 				"EDGE_SE3:QUAT", p[i+e_cnt].tgt, p[i+e_cnt].src, 
				// 				e_poses2[i](0,3), e_poses2[i](1,3), e_poses2[i](2,3),quat(1), quat(2), quat(3), quat(0),
				// 				1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0, 0.0,
				// 				1.0, 0.0, 0.0,
				// 				1.0, 0.0,
				// 				1.0);
				fprintf(fp,"%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
								"EDGE_SE3:QUAT", p[i+e_cnt].tgt, p[i+e_cnt].src, 
								e_poses2[i](0,3), e_poses2[i](1,3), e_poses2[i](2,3),quat(1), quat(2), quat(3), quat(0),
								0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0, 0.0,
								0.05, 0.0, 0.0,
								0.05, 0.0,
								0.05);
				// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses2[i](0,3)<<" "<<e_poses2[i](1,3)<<" "<<e_poses2[i](2,3)<<endl;
				// cout<<"EDGE "<<p[i].tgt<<" "<<p[i].src<<":"<<e_poses2[i]<<endl;
		}
		fclose(fp);
}

int main (int argc, char** argv)
{
		Matrix4f e_poses[MAX_NODE];
		Matrix4f e_poses2[MAX_NODE];
		Matrix4f g_poses[MAX_NODE];
		Matrix4f t_poses[MAX_NODE];

		size_t e_cnt = 0;
		size_t e_cnt2 = 0;
		size_t g_cnt = 0;
		size_t t_cnt = 0;

		load(e_cnt, e_cnt2, g_cnt, t_cnt, e_poses, e_poses2, g_poses, t_poses);
		output(e_cnt, e_cnt2, g_cnt, t_cnt, e_poses, e_poses2, g_poses, t_poses);

		// system("mv aft.csv aft_tmp.csv");

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
