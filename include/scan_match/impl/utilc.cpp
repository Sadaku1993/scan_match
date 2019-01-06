int ask(const vector<string> &dat, const string &qry){
	for(size_t i=0;i<dat.size();i++){
		if(dat[i] == qry) return i;
	}
	return -1;
}

// void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointNormal> &cloud){
//###void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZ> &cloud){
void show(ros::NodeHandle n, const string topic, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud){
	static vector<string> dat;
	static vector<ros::Publisher> pub;
	int index = ask(dat, topic);
	if(index == -1){
		ros::Publisher tmp_pub = n.advertise<sensor_msgs::PointCloud2>(topic,1);
		pub.push_back(tmp_pub);
		dat.push_back(topic);
		index = (int)pub.size()-1;
	}
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(cloud, pc);
	pc.header.frame_id = "/map";
	pc.header.stamp = ros::Time::now();
	pub[index].publish(pc);
}
// void show(ros::NodeHandle n, const string topic, sensor_msgs::PointCloud &pc){
// 	static ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>(topic,1);
// 	pc.header.frame_id = "/map";
// 	pc.header.stamp = ros::Time::now();
// 	pub.publish(pc);
// }

//################# transformer !! #########################################################
pcl::PointCloud<pcl::PointXYZRGBNormal> transformer(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_org, Eigen::Matrix4f m){
// pcl::PointCloud<pcl::PointNormal> transformer(const pcl::PointCloud<pcl::PointNormal> &cloud_org, Eigen::Matrix4f m){
// pcl::PointCloud<pcl::PointXYZ> transformer(const pcl::PointCloud<pcl::PointXYZ> &cloud_org, Eigen::Matrix4f m){
	// pcl::PointCloud<pcl::PointNormal> cloud;
	//###pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    cloud.points.resize(cloud_org.points.size());
	for(size_t i=0;i<cloud.points.size();i++){
		Eigen::Vector4f p;
		Eigen::Vector4f n;
		p(0) = cloud_org.points[i].x;
		p(1) = cloud_org.points[i].y;
		p(2) = cloud_org.points[i].z;
		p(3) = 1.0;
		n(0) = cloud_org.points[i].normal_x;
		n(1) = cloud_org.points[i].normal_y;
		n(2) = cloud_org.points[i].normal_z;
		n(3) = 0.0;
		p = m * p;
		n = m * n;
		cloud.points[i].x = p(0);
		cloud.points[i].y = p(1);
		cloud.points[i].z = p(2);
		cloud.points[i].normal_x = n(0);
		cloud.points[i].normal_y = n(1);
		cloud.points[i].normal_z = n(2);
		cloud.points[i].curvature = cloud_org.points[i].curvature;
		cloud.points[i].r = cloud_org.points[i].r;
		cloud.points[i].g = cloud_org.points[i].g;
		cloud.points[i].b = cloud_org.points[i].b;
		// cloud.points[i].curvature = cloud_org.points[i].curvature;
		//###ark###
        // cloud.points[i].intensity = cloud_org.points[i].intensity;
	}
	return cloud;
}


Eigen::Vector4d mat2quat(Eigen::Matrix3d mat){
	Eigen::Vector4d q;
	q(0) = ( mat(0,0) + mat(1,1) + mat(2,2) + 1.0) / 4.0;
	q(1) = ( mat(0,0) - mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(2) = (-mat(0,0) + mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(3) = (-mat(0,0) - mat(1,1) + mat(2,2) + 1.0) / 4.0;
	if(q(0) < 0.0) q(0) = 0.0;
	if(q(1) < 0.0) q(1) = 0.0;
	if(q(2) < 0.0) q(2) = 0.0;
	if(q(3) < 0.0) q(3) = 0.0;
	q(0) = sqrt(q(0));
	q(1) = sqrt(q(1));
	q(2) = sqrt(q(2));
	q(3) = sqrt(q(3));
	if(q(0) >= q(1) && q(0) >= q(2) && q(0) >= q(3)) {
		q(0) *= +1.0;
		q(1) *= SIGN(mat(2,1) - mat(1,2));
		q(2) *= SIGN(mat(0,2) - mat(2,0));
		q(3) *= SIGN(mat(1,0) - mat(0,1));
	} else if(q(1) >= q(0) && q(1) >= q(2) && q(1) >= q(3)) {
		q(0) *= SIGN(mat(2,1) - mat(1,2));
		q(1) *= +1.0;
		q(2) *= SIGN(mat(1,0) + mat(0,1));
		q(3) *= SIGN(mat(0,2) + mat(2,0));
	} else if(q(2) >= q(0) && q(2) >= q(1) && q(2) >= q(3)) {
		q(0) *= SIGN(mat(0,2) - mat(2,0));
		q(1) *= SIGN(mat(1,0) + mat(0,1));
		q(2) *= +1.0;
		q(3) *= SIGN(mat(2,1) + mat(1,2));
	} else if(q(3) >= q(0) && q(3) >= q(1) && q(3) >= q(2)) {
		q(0) *= SIGN(mat(1,0) - mat(0,1));
		q(1) *= SIGN(mat(2,0) + mat(0,2));
		q(2) *= SIGN(mat(2,1) + mat(1,2));
		q(3) *= +1.0;
	} else {
		printf("coding error\n");
	}
	double r = NORM(q(0), q(1), q(2), q(3));
	q(0) /= r;
	q(1) /= r;
	q(2) /= r;
	q(3) /= r;

	return q;
}

Eigen::Vector4f mat2quat_f(Eigen::Matrix3f mat){
	Eigen::Vector4f q;
	q(0) = ( mat(0,0) + mat(1,1) + mat(2,2) + 1.0) / 4.0;
	q(1) = ( mat(0,0) - mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(2) = (-mat(0,0) + mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(3) = (-mat(0,0) - mat(1,1) + mat(2,2) + 1.0) / 4.0;
	if(q(0) < 0.0) q(0) = 0.0;
	if(q(1) < 0.0) q(1) = 0.0;
	if(q(2) < 0.0) q(2) = 0.0;
	if(q(3) < 0.0) q(3) = 0.0;
	q(0) = sqrt(q(0));
	q(1) = sqrt(q(1));
	q(2) = sqrt(q(2));
	q(3) = sqrt(q(3));
	if(q(0) >= q(1) && q(0) >= q(2) && q(0) >= q(3)) {
		q(0) *= +1.0;
		q(1) *= SIGN(mat(2,1) - mat(1,2));
		q(2) *= SIGN(mat(0,2) - mat(2,0));
		q(3) *= SIGN(mat(1,0) - mat(0,1));
	} else if(q(1) >= q(0) && q(1) >= q(2) && q(1) >= q(3)) {
		q(0) *= SIGN(mat(2,1) - mat(1,2));
		q(1) *= +1.0;
		q(2) *= SIGN(mat(1,0) + mat(0,1));
		q(3) *= SIGN(mat(0,2) + mat(2,0));
	} else if(q(2) >= q(0) && q(2) >= q(1) && q(2) >= q(3)) {
		q(0) *= SIGN(mat(0,2) - mat(2,0));
		q(1) *= SIGN(mat(1,0) + mat(0,1));
		q(2) *= +1.0;
		q(3) *= SIGN(mat(2,1) + mat(1,2));
	} else if(q(3) >= q(0) && q(3) >= q(1) && q(3) >= q(2)) {
		q(0) *= SIGN(mat(1,0) - mat(0,1));
		q(1) *= SIGN(mat(2,0) + mat(0,2));
		q(2) *= SIGN(mat(2,1) + mat(1,2));
		q(3) *= +1.0;
	} else {
		printf("coding error\n");
	}
	double r = NORM(q(0), q(1), q(2), q(3));
	q(0) /= r;
	q(1) /= r;
	q(2) /= r;
	q(3) /= r;

	return q;
}



void mat2rpy(Matrix4f m, float &r, float &p, float &y){
	r = atan2(m(2,1), m(2,2));
	p = asin(-m(2,0));
	y = atan2(m(1,0), m(0,0));
}

//############## rpy2Mat ###########################################
Eigen::Matrix3d rpy2mat(double roll, double pitch, double yaw){
	Eigen::Matrix3d rsl = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d p = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d y = Eigen::Matrix3d::Identity();

	r(1,1)=cos(roll); r(1,2)=-sin(roll);
	r(2,1)=sin(roll); r(2,2)=cos(roll);

	p(0,0)=cos(pitch); p(0,2)=sin(pitch);
	p(2,0)=-sin(pitch); p(2,2)=cos(pitch);

	y(0,0)=cos(yaw); y(0,1)=-sin(yaw);
	y(1,0)=sin(yaw); y(1,1)=cos(yaw);

	rsl = y * p * r;
	return rsl;
}

//##################### clacMat ######################################
inline Matrix4f calcMat(float roll, float pitch, float yaw){
	Matrix3d tmp = rpy2mat((double)roll,(double)pitch,(double)yaw);
	//Matrix3d tmp = rpy2mat(0.0,0.0,yaw);
	Matrix4f rsl = Matrix4f::Identity();
	for(size_t i=0;i<3;i++)
		for(size_t j=0;j<3;j++)
			rsl(i,j)=tmp(i,j);
	
	return rsl;
}


void tf_bro(Eigen::Matrix4f mat, string child, string parent){
	tf::Vector3 origin;
	tf::Matrix3x3 tf3d;
	tf::Quaternion tfqt;
	static tf::Transform transform;
	static tf::TransformBroadcaster tf_broadcaster;

	origin.setValue(static_cast<double>(mat(0,3)),static_cast<double>(mat(1,3)),static_cast<double>(mat(2,3)));
	tf3d.setValue(static_cast<double>(mat(0,0)), static_cast<double>(mat(0,1)), static_cast<double>(mat(0,2)), 
			static_cast<double>(mat(1,0)), static_cast<double>(mat(1,1)), static_cast<double>(mat(1,2)), 
			static_cast<double>(mat(2,0)), static_cast<double>(mat(2,1)), static_cast<double>(mat(2,2)));
	tf3d.getRotation(tfqt);
	transform.setRotation(tfqt);
	transform.setOrigin(origin);
	//	static int cnt = 0;
	//	char *str;
	//	sprintf(str,"tf_%d",cnt);
	//	cnt ++;
	tf_broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(), parent, child));
}

Matrix3f quat2mat(float x,float y,float z,float w){
	Matrix3f rsl = Matrix3f::Identity();
	rsl << 
		1-2*y*y-2*z*z,	2*x*y+2*w*z,	2*x*z-2*w*y,
		2*x*y-2*w*z,	1-2*x*x-2*z*z,	2*y*z+2*w*x,
		2*x*z+2*w*y,	2*y*z-2*w*x,	1-2*x*x-2*y*y;
	return rsl.inverse();
}
