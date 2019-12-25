#include "util.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <set>

using namespace std;

int read_point_file(string path,vector<Point>& points){
	ifstream in(path.c_str());
	if(!in.is_open()) return -1;
	string buf;
	stringstream ss;
	while(getline(in,buf)){
		ss.clear();
		ss.str("");
		ss << buf;
		Point p;
		ss >>p;
		points.push_back(p);	
	}
	#ifdef COUT_INTERMEDIATE
	cout << "Reading point file:["<<path << "] Point number:" << points.size() << endl;
	#endif
}



int write_point_file(vector<Point>& points,string path){
	
	#ifdef COUT_INTERMEDIATE
	cout << "Writing point file:["<<path << "] Point number:" << points.size() << endl;
	#endif
	ofstream out(path.c_str());
	if(!out.is_open()) return -1;
	for(auto e: points) out << e << endl;
}



int write_point_file(vector<Point>& points,string path);




Plane _get_plane(vector<Point>& points){
	set<int> ids;
	while(ids.size() < 3){
		int id = rand() % points.size();
		ids.insert(id);
	}
	vector<Point> three_points;
	for(auto id:ids){
		three_points.push_back(points[id]);
	}
	return Plane(three_points[0],three_points[1],three_points[2]);
}

float_t _error_sum(vector<Point>& points,Plane& plane){
	float_t res = (float_t)0;
	
	for(auto p:points){
		res += plane.distance(p);
	}
	
	return res;
}

vector<Point> _vote(vector<Point>& points,Plane& plane,float_t distance_max){
	
	vector<Point> res;
	for(auto p:points){
		if(abs(plane.distance(p)) <= distance_max)
			res.push_back(p);
	}	
	return res;
	
}

Plane ransac_iter(vector<Point>& points,vector<Point>& points_filtered,float_t ransac_distance_max,int ransac_iter_times){
	Plane best_plane = _get_plane(points);
	vector<Point> best_subset;
	int max_vote = INT_MIN;
	
	
	int iter_cur = 0;
	while(iter_cur++ < ransac_iter_times){
		Plane cur_plane = _get_plane(points);
		vector<Point> cur_concensus = _vote(points,cur_plane,ransac_distance_max);
		int cur_vote_count = cur_concensus.size();
		if(cur_vote_count > max_vote){
			best_plane = cur_plane;
			best_subset = cur_concensus;
			max_vote = best_subset.size();
		}
	}

	#ifdef COUT_INTERMEDIATE
	cout << "Best plane: "<< best_plane.abc << endl;
	cout << "Best subset" << best_subset.size() << endl;
	cout << "Outlier: " <<100*( 1 - best_subset.size() / (points.size() + 0.0) )<< "%" << endl; 
	#endif

	//result
	points_filtered = best_subset;
	return Plane(points_filtered);
}



Eigen::Matrix3d get_rotation_matrix(Vector v, Vector target_v)
{
	//这个方法还不太了解原理
	//参考了一个网络文档
	// https://www.cnblogs.com/tiandsp/p/10687046.html

	Vector u = v.cross(target_v).normalize();
	float_t theta = (float_t)acos((double)v.dot(target_v)) / 2.0;

	u = u.mul(sin(theta));

	float_t q[4] = {(float_t)cos(theta), u.x, u.y, u.z}; //四元数

	Eigen::Matrix3d R;
	R(0, 0) = 2 * q[0] * q[0] - 1 + 2 * q[1] * q[1];
	R(0, 1) = 2 * q[1] * q[2] + 2 * q[0] * q[3];
	R(0, 2) = 2 * q[1] * q[3] - 2 * q[0] * q[2];
	R(1, 0) = 2 * q[1] * q[2] - 2 * q[0] * q[3];
	R(1, 1) = 2 * q[0] * q[0] - 1 + 2 * q[2] * q[2];
	R(1, 2) = 2 * (q[2] * q[3] + q[0] * q[1]);
	R(2, 0) = 2 * (q[1] * q[3] + q[0] * q[2]);
	R(2, 1) = 2 * (q[2] * q[3] - q[0] * q[1]);
	R(2, 2) = 2 * q[0] * q[0] - 1 + 2 * q[3] * q[3];

	return R.transpose();
}

// https://www.cnblogs.com/tiandsp/p/10687046.html
// clear all;
// close all;
// clc;

// v1=[1 2 3];
// v2=[4 5 6];

// %转为单位向量
// nv1 = v1/norm(v1);
// nv2 = v2/norm(v2);

// if norm(nv1+nv2)==0
//     q = [0 0 0 0];
// else
//     u = cross(nv1,nv2);         
//     u = u/norm(u);
    
//     theta = acos(sum(nv1.*nv2))/2;
//     q = [cos(theta) sin(theta)*u];
// end

// %由四元数构造旋转矩阵
// R=[2*q(1).^2-1+2*q(2)^2  2*(q(2)*q(3)+q(1)*q(4)) 2*(q(2)*q(4)-q(1)*q(3));
//     2*(q(2)*q(3)-q(1)*q(4)) 2*q(1)^2-1+2*q(3)^2 2*(q(3)*q(4)+q(1)*q(2));
//     2*(q(2)*q(4)+q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) 2*q(1)^2-1+2*q(4)^2];

// s = nv1*R;

// %显示结果
// v2
// s*norm(v2)