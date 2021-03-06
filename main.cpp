#include "util.h"

#include <cfloat>
#include <iostream>
#include <vector>
#include <set>
using namespace std;

void init(){
	cout << " # If you are using DEV-C++, you may need refer to the source code to checkout what I had printed." << endl;
}

void print_usage(){
	string to_print_= " 这个程序需要输入一个参数，参数是一个文件的路径\n";
	to_print_ += "文件内容：每个三维点一行，坐标用一个空格隔开,如： 1 2 3 \n";
	to_print_ += "程序通过旋转和平移，把所有数据点用同一个变换，使得它们尽量接近z = 0平面\n";
	to_print_ += "输入文件中的三维点应该尽量在一个平面上或接近一个平面\n";
	cout << to_print_ << endl;
}

int handle_3d_point_file(int argc, char **argv)
{	
	//处理输入，获得三维点文件路径
	if (argc < 2)
	{
		cerr << "Need point file path." << endl;
		print_usage();
		return -1;
	}
	
	//Windows中支持直接将文本文件拖放到命令行程序图标上
	string filepath = string(argv[1]);  


	//从文件读入三维数据点
	vector<Point> points;
	read_point_file(filepath, points);

	vector<Point> points_filtered;
	vector<Point> points_filter_out;
	
	Plane plane;
	cout << "Input 'e' to setting, or Press Enter directly to use default:  ";
	char flag = getchar();
	cout << endl;
	if(flag !='e'){
		//用默认参数
		cout << "Using default max distance: 0.5 " << endl; 
		plane = ransac_iter(points, points_filtered); // ransac过滤局外点，然后存到filtered
	} else{
		float_t max_distance = 1;
		cout << "Input Max distance for RANSAC:  (pixels)"; 
		cin >> max_distance;
		plane = ransac_iter(points, points_filtered,max_distance);
	}


	set<Point> points_filtered_set;
	for(auto& p: points_filtered){
		points_filtered_set.insert(p);
	}
	for(auto& p: points){
		if(!points_filtered_set.count(p))
		points_filter_out.push_back(p);
	}

	
	cout << "Plane equation: ax + by + cz - 1 = 0" << endl;
	cout << "Best plane " << plane.abc << endl;
	cout << "Origin fitted plane " << Plane(points).abc << endl;
	cout << "Outliers:" << 100 - 100 * points_filtered.size() / (0.0 + points.size()) << "%" << endl;

	Plane ori = Plane(points);
	cout << "Error before filter: " << _error_sum(points,ori) << endl;
	cout << "Error after  filter: " << _error_sum(points_filtered,plane) << endl;

	//获得法向量，系数a b c 就是一个法向量
	Vector v = plane.abc.normalize();
	Vector target_v = Vector(0, 0, 1);

	//计算旋转矩阵，旋转到0 0 1，也就是z轴的方向
	Eigen::Matrix3d rotation = get_rotation_matrix(v, target_v);
	//旋转矩阵用法，R乘以原向量，获得目标向量
	// cout << rotation * v.to_eigen_vector() << endl;


	//对所有点施加和法向量一样的旋转，使平面接近z = beta, beta是一个常数 
	vector<Point> points_rotated;
	for (auto &point : points)
	{
		Point rp = Point(rotation * point.to_eigen_vector());
		points_rotated.push_back(rp);
	}


	//找到最小的z,然后减去z
	float_t min_z = FLT_MAX;
	for (auto &p : points_rotated)
		if (p.z < min_z)
			min_z = p.z;
	for (auto &p : points_rotated)
		p.z -= min_z;

	cout << "Writing point file:["<<filepath << ".transed.txt] Point number:" << points_rotated.size() << endl;
	write_point_file(points_rotated, filepath + ".transed.txt");
	cout << "Writing point file:["<<filepath << ".filtered.txt] Point number:" << points_filtered.size() << endl;
	write_point_file(points_filtered, filepath + ".filtered.txt");
	cout << "Writing point file:["<<filepath << ".filteree.txt] Point number:" << points_filter_out.size() << endl;
	write_point_file(points_filter_out, filepath + ".filteree.txt");
	return 0;


	//知识点--------
	//RANSAC算法
	//三维平面方程
	//平面法向量
	//旋转矩阵
	//四元数
	//最小二乘
	//矩阵求逆
	//叉积
	//点积
	//归一化
	//知识点--------
	
}


int main(int argc, char **argv)
{
	init();
	int res = handle_3d_point_file(argc, argv); //将参数交给该功能函数
	return res;
}
