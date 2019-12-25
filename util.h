#ifndef UTIL_H
#define UTIL_H

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>

using std::cerr;
using std::cout;
using std::endl;
using std::istream;
using std::ostream;
using std::string;
using std::vector;

#include <Eigen/Dense> //you may need to unzip the eigen package 

inline void wait()
{
	cout << "Press Enter to exit." << endl;
	getchar();
}

//退出前输出字符串s
inline void wait_exit(string s)
{
	cerr << "Error happened!" << endl;
	cerr << s << endl;
	wait();
	exit(1);
}

#define COUT_INTERMEDIATE //用这个来限制一些函数的内部输出，主要是早期用于调试的临时cout
#undef COUT_INTERMEDIATE

typedef float float_t; //后面可以把float_t改为double

class Point
{
public:
	float_t x, y, z;

	Point()
	{
		x = y = z = (float_t)0;
	}

	Point(float_t xx, float_t yy, float_t zz) : x(xx), y(yy), z(zz)
	{
	}

	Point(Eigen::Vector3d v) : x(v[0]), y(v[1]), z(v[2])
	{
	}

	Point normalize()
	{
		float_t d = sqrtf(x * x + y * y + z * z);
		return Point(x / d, y / d, z / d);
	}

	Point cross(const Point &another)
	{
		//返回两个向量的叉积
		float_t resx = y * another.z - z * another.y;
		float_t resy = z * another.x - x * another.z;
		float_t resz = x * another.y - y * another.x;
		return Point(resx, resy, resz);
	}

	float_t dot(const Point &another)
	{
		//点乘
		return x * another.x + y * another.y + z * another.z;
	}

	Point mul(float_t ratio)
	{
		//乘以一个标量
		return Point(x * ratio, y * ratio, z * ratio);
	}

	Eigen::Vector3d to_eigen_vector()
	{
		//返回Eigen三维向量类型
		Eigen::Vector3d res;
		res << x, y, z;
		return res;
	}
};

typedef Point Vector; // 三维向量，以0为起点，其终点是一个点

inline ostream &operator<<(ostream &out, const Point &p)
{
	//由于文件格式问题。输出就不加前缀和后缀了
	//一开始考虑输出（1，2，3）这种
	const string pre = "";
	const string sep = " ";
	const string post = "";
	out << pre << p.x << sep << p.y << sep << p.z << post;
	return out;
}

inline istream &operator>>(istream &in, Point &p)
{
	in >> p.x >> p.y >> p.z;
	return in;
}

class Plane
{
public:
	// 平面方程是ax + by + z - 1 = 0
	// Point abc 的x y z 分别表示a b c
	Point abc;

public:
	Plane()
	{
	}

	Plane(float_t a, float_t b, float_t c) : abc(a, b, c)
	{
	}

	Plane(Point &p1, Point &p2, Point &p3) // 三个点构造一个平面, 这个考虑删掉
	{
		Eigen::Matrix3d A;
		Eigen::Vector3d b;
		b << 1, 1, 1;
		A << p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z;

		Eigen::Vector3d x = A.inverse() * b;

		abc.x = x[0];
		abc.y = x[1];
		abc.z = x[2];
	}

	Plane(vector<Point> points) //构造函数，用最小二乘对输入点集拟合一个平面
	{
		int n = points.size();
		if (n < 3)
			wait_exit("Not enough points to build plane");

		Eigen::MatrixXd A(n, 3);

		for (int i = 0; i < n; i++)
		{
			A(i, 0) = points[i].x;
			A(i, 1) = points[i].y;
			A(i, 2) = points[i].z;
		}

		Eigen::MatrixXd b = Eigen::MatrixXd::Ones(n, 1);

		Eigen::Vector3d x = (A.transpose() * A).inverse() * A.transpose() * b;

		abc.x = x[0];
		abc.y = x[1];
		abc.z = x[2];
	}

	float_t error(Point &p) //将点代入平面方程，计算误差
	{
		return 1 - (abc.x * p.x + abc.y * p.y + abc.z * p.z);
	}

	float_t distance(Point &p) // 计算点到平面的距离
	{
		float_t res = fabs(error(p)); /*考虑点到平面的距离的计算公式*/
		res /= sqrtf(abc.x * abc.x + abc.y * abc.y + abc.z * abc.z);
		return res;
	}
};

//读路径为path的文件，将其中的三维点放到 points里
//文件格式是一行一个点，坐标用一个空格隔开
//如行：
//1.221 2.333 -5.23
int read_point_file(string path, vector<Point> &points);

//将一个点的集合points写到路径为path的文本文件中
//格式参考read_point_file的注释
int write_point_file(vector<Point> &points, string path);

//返回最好的平面；参数points_filtered是inliers的集合
//iter_time是ransac迭代次数
//max_distance是一个距离阈值，当点到平面的距离小于它时，表示该点与平面一致。
//max_distance单位是像素
const int default_ransac_iter_times = 150;
const float_t default_ransac_max_distance = (float_t)0.5;
Plane ransac_iter(vector<Point> &points, vector<Point> &points_filtered, float_t max_distance = default_ransac_max_distance, int iter_times = default_ransac_iter_times);

//返回一个Eigen矩阵 3x3, 这个矩阵是v到 target v的旋转矩阵
// R * v = target_v
Eigen::Matrix3d get_rotation_matrix(Vector v, Vector target_v);

#endif
