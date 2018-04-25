#pragma once
#include <fstream>
#include <vector>
#include<set>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#define OUT -1
#define ON 0
#define IN 1
// 用于index和实际坐标之间的转换
struct CoordinateInfo
{
	int m_resolution;
	double m_min;
	double m_max;

	double index2coor(int index)
	{
		return m_min + index * (m_max - m_min) / m_resolution;
	}

	CoordinateInfo(int resolution = 10, double min = 0.0, double max = 100.0)
		: m_resolution(resolution)
		, m_min(min)
		, m_max(max)
	{
	}
};

// 用于判断投影是否在visual hull内部
struct Projection
{
	Eigen::Matrix<float, 3, 4> m_projMat;//声明一个3行4列的矩阵,存放的是相机内参与相机外部参数的乘积
	cv::Mat m_image;
	const uint m_threshold = 125;

	bool outOfRange(int x, int max)
	{
		return x < 0 || x >= max;
	}

	bool checkRange(double x, double y, double z)
	{
		Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(x, y, z, 1);//vector3f与vector4f分别为三维向量与四维向量，数据类型float
		int indX = vec3[1] / vec3[2];
		int indY = vec3[0] / vec3[2];

		if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
			return false;
		return m_image.at<uchar>((uint)indX, (uint)indY) > m_threshold;//函数模板
	}
	int checkType(int x, int y, int z, int len, int resolution)
	{
		
		std::vector<std::pair<uint, uint>> help;
		bool noPoint = true, allPoint = true, now;
		if (len != 1)
			help = getCubePoint(x, y, z, len, resolution);
		else
		{ 
			help = getCubeEdge(x, y, z, len, resolution,allPoint);
			now = m_image.at<uchar>(help.begin()->first, help.begin()->second) > m_threshold;
			if (now) return IN;
			else return OUT;
		}

		for (auto i = help.begin(); i != help.end(); ++i)
		{
			now = m_image.at<uchar>(i->first, i->second) > m_threshold;
			if (now)
				noPoint = false;
			if (!now) allPoint = false;
			if (!noPoint && !allPoint)
				return ON;
		}
		help = getCubeEdge(x, y, z, len, resolution,noPoint);
		for (auto i = help.begin(); i != help.end(); ++i)
		{
			now = m_image.at<uchar>(i->first, i->second) > m_threshold;
			if (now)
				noPoint = false;
			if (!now) allPoint = false;
			if (!noPoint && !allPoint)
				return ON;
		}		
			help = getCubeSurface(x, y, z, len, resolution,noPoint);
			for (auto i = help.begin(); i != help.end(); ++i)
			{
				now = m_image.at<uchar>(i->first, i->second) > m_threshold;
				if (now)
					noPoint = false;
				if (!now) allPoint = false;
				if (!noPoint && !allPoint)
					return ON;
			}
			if (allPoint) return IN;
			else return OUT;
	}
	std::vector<std::pair<uint, uint>> getCubePoint(int x, int y, int z, int len, int resolution)
	{
		std::vector<std::pair<uint, uint>> result;
		CoordinateInfo m_x(resolution, -5, 5), m_y(resolution, -10, 10), m_z(resolution, 15, 30);
		std::vector<std::vector<int>> weNeed = { {x,y,z} ,{x + len - 1,y,z},{x,y + len - 1,z},{x,y,z + len - 1},{x + len - 1,y + len - 1,z}, {x+len-1,y,z+len-1},{x,y+len-1,z+len-1},{x+len-1,y+len-1,z+len-1}
		};
		
		for (int i = 0; i != 8; ++i)
		{
			Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(weNeed[i][0]), m_y.index2coor(weNeed[i][1]), m_z.index2coor(weNeed[i][2]), 1);
			int indX = vec3[1] / vec3[2];
			int indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				result.push_back(std::make_pair(0, 0));
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
			
		}
		return result;
	}
	std::vector<std::pair<uint, uint>> getCubeEdge(int x, int y, int z, int len, int resolution,bool noPoint)//实际上，我们只要记录边的投影即可，这里约定传入第一个顶点，并且传入边长
	{
		std::vector<std::pair<uint, uint>> result;
		CoordinateInfo m_x(resolution, -5, 5), m_y(resolution, -10, 10), m_z(resolution, 15, 30);
		bool outrange = false;
		Eigen::Vector3f vec3;
		int indX, indY;
		for (int i = 0; i != len; ++i)
		{
		
			vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y), m_z.index2coor(z), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		if (len == 1) return result;
		for (int i = 1; i != len-1; ++i)
		{
			
			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x), m_y.index2coor(y + i), m_z.index2coor(z), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x), m_y.index2coor(y), m_z.index2coor(z + i), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1), m_y.index2coor(y + i), m_z.index2coor(z), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1), m_y.index2coor(y), m_z.index2coor(z + i), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y), m_z.index2coor(z + len - 1), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x), m_y.index2coor(y + i), m_z.index2coor(z + len - 1), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y + len - 1), m_z.index2coor(z), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x), m_y.index2coor(y + len - 1), m_z.index2coor(z + i), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 0; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1 - i), m_y.index2coor(y + len - 1), m_z.index2coor(z + len - 1), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1), m_y.index2coor(y + len - 1 - i), m_z.index2coor(z + len - 1), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len-1; ++i)
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1), m_y.index2coor(y + len - 1), m_z.index2coor(z + len - 1 - i), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		if(!noPoint && outrange)
			result.push_back(std::make_pair(0, 0));
		return result;
	}
	std::vector<std::pair<uint, uint>> getCubeSurface(int x, int y, int z, int len, int resolution,bool noPoint)//实际上，我们只要记录边的投影即可，这里约定传入第一个顶点，并且传入边长
	{
		std::vector<std::pair<uint, uint>> result;
		CoordinateInfo m_x(resolution, -5, 5), m_y(resolution, -10, 10), m_z(resolution, 15, 30);
		bool outrange = false;
		Eigen::Vector3f vec3;
		int indX, indY;
		for (int i = 1; i != len-1; ++i)
			for(int j = 1;j!=len-1;++j)
			
		{

			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y+j), m_z.index2coor(z), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
			 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y + j), m_z.index2coor(z+len-1), 1);
			 indX = vec3[1] / vec3[2];
			 indY = vec3[0] / vec3[2];
			if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
				outrange = true;
			else
				result.push_back(std::make_pair((uint)indX, (uint)indY));
		}
		for (int i = 1; i != len - 1; ++i)
			for (int k = 1; k != len - 1; ++k)

			{

				 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y ), m_z.index2coor(z+k), 1);
				 indX = vec3[1] / vec3[2];
				 indY = vec3[0] / vec3[2];
				if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
					outrange = true;
				else
					result.push_back(std::make_pair((uint)indX, (uint)indY));
				vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + i), m_y.index2coor(y + len - 1), m_z.index2coor(z + k), 1);
				indX = vec3[1] / vec3[2];
				indY = vec3[0] / vec3[2];
				if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
					outrange = true;
				else
					result.push_back(std::make_pair((uint)indX, (uint)indY));
			}
		for (int j = 1; j != len - 1; ++j)
			for (int k = 1; k != len - 1; ++k)

			{

				 vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x ), m_y.index2coor(y + j), m_z.index2coor(z+k), 1);
				 indX = vec3[1] / vec3[2];
				 indY = vec3[0] / vec3[2];
				if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
					outrange = true;
				else
					result.push_back(std::make_pair((uint)indX, (uint)indY));
				vec3 = m_projMat * Eigen::Vector4f(m_x.index2coor(x + len - 1), m_y.index2coor(y + j), m_z.index2coor(z +k), 1);
				indX = vec3[1] / vec3[2];
				indY = vec3[0] / vec3[2];
				if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
					outrange = true;
				else
					result.push_back(std::make_pair((uint)indX, (uint)indY));
			}
		if (!noPoint && outrange)
			result.push_back(std::make_pair(0, 0));
		return result;
	}
};




int checkType(int x, int y, int z, int len, int resolution, std::vector<Projection> & projectionList);
