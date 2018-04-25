#pragma once
#include <fstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include"octree.h"
class Model
{
public:
	int max_depth;
	typedef std::vector<std::vector<bool>> Pixel;//一个二维数组
	typedef std::vector<Pixel> Voxel;//三维向量

	Model(int resX = 100, int resY = 100, int resZ = 100,int max_depth=8);
	~Model();

	void Model::saveModel(const std::string & pFileName);//输出不带法向信息的点云，保存为后缀为.xyz的文件
	void saveModelWithNormal(const std::string & pFileName);//输出带法向信息的点云信息，文件格式同上
	void loadMatrix(const char* pFileName);
	void loadImage(const char* pDir, const char* pPrefix, const char* pSuffix);
	void getModel();
	void getSurface();
	Eigen::Vector3f getNormal(int indX, int indY, int indZ);
	void Model::help(octree *oct);
	void Model::new_getModel();
private:
	CoordinateInfo m_corrX;
	CoordinateInfo m_corrY;
	CoordinateInfo m_corrZ;

	int m_neiborSize;

	std::vector<Projection> m_projectionList;

	Voxel m_voxel;
	Voxel m_surface;
};