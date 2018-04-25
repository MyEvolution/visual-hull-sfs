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
	typedef std::vector<std::vector<bool>> Pixel;//һ����ά����
	typedef std::vector<Pixel> Voxel;//��ά����

	Model(int resX = 100, int resY = 100, int resZ = 100,int max_depth=8);
	~Model();

	void Model::saveModel(const std::string & pFileName);//�������������Ϣ�ĵ��ƣ�����Ϊ��׺Ϊ.xyz���ļ�
	void saveModelWithNormal(const std::string & pFileName);//�����������Ϣ�ĵ�����Ϣ���ļ���ʽͬ��
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