#pragma warning(disable:4819)
#pragma warning(disable:4244)
#pragma warning(disable:4267)
//禁止警告语句
#include <time.h>
#include <iostream>
#include<string>
#include"octree.h"
#include"related_class.h"
#include"Model.h"


Model::Model(int resX, int resY, int resZ, int _max_depth)
	: m_corrX(resX, -5, 5)
	, m_corrY(resY, -10, 10)
	, m_corrZ(resZ, 15, 30)
	, max_depth(_max_depth)
{
	if (resX > 100)
		m_neiborSize = resX / 100;
	else
		m_neiborSize = 1;
	m_voxel = Voxel(m_corrX.m_resolution, Pixel(m_corrY.m_resolution, std::vector<bool>(m_corrZ.m_resolution, true)));//三维向量的构建
	m_surface = m_voxel;//表面点
	/*初试构造函数会确定三维重建模型的分辨率，并且对所有三维点都为true，也就是构建了一个长方体*/
}

Model::~Model()
{
}

void Model::saveModel(const std::string & pFileName)
{
	std::ofstream fout(pFileName);
	/*主要是确定了具体的坐标*/

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ])
				{
					//std::cout << m_corrX.m_resolution << " " << m_corrY.m_resolution << " " << m_corrZ.m_resolution << std::endl;
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << std::endl;
					
				}
}

void Model::saveModelWithNormal(const std::string & pFileName)
{
	std::ofstream fout(pFileName);

	double midX = m_corrX.index2coor(m_corrX.m_resolution / 2);
	double midY = m_corrY.index2coor(m_corrY.m_resolution / 2);
	double midZ = m_corrZ.index2coor(m_corrZ.m_resolution / 2);

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				if (m_surface[indexX][indexY][indexZ])
				{
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					fout << coorX << ' ' << coorY << ' ' << coorZ << ' ';

					Eigen::Vector3f nor = getNormal(indexX, indexY, indexZ);//获取了法向信息
					fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;//输出法向信息，似乎
				}
}

void Model::loadMatrix(const char* pFileName)//主要读取的是相机参数与位置等信息
{
	std::ifstream fin(pFileName);

	int num;
	Eigen::Matrix<float, 3, 3> matInt;
	Eigen::Matrix<float, 3, 4> matExt;
	Projection projection;
	while (fin >> num)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fin >> matInt(i, j);//依然是对（）运算符的重载，也就是该向量的[i][j]

		double temp;
		fin >> temp;
		fin >> temp;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				fin >> matExt(i, j);

		projection.m_projMat = matInt * matExt;
		m_projectionList.push_back(projection);
	}
}

void Model::loadImage(const char* pDir, const char* pPrefix, const char* pSuffix)//读取2维图片
{
	int fileCount = m_projectionList.size();
	std::string fileName(pDir);
	fileName += '/';
	fileName += pPrefix;//获取到文件名
	for (int i = 0; i < fileCount; i++)
	{
		//std::cout << fileName + std::to_string(i) + pSuffix << std::endl;
		m_projectionList[i].m_image = cv::imread(fileName + std::to_string(i) + pSuffix, CV_8UC1);
	}
}
/*getModel是本次作业需要改进的地方
通过建立octree来加速*/
void Model::getModel()
{
	int projectionCount = m_projectionList.size();//投影的数量

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
				for (int i = 0; i < projectionCount; i++)
				{
					double coorX = m_corrX.index2coor(indexX);
					double coorY = m_corrY.index2coor(indexY);
					double coorZ = m_corrZ.index2coor(indexZ);
					m_voxel[indexX][indexY][indexZ] = m_voxel[indexX][indexY][indexZ] && m_projectionList[i].checkRange(coorX, coorY, coorZ);
				}
}

void Model::getSurface()
{
	// 邻域：上、下、左、右、前、后。
	int dx[6] = { -1, 0, 0, 0, 0, 1 };
	int dy[6] = { 0, 1, -1, 0, 0, 0 };
	int dz[6] = { 0, 0, 0, 1, -1, 0 };

	// lambda表达式，用于判断某个点是否在Voxel的范围内
	auto outOfRange = [&](int indexX, int indexY, int indexZ){
		return indexX < 0 || indexY < 0 || indexZ < 0
			|| indexX >= m_corrX.m_resolution
			|| indexY >= m_corrY.m_resolution
			|| indexZ >= m_corrZ.m_resolution;
	};

	for (int indexX = 0; indexX < m_corrX.m_resolution; indexX++)
		for (int indexY = 0; indexY < m_corrY.m_resolution; indexY++)
			for (int indexZ = 0; indexZ < m_corrZ.m_resolution; indexZ++)
			{
				if (!m_voxel[indexX][indexY][indexZ])//根据前面的原理，如果某个点是外部节点，则一定不会是表面节点
				{
					m_surface[indexX][indexY][indexZ] = false;
					continue;
				}
				bool ans = false;
				for (int i = 0; i < 6; i++)//检查是否是2维图片的边界点
				{
					ans = ans || outOfRange(indexX + dx[i], indexY + dy[i], indexZ + dz[i])//在图片边界，那么一定是边界点
						|| !m_voxel[indexX + dx[i]][indexY + dy[i]][indexZ + dz[i]];//边上的点是false，满足任何一个，则是轮廓的边界点
				}

				m_surface[indexX][indexY][indexZ] = ans;
			}
}
void Model::new_getModel()
{
	
	octree *oct = new octree(0,0,0,0,0,max_depth);
	oct->createNew(m_projectionList);
	help(oct);
}
void Model::help(octree *oct)
{
	
	if (oct == nullptr) { std::cout << "nullptr" << std::endl; return; }
	int  len = 1 << (oct->max_depth - oct -> now);
	//std::cout <<  oct->now << " " << (oct->max_depth - oct->now) << " "<<len << std::endl;
	if (oct->type == OUT)
	{
		for (int x = oct->x; x != oct->x + len; ++x)
			for (int y = oct->y; y != oct->y + len; ++y)
				for (int z = oct->z; z != oct->z + len; ++z)
				{
					m_voxel[x][y][z] = false;

				}
	}
	else if (oct->type == IN)
	{
		for (int x = oct->x; x != oct->x + len; ++x)
			for (int y = oct->y; y != oct->y + len; ++y)
				for (int z = oct->z; z != oct->z + len; ++z)
				{
					m_voxel[x][y][z] = true; 
				}
	}
	else
	{
			for (int i = 0; i != 8; ++i)
				help(oct->cubes[i]);
	}

}
Eigen::Vector3f Model::getNormal(int indX, int indY, int indZ)
{
	auto outOfRange = [&](int indexX, int indexY, int indexZ){
		return indexX < 0 || indexY < 0 || indexZ < 0
			|| indexX >= m_corrX.m_resolution
			|| indexY >= m_corrY.m_resolution
			|| indexZ >= m_corrZ.m_resolution;
	};

	std::vector<Eigen::Vector3f> neiborList;
	std::vector<Eigen::Vector3f> innerList;

	for (int dX = -m_neiborSize; dX <= m_neiborSize; dX++)
		for (int dY = -m_neiborSize; dY <= m_neiborSize; dY++)
			for (int dZ = -m_neiborSize; dZ <= m_neiborSize; dZ++)
			{
				if (!dX && !dY && !dZ)
					continue;
				int neiborX = indX + dX;
				int neiborY = indY + dY;
				int neiborZ = indZ + dZ;
				if (!outOfRange(neiborX, neiborY, neiborZ))
				{
					float coorX = m_corrX.index2coor(neiborX);	
					float coorY = m_corrY.index2coor(neiborY);
					float coorZ = m_corrZ.index2coor(neiborZ);
					if (m_surface[neiborX][neiborY][neiborZ])
						neiborList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
					else if (m_voxel[neiborX][neiborY][neiborZ])
						innerList.push_back(Eigen::Vector3f(coorX, coorY, coorZ));
				}
			}

	Eigen::Vector3f point(m_corrX.index2coor(indX), m_corrY.index2coor(indY), m_corrZ.index2coor(indZ));

	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++)
		matA.col(i) = neiborList[i] - point;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	int indexEigen = 0;
	if (abs(eigenValues[1]) < abs(eigenValues[indexEigen]))
		indexEigen = 1;
	if (abs(eigenValues[2]) < abs(eigenValues[indexEigen]))
		indexEigen = 2;
	Eigen::Vector3f normalVector = eigenSolver.eigenvectors().col(indexEigen);
	
	Eigen::Vector3f innerCenter = Eigen::Vector3f::Zero();
	for (auto const& vec : innerList)
		innerCenter += vec;
	innerCenter /= innerList.size();

	if (normalVector.dot(point - innerCenter) < 0)
		normalVector *= -1;
	return normalVector;
}
int checkType(int x, int y, int z, int len, int resolution, std::vector<Projection> & projectionList)
{
	bool surface = false, out = false;
	int now;
	for (int i = 0; i != projectionList.size(); ++i)
	{
		now = projectionList[i].checkType(x, y, z, len, resolution);
		if (now == OUT)
			out = true;
		if (now == ON)
			surface = true;

	}
	if (out) return OUT;
	if (surface) return ON;
	return IN;
}
void compare_print(int resolution)
{
	int n = log(resolution)/log(2);
	std::cout << "---------------------- resolution:" << resolution << "*"<<resolution<<"*"<<resolution<<"--------------------\n";
	std::cout << "-----------------The old method----------------\n";
	clock_t old_t = clock(), t;

	// 分别设置xyz方向的Voxel分辨率
	Model model(resolution, resolution, resolution);

	// 读取相机的内外参数
	model.loadMatrix("../../calibParamsI.txt");
	// 读取投影图片
	model.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");
	// 得到Voxel模型


	std::cout << "start to use the old method to create the model...\n";

	t = clock();
	model.getModel();
	std::cout << "get model done\n";
	std::cout << "the time consumption: " << (float(clock() - t) / CLOCKS_PER_SEC) << "seconds\n";
	// 获得Voxel模型的表面

	model.getSurface();

	std::cout << "get surface done \n";

	// 将模型导出为xyz格式
	model.saveModel(std::string("../../WithoutNormal")+std::to_string(resolution) + ".xyz");
	std::cout << "save without normal done\n";

	model.saveModelWithNormal(std::string("../../WithNormal") + std::to_string(resolution) + ".xyz");
	std::cout << "save with normal done\n";
	std::string sys = "PoissonRecon.x64 --in ../../WithNormal" + std::to_string(resolution) + ".xyz --out ../../mesh"+std::to_string(resolution)+".ply";
	system(sys.c_str());
	std::cout << "save mesh.ply done\n";


	std::cout << "time: " << (float(clock() - old_t) / CLOCKS_PER_SEC) << "seconds\n";
	/*--------------------------------------------------------------------------*/
	std::cout << "-----------------The new method----------------\n";
	old_t = clock();
	Model model_new(resolution, resolution, resolution, n);

	// 读取相机的内外参数
	model_new.loadMatrix("../../calibParamsI.txt");

	// 读取投影图片
	model_new.loadImage("../../wd_segmented", "WD2_", "_00020_segmented.png");

	std::cout << "start to use the new method to create the model and get the surface...\n";
	t = clock();
	model_new.new_getModel();

	std::cout << "get model  done\n";

	std::cout << "the time consumption: " << (float(clock() - t) / CLOCKS_PER_SEC) << "seconds\n";

	model_new.getSurface();
	std::cout << "get surface done\n";
	// 将模型导出为xyz格式
	model_new.saveModel(std::string("../../WithoutNormal_new") + std::to_string(resolution) + ".xyz");
	std::cout << "save without normal done\n";

	model_new.saveModelWithNormal(std::string("../../WithNormal_new") + std::to_string(resolution) + ".xyz");
	std::cout << "save with normal done\n";
	 sys = "PoissonRecon.x64 --in ../../WithNormal_new" + std::to_string(resolution) + ".xyz --out ../../mesh_new" + std::to_string(resolution) + ".ply";
	system(sys.c_str());
	std::cout << "save mesh_new.ply done\n";

	std::cout << "time: " << (float(clock() - old_t) / CLOCKS_PER_SEC) << "seconds\n";
}
int main(int argc, char** argv)
{
	
	int resolution;
	while (std::cin >> resolution)
		compare_print(resolution);
	return 0;
}
