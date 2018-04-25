#pragma once
#include"related_class.h"
struct octree
{

	
	std::vector <octree*> cubes;
	int max_depth=8;//最大递归深度
	int x = 0,y=0,z=0;
	int now = 0;
	int type=0;
	octree(  int _x = 0, int _y = 0,int _z=0,int _now = 0, int _type = 0, int _max_depth = 8)
	{
		type = _type;
		x = _x; y = _y; z = _z;
		max_depth = _max_depth;
		cubes = std::vector<octree*>(8,nullptr);
		now = _now;
	}
	void setType(int i)
	{
		type = i;
	}
	void createNew(std::vector<Projection> &projectionList)
	{
		if (now > max_depth -1)
			return;
		int  len = 1 << (max_depth - now -1);
		cubes[0] = new octree(x, y, z,now + 1,0,max_depth);
		cubes[1] = new octree(x + len, y, z, now +1,0, max_depth); 
		cubes[2] = new octree(x, y + len, z, now +1, 0, max_depth);
		cubes[3] = new octree(x + len, y + len, z, now+1, 0, max_depth);
		cubes[4] = new octree(x, y, z + len, now +1, 0, max_depth);
		cubes[5] = new octree(x + len, y, z + len, now+1, 0, max_depth);
		cubes[6] = new octree(x, y + len, z + len, now +1, 0, max_depth);
		cubes[7] = new octree(x + len, y + len, z + len, now+1, 0, max_depth);
		for (int i = 0; i != 8; ++i)
		{

			cubes[i]->setType(checkType(cubes[i]->x, cubes[i]->y, cubes[i]->z, len, 1 << max_depth, projectionList));

		}
		for (int i = 0; i != 8; ++i)
		{
			if (cubes[i]->type == ON)
				cubes[i]->createNew(projectionList);
		}
	}
};