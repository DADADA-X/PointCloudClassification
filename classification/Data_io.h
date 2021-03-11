#pragma once

#ifndef DATA_IO_
#define DATA_IO

#include "DataStruct.h"

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

#include <vector>
#include <list>

class CLidarDataIo
{
public:
	//	CLidarDataIo();
	//	~CLidarDataIo();
	PointProperty * read_las(int * point_number, double *center_x, double *center_y, double *center_z,
		double *max_x, double *max_y, double *max_z, double *min_x, double *min_y, double *min_z,		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
	HEADERBLOCK MyHeader;
	VARISBLELENGTHRECORDS MyVLR;
	POINTDATARECORDFORMAT0 myPointData0;
	POINTDATARECORDFORMAT1 myPointData1;
	void CopyPoint(int point_number, PointCoordinate * points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double center_x, double center_y, double center_z);
};





#endif // !DATA_IO_
