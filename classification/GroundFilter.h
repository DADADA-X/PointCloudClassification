#pragma once
#ifndef GROUND_FILTER_
#define GROUND_FILTER_

#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\features\normal_3d.h>

#include <vector>

class CGroundFilter
{
public:
//	CGroundFilter();
//	~CGroundFilter();
	void ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PointProperty * cloud_property,
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud, vector<PointProperty> *ground_cloud_property,
		pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>* no_ground_cloud_property,
		float cylinder_radius, float max_height_difference);

private:
	void TransformCloud3D_Cloud2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d);
	void InitializePointProperty(PointProperty * cloud_property, int point_number);
	void CalculateCylinderNeiborhoodMaxz_Minz(float *max_z, float *min_z, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d);
	void JudgeGroundPoints(int point_id, float max_height_difference, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d,
		float min_z, float max_z, PointProperty * cloud_property);
};


#endif // !GROUND_FILTER_


