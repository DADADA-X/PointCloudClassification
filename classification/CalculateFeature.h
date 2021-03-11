#pragma once
#ifndef FEATURE
#define FEATURE

#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\features\normal_3d.h>
#include <pcl\point_representation.h>
#include <pcl\io\pcd_io.h>
#include <vector>

class CalculateFeature
{
public:
//	CalculateFeature();
//	~CalculateFeature();
	vector<PointProperty> CalculateGeomatricalFeature
	(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, double radius, vector<PointProperty> no_ground_cloud_property);
	void NormalizationFeature(vector<PointProperty> &no_ground_cloud_property);
	void CalculateSegmentFeature(vector<Segment>& FinalResult, vector<PointProperty> no_ground_cloud_property, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud);
	vector<Object> CalculateObjectFeature(vector<Object> MultiObject, vector<Segment>FinalResult, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud);
};


#endif // !FEATURE