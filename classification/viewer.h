#pragma once
#ifndef VIEWER
#define VIEVER

#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <vector>
#include <list>

using namespace std;

class CLiDARViewer
{
public:
//	CLiDARViewer();
//	~CLiDARViewer();
	void DisplayPointCloudXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudold, PointProperty * point_property);
	void DisplaySegmentationResultOnMultiWindows(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud,
		vector<Segment> SegmentResultBasedOnNormal, vector<Segment> SegmentResultBasedOnPrinciple,
		vector<Segment> SegmentResultBasedOnColor, vector<Segment> SegmentResultBasedOnIntensity);
	void DisPlaySegmentationResult(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<Segment> SegmentResult);
	void DisPlaySegmentationSortResult(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<Segment> SegmentResult);
	void DisplayMultiObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject);
	void DisplayMultiSortObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject);
	void DisplayTreeObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject);
};



#endif // !VIEWER

