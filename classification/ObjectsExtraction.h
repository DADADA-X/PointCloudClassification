#pragma once
#ifndef OBJECTEXTRACTION
#define OBJECTEXTRACTION

#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\features\normal_3d.h>
#include <vector>

class CObjectsExtraction
{
public:
//	CObjectsExtraction();
//	~CObjectsExtraction();

	vector<Segment> RegionGrowningBasedOnNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT);
	vector<Segment> RegionGrowningBasedOnPrinciple(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT);
	vector<Segment> RegionGrowningBasedOnColor(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT);
	vector<Segment> RegionGrowningBasedOnIntensity(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT);

	//int FindMaxSegment(vector<Segment> SegmentResultBasedOnNormal, vector<Segment> SegmentResultBasedOnPrinciple, vector<Segment> SegmentResultBasedOnColor, vector<Segment> SegmentResultBasedOnIntensity,
	//	pcl::PointIndices::Ptr inliers_plane);
	//vector<PointProperty> CalculateremainingPointProperty(vector<PointProperty> no_ground_cloud_property, pcl::PointIndices::Ptr inliers_plane);

	//vector<Segment> GetTheFinalResult(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty> no_ground_cloud_property, int NumOfPointsInMinSegmentT,
	//	float radius, float original_cosfaT, float new_cosfaT, int color_difference_originalT, int color_difference_newT, int intensity_difference_originalT,
	//	int intensity_difference_newT, pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<PointProperty>& ReslutCloudProperty);
	//vector<int> CalculateNumberOfLabels(vector<int> a);
	//vector<Segment> FindEdgePoints(double radius, pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<PointProperty>&ReslutCloudProperty, vector<Segment>FinalResultSegment);
	///*计算每个点对于可能属于的区域的得分;*/
	//vector<FitnessScore> CalculateFitnessScore(vector<Segment> FinalResult, vector<PointProperty> ReslutCloudProperty, pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud);
	//vector<PointProperty> RefineEdgePoint(vector<FitnessScore> fitness_score, vector<Segment> FinalResult, vector<PointProperty> ReslutCloudProperty);
	vector<Segment> BuildTopologicalRelationsBetweenSegments(vector<Segment>FinalResult, vector<PointProperty> ReslutCloudProperty, pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, float Radius);
	vector<Object> SegmentsMergeToObject(vector<Segment>& FinalResult);

	//vector<Object> JudgeObjectSort(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Object> MultiObject, vector<Segment>FinalResult);
	vector<Object> JudgeObjectSort2(vector<Object> MultiObject, vector<Segment>FinalResult);



	void RemovePoints(int p_id, vector<Segment> & segment_result, vector<int> PerPointSegmentID);
	Segment FindMaxSegment2(vector<Segment>& SegmentResultBasedOnNormal, vector<Segment>& SegmentResultBasedOnPrinciple, vector<Segment>& SegmentResultBasedOnColor,
		vector<Segment>& SegmentResultBasedOnIntensity, vector<PointProperty> & no_ground_cloud_property, int SegmentId, vector<int> PerPointSegmentID1, vector<int> PerPointSegmentID2, vector<int> PerPointSegmentID3, vector<int> PerPointSegmentID4);
	vector<Segment> GetTheFinalResult2(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property,
		int NumOfPointsInMinSegmentT, float radius, float original_cosfaT, float new_cosfaT, int color_difference_originalT,
		int color_difference_newT, int intensity_difference_originalT, int intensity_difference_newT);

	/*把no_ground_cloud比FinalResult多的点找出来，该点的final_result_point 值为false;*/
	void TheLastResult(vector<Segment>FinalResult, vector<PointProperty>& no_ground_cloud_property);

};


#endif // !OBJECTEXTRACTION

