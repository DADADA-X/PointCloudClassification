#include "DataStruct.h"
#include "Data_io.h"
#include "ColorFeatureCalculation.h"
#include "GroundFilter.h"
#include "CalculateFeature.h"
#include "ObjectsExtraction.h"
#include "viewer.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl\visualization\cloud_viewer.h>

int main()
{
	/*�����㷨����*/
	int NumOfNeiboringPoints = 25;
	float cylinder_radius = 0.5;
	float max_height_difference = 0.3;
	float radiusInCalculateFeature = 0.3;
	float radius_in_region_growning = 0.3;
	float original_cosfaT = 0.966;  /*�������ָ���ֵ;*/
	float new_cosfaT = 0.922;	/*������ָ���ֵ*/
	int color_difference_originalT = 30;     /*��ɫ�ָ���ֵ;*/
	int color_difference_newT = 10;
	int intensity_difference_originalT = 5;   /*ǿ�ȷָ���ֵ;*/
	int intensity_difference_newT = 3;
	int NumOfPointsInMinSegmentT = 50;
	double radius_in_topological_relation = 0.5;


	/*���������*/
	CLidarDataIo DataInOut;
	ColorFeature m_corlor;
	CGroundFilter GroundFiler;
	CalculateFeature Calculator;
	CObjectsExtraction Extraction;
	CLiDARViewer Viewer;


	/*����ȫ�ֱ���*/
	PointProperty * cloud_property;//�洢ÿ���������,����;
	vector<PointProperty> ground_cloud_property;
	vector<PointProperty> no_ground_cloud_property;
	vector<Segment>FinalResult;
	vector<Object>multi_object;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//���ڴ洢��������Ļ��������;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	/*�ļ���ȡ*/
	double center_x, center_y, center_z;//��������;
	double max_x, max_y, max_z, min_x, min_y, min_z;//�����������»�������ֵ����Сֵ;
	int point_number;
	cloud_property = DataInOut.read_las(&point_number, &center_x, &center_y, &center_z, &max_x, &max_y, &max_z, &min_x, &min_y, &min_z, cloud);

	/*����ƽ��*/
	cloud_property = m_corlor.PointCloudSmooth_Median(NumOfNeiboringPoints, cloud, cloud_property);

	/*�������ȡ*/
	GroundFiler.ExtractGroundPoint(cloud, cloud_property, ground_cloud, &ground_cloud_property, no_ground_cloud, &no_ground_cloud_property, cylinder_radius, max_height_difference);

	//pcl::visualization::CloudViewer viewer("viewer");
	//viewer.showCloud(no_ground_cloud);
	//system("pause");

	/*�ǵ�����������㣬����һ����0-1*/
	no_ground_cloud_property = Calculator.CalculateGeomatricalFeature(no_ground_cloud, radiusInCalculateFeature, no_ground_cloud_property);
	Calculator.NormalizationFeature(no_ground_cloud_property);
	//cout << "no_ground_cloud->size()=" << no_ground_cloud->size() << endl;

	/*�������ںϷָ�*/
	FinalResult = Extraction.GetTheFinalResult2(no_ground_cloud, no_ground_cloud_property, NumOfPointsInMinSegmentT, radius_in_region_growning,
		original_cosfaT, new_cosfaT, color_difference_originalT, color_difference_newT,
		intensity_difference_originalT, intensity_difference_newT);
	Extraction.TheLastResult(FinalResult, no_ground_cloud_property);		//��ÿ���㸳���Ƿ�����segmentid������ֵ

	/*����ÿ��segment������*/
	Calculator.NormalizationFeature(no_ground_cloud_property);	//�ǵ����������һ����0-1
	Calculator.CalculateSegmentFeature(FinalResult, no_ground_cloud_property, no_ground_cloud);

	//for (int i = 0; i < no_ground_cloud_property.size(); i++)
	//	if(no_ground_cloud_property[i].final_result_point)
	//		cout << no_ground_cloud_property[i].segment_id << endl;

	//system("pause");


	/*�ϲ�object*/
	FinalResult = Extraction.BuildTopologicalRelationsBetweenSegments(FinalResult,no_ground_cloud_property,no_ground_cloud,radius_in_topological_relation);		//�������˹�ϵ
	multi_object = Extraction.SegmentsMergeToObject(FinalResult);
	//cout << multi_object.size();
	Viewer.DisplayMultiObject(no_ground_cloud, FinalResult, multi_object);

	/*����ÿ��object������*/
	multi_object = Calculator.CalculateObjectFeature(multi_object, FinalResult, no_ground_cloud);
	multi_object = Extraction.JudgeObjectSort2(multi_object, FinalResult);
	Viewer.DisplayMultiSortObject(no_ground_cloud, FinalResult, multi_object);
	Viewer.DisplayTreeObject(no_ground_cloud, FinalResult, multi_object);


	/*view*/
	//Viewer.DisplayPointCloudXYZRGB(cloud, cloud_property);


	system("pause");
	return 0;
}