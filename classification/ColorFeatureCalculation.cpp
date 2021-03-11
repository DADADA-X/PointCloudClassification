#include "ColorFeatureCalculation.h"
#include "DataStruct.h"

#include <pcl\point_types.h>
#include <pcl\kdtree\kdtree_flann.h>

using namespace std;

//ColorFeature::ColorFeature()
//{
//}


//ColorFeature::~ColorFeature()
//{
//}

/*RGBI中值滤波（去除斑点）*/
PointProperty * ColorFeature::PointCloudSmooth_Median(int NumOfNeiboringPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PointProperty * cloud_property)
{
	/*创建KD树*/
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdxSearch(NumOfNeiboringPoints);
	vector<float>pointDistance(NumOfNeiboringPoints);
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchPoint;

	for (size_t i = 0; i < cloud->size(); i++)
	{
		int Medianr, Mediang, Medianb, Mediani;
		int *MedianR = new int[NumOfNeiboringPoints];
		int *MedianG = new int[NumOfNeiboringPoints];
		int *MedianB = new int[NumOfNeiboringPoints];
		int *MedianI = new int[NumOfNeiboringPoints];

		searchPoint = cloud->points[i];
		int N = kdtree.nearestKSearch(searchPoint, NumOfNeiboringPoints, pointIdxSearch, pointDistance);
		for (int j = 0; j < N; j++)
		{
			MedianR[j] = cloud_property[pointIdxSearch[j]].color_r;
			MedianG[j] = cloud_property[pointIdxSearch[j]].color_g;
			MedianB[j] = cloud_property[pointIdxSearch[j]].color_b;
			MedianI[j] = cloud_property[pointIdxSearch[j]].intensity;
		}
		sort(MedianR, MedianR + NumOfNeiboringPoints);	Medianr = MedianR[NumOfNeiboringPoints / 2];
		sort(MedianG, MedianG + NumOfNeiboringPoints);	Mediang = MedianG[NumOfNeiboringPoints / 2];
		sort(MedianB, MedianB + NumOfNeiboringPoints);	Medianb = MedianB[NumOfNeiboringPoints / 2];
		sort(MedianI, MedianI + NumOfNeiboringPoints);		Mediani = MedianI[NumOfNeiboringPoints / 2];
		cloud_property[i].color_r = Medianr;
		cloud_property[i].color_g = Mediang;
		cloud_property[i].color_b = Medianb;
		//cloud_property[i].intensity = Mediani;
	}
	return cloud_property;


}
