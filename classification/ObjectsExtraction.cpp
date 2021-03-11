#include "ObjectsExtraction.h"
#include "DataStruct.h"
#include "viewer.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>

#include <ctime>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
#include <algorithm>

#define UNSEGMENTATION -1

using namespace std;

//CObjectsExtraction::CObjectsExtraction()
//{
//}


//CObjectsExtraction::~CObjectsExtraction()
//{
//}

/*根据法向量分割;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT)
{
	/*建立UnSegment以及UnSegment的迭代器，存储未分割的点号;*/
	vector<Segment> SegmentResultBasedOnNormal;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}

	/*建立队列deque，用于存储一个分割区域的种子点;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;

	/*建立KD树;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	//int maxnum = 0;
	//int minnum = 0;
	while (!UnSegment.empty())
	{
		/*选取剩余点中平面拟合残差最小的最小点为种子点;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*从UnSegment中选取一点，push到segment和seed中，并从UnSegment中去除*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		//UnSegment.erase(minNum);
		/*得到最初种子点的法线nx,ny,nz;*/
		double nx_original, ny_original, nz_original;
		nx_original = no_ground_cloud_property[minNum].normal_x;
		ny_original = no_ground_cloud_property[minNum].normal_y;
		nz_original = no_ground_cloud_property[minNum].normal_z;
		while (!seed.empty())
		{
			/*将种子点的坐标赋值给searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//种子点弹出;
			/*寻找种子点的邻域点;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			for (int ii = 0; ii < N; ii++)
			{
				/*如果邻域点未被分割则进行生长条件的判断;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*计算邻域点和种子点法向量的夹角;*/
					float nx1 = no_ground_cloud_property[pointIdxSearch[ii]].normal_x;
					float ny1 = no_ground_cloud_property[pointIdxSearch[ii]].normal_y;
					float nz1 = no_ground_cloud_property[pointIdxSearch[ii]].normal_z;

					float n1 = nx_original * nx1 + ny_original * ny1 + nz_original * nz1;
					float nn1 = sqrt(nx_original*nx_original + ny_original * ny_original + nz_original * nz_original);
					float nn2 = sqrt(nx1*nx1 + ny1 * ny1 + nz1 * nz1);
					float CosNormalOriginal = abs(n1 / (nn1*nn2));

					//if (CosNormalOriginal > original_cosfaT)
					//	maxnum++;
					//else
					//	minnum++;

					if (CosNormalOriginal > original_cosfaT)//面状点生长的条件;
					{
						/*修改分类号，从UnSegment中删除，push进segment和seed中*/
						no_ground_cloud_property[pointIdxSearch[ii]].segment_id = labelOfSeg;
						UnSegment.erase(pointIdxSearch[ii]);
						TempSeg.point_id.push_back(pointIdxSearch[ii]);
						seed.push_back(no_ground_cloud->points[pointIdxSearch[ii]]);
					}
				}
			}
		}
		SegmentResultBasedOnNormal.push_back(TempSeg);
		labelOfSeg++;
		TempSeg.point_id.clear();
		seed.clear();
	}
	//cout << float(maxnum) / (maxnum + minnum) << endl;
	return SegmentResultBasedOnNormal;
}

/*根据主方向分割;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnPrinciple(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT)
{
	/*建立UnSegment以及UnSegment的迭代器，存储未分割的点号;*/
	vector<Segment> SegmentResultBasedOnPrinciple;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	
	/*建立队列deque，用于存储一个分割区域的种子点;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;

	/*建立KD树;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	//int maxnum = 0;
	//int minnum = 0;
	while (!UnSegment.empty())
	{
		/*选取剩余点中平面拟合残差最小的最小点为种子点;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*将种子点压入segment和seed中;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*从UnSegment中去除掉minNum点;*/
		//UnSegment.erase(minNum);
		/*得到最初种子点的主方向;*/
		double px_original, py_original, pz_original;
		px_original = no_ground_cloud_property[minNum].Principal_x;
		py_original = no_ground_cloud_property[minNum].Principal_y;
		pz_original = no_ground_cloud_property[minNum].Principal_z;
		while (!seed.empty())
		{
			/*将种子点的坐标赋值给searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			for (int ii = 0; ii < N; ii++)
			{
				/*如果邻域点未被分割则进行生长条件的判断;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*计算邻域点和种子点主方向的夹角;*/
					float px1 = no_ground_cloud_property[pointIdxSearch[ii]].Principal_x;
					float py1 = no_ground_cloud_property[pointIdxSearch[ii]].Principal_y;
					float pz1 = no_ground_cloud_property[pointIdxSearch[ii]].Principal_z;

					float p1 = px_original * px1 + py_original * py1 + pz_original * pz1;
					float pp1 = sqrt(px_original*px_original + py_original * py_original + pz_original * pz_original);
					float pp2 = sqrt(px1*px1 + py1 * py1 + pz1 * pz1);
					float CosPrincipalOriginal = abs(p1 / (pp1*pp2));

					//if (CosPrincipalOriginal > 0.931)
					//	maxnum++;
					//else
					//	minnum++;

					if (CosPrincipalOriginal > new_cosfaT )//面状点生长的条件;
					{

						no_ground_cloud_property[pointIdxSearch[ii]].segment_id = labelOfSeg;
						TempSeg.point_id.push_back(pointIdxSearch[ii]);
						UnSegment.erase(pointIdxSearch[ii]);
						seed.push_back(no_ground_cloud->points[pointIdxSearch[ii]]);
					}
				}
			}
		}
		SegmentResultBasedOnPrinciple.push_back(TempSeg);
		labelOfSeg++;
		TempSeg.point_id.clear();
		seed.clear();
	}
	//cout << float(maxnum) / (maxnum + minnum) << endl;
	//system("pause");
	return SegmentResultBasedOnPrinciple;
}

/*根据颜色分割;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnColor(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT)
{
	/*建立UnSegment以及UnSegment的迭代器，存储未分割的点号;*/
	vector<Segment> SegmentResultBasedOnColor;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	/*建立KD树;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	/*建立队列deque，用于存储一个分割区域的种子点;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;
	/*先排序，然后建树;*/
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	while (!UnSegment.empty())
	{
		/*选取剩余点中平面拟合残差最小的最小点为种子点;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*将种子点压入segment和seed中;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*从UnSegment中去除掉minNum点;*/
		//UnSegment.erase(minNum);
		/*得到最初种子点的RGB;*/
		uint8_t r_original, g_original, b_original;
		r_original = no_ground_cloud_property[minNum].color_r;
		g_original = no_ground_cloud_property[minNum].color_g;
		b_original = no_ground_cloud_property[minNum].color_b;
		while (!seed.empty())
		{
			/*将种子点的坐标赋值给searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//种子点弹出;
			/*寻找种子点的邻域点;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			uint8_t r_new, g_new, b_new;
			r_new = no_ground_cloud_property[pointIdxSearch[0]].color_r;
			g_new = no_ground_cloud_property[pointIdxSearch[0]].color_g;
			b_new = no_ground_cloud_property[pointIdxSearch[0]].color_b;
			for (int ii = 0; ii < N; ii++)
			{
				/*如果邻域点未被分割则进行生长条件的判断;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*计算邻域点和种子点法向量的夹角;*/
					uint8_t r1 = no_ground_cloud_property[pointIdxSearch[ii]].color_r;
					uint8_t g1 = no_ground_cloud_property[pointIdxSearch[ii]].color_g;
					uint8_t b1 = no_ground_cloud_property[pointIdxSearch[ii]].color_b;

					int difference_original, difference_new;
					difference_original = abs(r_original - r1) + abs(g_original - g1) + abs(b_original - b1);
					difference_new = abs(r_new - r1) + abs(g_new - g1) + abs(b_new - b1);
					if (difference_original < difference_originalT  &&  difference_new < difference_newT)//种子点是不断往外扩的，original是原始种子点，new是当前种子点，搜索到判断条件不符合了，当前区域增长完成，才换原始种子点
					{
						no_ground_cloud_property[pointIdxSearch[ii]].segment_id = labelOfSeg;
						TempSeg.point_id.push_back(pointIdxSearch[ii]);
						UnSegment.erase(pointIdxSearch[ii]);
						seed.push_back(no_ground_cloud->points[pointIdxSearch[ii]]);
					}
				}
			}
		}
		SegmentResultBasedOnColor.push_back(TempSeg);
		labelOfSeg++;
		TempSeg.point_id.clear();
		seed.clear();
	}
	return SegmentResultBasedOnColor;
}

/*根据强度分割;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnIntensity(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT)
{
	/*建立UnSegment以及UnSegment的迭代器，存储未分割的点号;*/
	vector<Segment> SegmentResultBasedOnIntensity;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	/*建立KD树;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	/*建立队列deque，用于存储一个分割区域的种子点;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;
	/*先排序，然后建树;*/
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	while (!UnSegment.empty())
	{
		/*选取剩余点中平面拟合残差最小的最小点为种子点;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*将种子点压入segment和seed中;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*从UnSegment中去除掉minNum点;*/
		//UnSegment.erase(minNum);
		/*得到最初种子点的强度I1;*/
		int I_original;
		I_original = (int)no_ground_cloud_property[minNum].intensity;

		while (!seed.empty())
		{
			/*将种子点的坐标赋值给searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//种子点弹出;
			/*寻找种子点的邻域点;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			int I_new;
			I_new = (int)no_ground_cloud_property[pointIdxSearch[0]].intensity;
			for (int ii = 0; ii < N; ii++)
			{
				/*如果邻域点未被分割则进行生长条件的判断;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*计算邻域点和种子点法向量的夹角;*/
					int i1;
					i1 = (int)no_ground_cloud_property[pointIdxSearch[ii]].intensity;

					int difference_original, difference_new;
					difference_original = abs(I_original - i1);
					difference_new = abs(I_new - i1);

					//	cout<<I_new<<"  "<<I_original<<"  "<<difference_new<<"  "<<difference_original<<"  "<<difference_newT<<"  "<<difference_originalT<<endl;
					if ((difference_original < difference_originalT) && (difference_new < difference_newT))
					{
						no_ground_cloud_property[pointIdxSearch[ii]].segment_id = labelOfSeg;
						TempSeg.point_id.push_back(pointIdxSearch[ii]);
						UnSegment.erase(pointIdxSearch[ii]);
						seed.push_back(no_ground_cloud->points[pointIdxSearch[ii]]);
					}
				}
			}
		}
		SegmentResultBasedOnIntensity.push_back(TempSeg);
		labelOfSeg++;
		TempSeg.point_id.clear();
		seed.clear();
	}
	return SegmentResultBasedOnIntensity;
}

/*移除选出来的最大块中在其他三个通道的点;*/
void CObjectsExtraction::RemovePoints(int p_id, vector<Segment> & segment_result, vector<int> PerPointSegmentID)
{
	int seg_id = PerPointSegmentID[p_id];
	for (vector<int>::iterator it = segment_result[seg_id].point_id.begin(); it != segment_result[seg_id].point_id.end(); )
	{
		int point_id = *it;
		if (point_id == p_id)
		{
			it = segment_result[seg_id].point_id.erase(it);
		}
		else
		{
			++it;

		}
	}
}


/*最大块最优*/
Segment CObjectsExtraction::FindMaxSegment2(vector<Segment> & SegmentResultBasedOnNormal, vector<Segment> & SegmentResultBasedOnPrinciple, vector<Segment>& SegmentResultBasedOnColor,
	vector<Segment>& SegmentResultBasedOnIntensity, vector<PointProperty> & no_ground_cloud_property, int SegmentId, vector<int> PerPointSegmentID1, vector<int> PerPointSegmentID2, vector<int> PerPointSegmentID3, vector<int> PerPointSegmentID4)
{
	Segment MaxSegment;
	int num_of_point_in_maxSegment = 0;
	int segment_id;
	int NormalPrincipleColorIntensity;
	for (int i = 0; i < SegmentResultBasedOnNormal.size(); i++)
	{
		if (SegmentResultBasedOnNormal[i].point_id.size() > num_of_point_in_maxSegment)
		{
			num_of_point_in_maxSegment = SegmentResultBasedOnNormal[i].point_id.size();
			segment_id = i;
			NormalPrincipleColorIntensity = 1;
		}
	}
	for (int i = 0; i < SegmentResultBasedOnPrinciple.size(); i++)
	{
		if (SegmentResultBasedOnPrinciple[i].point_id.size() > num_of_point_in_maxSegment)
		{
			num_of_point_in_maxSegment = SegmentResultBasedOnPrinciple[i].point_id.size();
			segment_id = i;
			NormalPrincipleColorIntensity = 2;
		}
	}
	for (int i = 0; i < SegmentResultBasedOnColor.size(); i++)
	{
		if (SegmentResultBasedOnColor[i].point_id.size() > num_of_point_in_maxSegment)
		{
			num_of_point_in_maxSegment = SegmentResultBasedOnColor[i].point_id.size();
			segment_id = i;
			NormalPrincipleColorIntensity = 3;
		}
	}
	for (int i = 0; i < SegmentResultBasedOnIntensity.size(); i++)
	{
		if (SegmentResultBasedOnIntensity[i].point_id.size() > num_of_point_in_maxSegment)
		{
			num_of_point_in_maxSegment = SegmentResultBasedOnIntensity[i].point_id.size();
			segment_id = i;
			NormalPrincipleColorIntensity = 4;
		}
	}

	MaxSegment.NormalPrincipleColorIntensity = NormalPrincipleColorIntensity;
	/*将当前最大分块中的所有点push到MaxSegment中，并将这些点从其他分类中删除，将在该分类方式下，该分类号中所有点clear*/
	if (NormalPrincipleColorIntensity == 1) 
	{
		for (int i = 0; i < SegmentResultBasedOnNormal[segment_id].point_id.size(); i++)
		{
			int p_id = SegmentResultBasedOnNormal[segment_id].point_id[i];
			MaxSegment.point_id.push_back(p_id);
			no_ground_cloud_property[p_id].segment_id = SegmentId;

			RemovePoints(p_id, SegmentResultBasedOnPrinciple, PerPointSegmentID2);
			RemovePoints(p_id, SegmentResultBasedOnColor, PerPointSegmentID3);
			RemovePoints(p_id, SegmentResultBasedOnIntensity, PerPointSegmentID4);

		}
		SegmentResultBasedOnNormal[segment_id].point_id.clear();

	}
	if (NormalPrincipleColorIntensity == 2)
	{
		for (int i = 0; i < SegmentResultBasedOnPrinciple[segment_id].point_id.size(); i++)
		{
			int p_id = SegmentResultBasedOnPrinciple[segment_id].point_id[i];
			MaxSegment.point_id.push_back(p_id);
			no_ground_cloud_property[p_id].segment_id = SegmentId;


			RemovePoints(p_id, SegmentResultBasedOnNormal, PerPointSegmentID1);
			RemovePoints(p_id, SegmentResultBasedOnColor, PerPointSegmentID3);
			RemovePoints(p_id, SegmentResultBasedOnIntensity, PerPointSegmentID4);
		}
		SegmentResultBasedOnPrinciple[segment_id].point_id.clear();
	}

	if (NormalPrincipleColorIntensity == 3)
	{
		for (int i = 0; i < SegmentResultBasedOnColor[segment_id].point_id.size(); i++)
		{
			int p_id = SegmentResultBasedOnColor[segment_id].point_id[i];
			MaxSegment.point_id.push_back(p_id);
			no_ground_cloud_property[p_id].segment_id = SegmentId;


			RemovePoints(p_id, SegmentResultBasedOnNormal, PerPointSegmentID1);
			RemovePoints(p_id, SegmentResultBasedOnPrinciple, PerPointSegmentID2);
			RemovePoints(p_id, SegmentResultBasedOnIntensity, PerPointSegmentID4);
		}
		SegmentResultBasedOnColor[segment_id].point_id.clear();
	}

	if (NormalPrincipleColorIntensity == 4)
	{
		for (int i = 0; i < SegmentResultBasedOnIntensity[segment_id].point_id.size(); i++)
		{
			int p_id = SegmentResultBasedOnIntensity[segment_id].point_id[i];
			MaxSegment.point_id.push_back(p_id);
			no_ground_cloud_property[p_id].segment_id = SegmentId;

			RemovePoints(p_id, SegmentResultBasedOnNormal, PerPointSegmentID1);
			RemovePoints(p_id, SegmentResultBasedOnPrinciple, PerPointSegmentID2);
			RemovePoints(p_id, SegmentResultBasedOnColor, PerPointSegmentID3);
		}
		SegmentResultBasedOnIntensity[segment_id].point_id.clear();
	}
	return MaxSegment;

}


vector<Segment> CObjectsExtraction::GetTheFinalResult2(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property,
	int NumOfPointsInMinSegmentT, float radius, float original_cosfaT, float new_cosfaT, int color_difference_originalT,
	int color_difference_newT, int intensity_difference_originalT, int intensity_difference_newT)
{
	vector<Segment>FinalResult;
	Segment MaxSegment;
	int NumOfPointsInMinSegment = NumOfPointsInMinSegmentT + 1;
	int original_seg_id = 0;

	vector<Segment> SegmentResultBasedOnNormal;		vector<int> PerPointSegmentID1;	//存储segment结果和每个点的segment_id
	vector<Segment> SegmentResultBasedOnPrinciple;		vector<int> PerPointSegmentID2;
	vector<Segment> SegmentResultBasedOnColor;			vector<int> PerPointSegmentID3;
	vector<Segment> SegmentResultBasedOnIntensity;		vector<int> PerPointSegmentID4;

	/*按法向量，主方向，颜色，强度进行分割*/
SegmentResultBasedOnNormal = RegionGrowningBasedOnNormal(no_ground_cloud, no_ground_cloud_property, radius, original_cosfaT, new_cosfaT);
for (int i = 0; i < no_ground_cloud_property.size(); i++)
{
	PerPointSegmentID1.push_back(no_ground_cloud_property[i].segment_id);
}

SegmentResultBasedOnPrinciple = RegionGrowningBasedOnPrinciple(no_ground_cloud, no_ground_cloud_property, radius, original_cosfaT, new_cosfaT);
for (int i = 0; i < no_ground_cloud_property.size(); i++)
{
	PerPointSegmentID2.push_back(no_ground_cloud_property[i].segment_id);
}
SegmentResultBasedOnColor = RegionGrowningBasedOnColor(no_ground_cloud, no_ground_cloud_property, radius, color_difference_originalT, color_difference_newT);
for (int i = 0; i < no_ground_cloud_property.size(); i++)
{
	PerPointSegmentID3.push_back(no_ground_cloud_property[i].segment_id);
}

SegmentResultBasedOnIntensity = RegionGrowningBasedOnIntensity(no_ground_cloud, no_ground_cloud_property, radius, intensity_difference_originalT, intensity_difference_newT);
for (int i = 0; i < no_ground_cloud_property.size(); i++)
{
	PerPointSegmentID4.push_back(no_ground_cloud_property[i].segment_id);
}

CLiDARViewer viewer;
viewer.DisplaySegmentationResultOnMultiWindows(no_ground_cloud, SegmentResultBasedOnNormal, SegmentResultBasedOnPrinciple, SegmentResultBasedOnColor, SegmentResultBasedOnIntensity);
//cout << "normal: " << SegmentResultBasedOnNormal.size() << "\tprincipal: " << SegmentResultBasedOnPrinciple.size() << "\tcolor: " << SegmentResultBasedOnColor.size() << "\tintensity: " << SegmentResultBasedOnIntensity.size() << endl;

/*多特征分割融合*/
int num_of_iteration = 0;
int SegmentId = 0;
while (NumOfPointsInMinSegment > NumOfPointsInMinSegmentT)
{
	MaxSegment = FindMaxSegment2(SegmentResultBasedOnNormal, SegmentResultBasedOnPrinciple, SegmentResultBasedOnColor, SegmentResultBasedOnIntensity, no_ground_cloud_property, SegmentId, PerPointSegmentID1, PerPointSegmentID2, PerPointSegmentID3, PerPointSegmentID4);
	MaxSegment.segment_id = SegmentId;
	FinalResult.push_back(MaxSegment);
	NumOfPointsInMinSegment = MaxSegment.point_id.size();

	SegmentId++;
	MaxSegment.point_id.clear();

	num_of_iteration++;
}
viewer.DisPlaySegmentationResult(no_ground_cloud, FinalResult);
viewer.DisPlaySegmentationSortResult(no_ground_cloud, FinalResult);
//cout << "FinalResult.size = " << FinalResult.size() << endl;

return FinalResult;
}

void CObjectsExtraction::TheLastResult(vector<Segment>FinalResult, vector<PointProperty>& no_ground_cloud_property)
{
	vector<PointProperty> ReslutCloudProperty;
	/*final_result_point初始化为false*/
	for (int i = 0; i < no_ground_cloud_property.size(); i++)
	{
		no_ground_cloud_property[i].final_result_point = false;
	}

	/*给有分类结果的点云final_result_point赋值为true*/
	for (int i = 0; i < FinalResult.size(); i++)
	{
		for (int j = 0; j < FinalResult[i].point_id.size(); j++)
		{
			int p_id = FinalResult[i].point_id[j];
			no_ground_cloud_property[p_id].final_result_point = true;
			no_ground_cloud_property[p_id].segment_id = FinalResult[i].segment_id;
		}
	}
}


/*建立segment的拓扑关系（segment的邻域点的segmentid）*/
vector<Segment>  CObjectsExtraction::BuildTopologicalRelationsBetweenSegments(vector<Segment>FinalResult, vector<PointProperty> no_ground_cloud_property, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, float Radius)
{
	/*创建KD树;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	vector<int>pointIdxSearch;
	vector<float>pointDistance;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	for (int i = 0; i < FinalResult.size(); i++)
	{
		if (FinalResult[i].none_tree == false)
		{
			//cout << "segmentid--------------------------------------------------" << i << endl << endl;
			for (int j = 0; j < FinalResult[i].point_id.size(); j++)
			{
				/*遍历segment的每一个点*/
				//cout << "pointid____________________________________________________________" << j << endl << endl;
				searchPoint = no_ground_cloud->points[FinalResult[i].point_id[j]];
				int N = kdtree.radiusSearch(searchPoint, Radius, pointIdxSearch, pointDistance);
				for (int n = 0; n < N; n++)
				{
					//cout << n << endl;
					/*遍历邻域内每个点*/
					//邻域点不同segment，已分类， 不是non_tree， 在finalresult中;
					//判断是否在FinalResult[[i]这个segment的neiboringsegment中，如果不在，则push进segment的邻域数组中
					if (no_ground_cloud_property[pointIdxSearch[n]].final_result_point)
					{
						if ((no_ground_cloud_property[pointIdxSearch[n]].segment_id != i)
							&& (no_ground_cloud_property[pointIdxSearch[n]].segment_id != UNSEGMENTATION)
							&& (FinalResult[no_ground_cloud_property[pointIdxSearch[n]].segment_id].none_tree == false))
						{
							bool IsUnique = true;
							for (int m = 0; m < FinalResult[i].NeiboringSegmentID.size(); m++)
							{
								if (FinalResult[i].NeiboringSegmentID[m] == no_ground_cloud_property[pointIdxSearch[n]].segment_id)
								{
									IsUnique = false;
									break;
								}
							}
							if (IsUnique == true)
								FinalResult[i].NeiboringSegmentID.push_back(no_ground_cloud_property[pointIdxSearch[n]].segment_id);
						}
					}
				}
			}
		}
	}
	return FinalResult;
}

/*segment合并object*/
vector<Object> CObjectsExtraction::SegmentsMergeToObject(vector<Segment>& FinalResult)
{
	vector<Object> MultiObject;
	/*建立UnSegment以及UnSegment的迭代器，存储未分割的点号;*/
	Object TempObj;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < FinalResult.size(); i++)
	{
		UnSegment.insert(i);
		FinalResult[i].ObjectID = UNSEGMENTATION;
	}
	/*建立队列deque,用于存储一个分割区域的种子segment;*/
	deque <Segment>seed;
	int labelOfObj = 0;
	while (!UnSegment.empty())
	{
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		if (FinalResult[minNum].none_tree)  /*如果该segment是non_tree则不参与object的形成;*/
		{
			UnSegment.erase(minNum);
		}
		else
		{
			FinalResult[minNum].ObjectID = labelOfObj;
			seed.push_back(FinalResult[minNum]);
			TempObj.ObjectID = labelOfObj;
			TempObj.SegmentID.push_back(minNum);
			UnSegment.erase(minNum);
			while (!seed.empty())
			{
				Segment TempSeg;
				TempSeg = seed.front();
				seed.pop_front();//种子点弹出;
				/*寻找种子segment的邻域点;*/
				for (int i = 0; i < TempSeg.NeiboringSegmentID.size(); i++)
				{
					/*如果邻域点对应的segment未被分进object则分给当前object，即把该segment的objectid设置为当前object，然后把这个segmentpush到当前object中*/
					if (FinalResult[TempSeg.NeiboringSegmentID[i]].ObjectID == UNSEGMENTATION)
					{
						FinalResult[TempSeg.NeiboringSegmentID[i]].ObjectID = labelOfObj;
						TempObj.SegmentID.push_back(TempSeg.NeiboringSegmentID[i]);

						UnSegment.erase(TempSeg.NeiboringSegmentID[i]);
						seed.push_back(FinalResult[TempSeg.NeiboringSegmentID[i]]);
					}
				}
			}
			MultiObject.push_back(TempObj);
			labelOfObj++;
			TempObj.SegmentID.clear();
			seed.clear();
		}
	}
	return MultiObject;
}

vector<Object> CObjectsExtraction::JudgeObjectSort2(vector<Object> MultiObject, vector<Segment>FinalResult)
{
	/*判断每个object是否有杆;*/
	for (int i = 0; i < MultiObject.size(); i++)
	{
		MultiObject[i].ObjectSort = 0;
		MultiObject[i].HavePole = false; 
		for (int j = 0; j < MultiObject[i].SegmentID.size(); j++)
		{
			int seg_id = MultiObject[i].SegmentID[j];
			if (FinalResult[seg_id].is_pole)
			{
				MultiObject[i].HavePole = true;
				break;
			}
		}
	}

	for (int i = 0; i < MultiObject.size(); i++)
	{
		if (MultiObject[i].ObjectSort == 0)
		{
			if (MultiObject[i].NumOfPoint > 100)
			{
				if (MultiObject[i].average_standard_deviation < 15) /*平面拟合差较小，为人工建筑;*/
				{
					if (/*MultiObject[i].noamal_ratio > 0.5 && */MultiObject[i].NumOfPoint > 800) /*如果大部点来自法向量分割，则为建筑物;*/
						MultiObject[i].ObjectSort = 1;/*建筑物;*/
					if (MultiObject[i].principle_ratio > 0.5)  /*如果大部点来自主方向分割，则为杆状物，判断为路灯;*/
						MultiObject[i].ObjectSort = 4;  /*路灯;*/
					if (MultiObject[i].ObjectSort == 0)
						MultiObject[i].ObjectSort = 5;  /*其他非建筑和非路灯的人造物体;*/
				}
				else/*平面拟合差较大，为自然物体;*/
				{
					if (/*MultiObject[i].HavePole || */MultiObject[i].HeightDifferenceBeteewnGeometricalCenterAndBarycenter > 0 && MultiObject[i].height > 5 && MultiObject[i].PointDensityRatio < 0.5)
						MultiObject[i].ObjectSort = 2;	/*行树*/
					else
						MultiObject[i].ObjectSort = 6;  /*不规则的其他自然物体;*/
				}
			}
			else
				MultiObject[i].ObjectSort = 7; /*点数小于1000的为碎片;*/
		}
	}
	return MultiObject;
}