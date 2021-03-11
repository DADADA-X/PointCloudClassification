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

/*���ݷ������ָ�;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT)
{
	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
	vector<Segment> SegmentResultBasedOnNormal;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}

	/*��������deque�����ڴ洢һ���ָ���������ӵ�;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;

	/*����KD��;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	//int maxnum = 0;
	//int minnum = 0;
	while (!UnSegment.empty())
	{
		/*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*��UnSegment��ѡȡһ�㣬push��segment��seed�У�����UnSegment��ȥ��*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		//UnSegment.erase(minNum);
		/*�õ�������ӵ�ķ���nx,ny,nz;*/
		double nx_original, ny_original, nz_original;
		nx_original = no_ground_cloud_property[minNum].normal_x;
		ny_original = no_ground_cloud_property[minNum].normal_y;
		nz_original = no_ground_cloud_property[minNum].normal_z;
		while (!seed.empty())
		{
			/*�����ӵ�����긳ֵ��searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//���ӵ㵯��;
			/*Ѱ�����ӵ�������;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			for (int ii = 0; ii < N; ii++)
			{
				/*��������δ���ָ�����������������ж�;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*�������������ӵ㷨�����ļн�;*/
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

					if (CosNormalOriginal > original_cosfaT)//��״������������;
					{
						/*�޸ķ���ţ���UnSegment��ɾ����push��segment��seed��*/
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

/*����������ָ�;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnPrinciple(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, float original_cosfaT, float new_cosfaT)
{
	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
	vector<Segment> SegmentResultBasedOnPrinciple;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	
	/*��������deque�����ڴ洢һ���ָ���������ӵ�;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;

	/*����KD��;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	//int maxnum = 0;
	//int minnum = 0;
	while (!UnSegment.empty())
	{
		/*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*�����ӵ�ѹ��segment��seed��;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*��UnSegment��ȥ����minNum��;*/
		//UnSegment.erase(minNum);
		/*�õ�������ӵ��������;*/
		double px_original, py_original, pz_original;
		px_original = no_ground_cloud_property[minNum].Principal_x;
		py_original = no_ground_cloud_property[minNum].Principal_y;
		pz_original = no_ground_cloud_property[minNum].Principal_z;
		while (!seed.empty())
		{
			/*�����ӵ�����긳ֵ��searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			for (int ii = 0; ii < N; ii++)
			{
				/*��������δ���ָ�����������������ж�;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*�������������ӵ�������ļн�;*/
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

					if (CosPrincipalOriginal > new_cosfaT )//��״������������;
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

/*������ɫ�ָ�;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnColor(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT)
{
	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
	vector<Segment> SegmentResultBasedOnColor;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	/*����KD��;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	/*��������deque�����ڴ洢һ���ָ���������ӵ�;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;
	/*������Ȼ����;*/
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	while (!UnSegment.empty())
	{
		/*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*�����ӵ�ѹ��segment��seed��;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*��UnSegment��ȥ����minNum��;*/
		//UnSegment.erase(minNum);
		/*�õ�������ӵ��RGB;*/
		uint8_t r_original, g_original, b_original;
		r_original = no_ground_cloud_property[minNum].color_r;
		g_original = no_ground_cloud_property[minNum].color_g;
		b_original = no_ground_cloud_property[minNum].color_b;
		while (!seed.empty())
		{
			/*�����ӵ�����긳ֵ��searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//���ӵ㵯��;
			/*Ѱ�����ӵ�������;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			uint8_t r_new, g_new, b_new;
			r_new = no_ground_cloud_property[pointIdxSearch[0]].color_r;
			g_new = no_ground_cloud_property[pointIdxSearch[0]].color_g;
			b_new = no_ground_cloud_property[pointIdxSearch[0]].color_b;
			for (int ii = 0; ii < N; ii++)
			{
				/*��������δ���ָ�����������������ж�;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*�������������ӵ㷨�����ļн�;*/
					uint8_t r1 = no_ground_cloud_property[pointIdxSearch[ii]].color_r;
					uint8_t g1 = no_ground_cloud_property[pointIdxSearch[ii]].color_g;
					uint8_t b1 = no_ground_cloud_property[pointIdxSearch[ii]].color_b;

					int difference_original, difference_new;
					difference_original = abs(r_original - r1) + abs(g_original - g1) + abs(b_original - b1);
					difference_new = abs(r_new - r1) + abs(g_new - g1) + abs(b_new - b1);
					if (difference_original < difference_originalT  &&  difference_new < difference_newT)//���ӵ��ǲ����������ģ�original��ԭʼ���ӵ㣬new�ǵ�ǰ���ӵ㣬�������ж������������ˣ���ǰ����������ɣ��Ż�ԭʼ���ӵ�
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

/*����ǿ�ȷָ�;*/
vector<Segment> CObjectsExtraction::RegionGrowningBasedOnIntensity(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>& no_ground_cloud_property, double radius, int difference_originalT, int difference_newT)
{
	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
	vector<Segment> SegmentResultBasedOnIntensity;
	Segment TempSeg;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < no_ground_cloud->size(); i++)
	{
		UnSegment.insert(i);
		no_ground_cloud_property[i].segment_id = UNSEGMENTATION;
	}
	/*����KD��;*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	std::vector<int>pointIdxSearch;
	std::vector<float>pointDistance;
	/*��������deque�����ڴ洢һ���ָ���������ӵ�;*/
	deque <pcl::PointXYZ>seed;
	int labelOfSeg = 0;
	/*������Ȼ����;*/
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;
	while (!UnSegment.empty())
	{
		/*ѡȡʣ�����ƽ����ϲв���С����С��Ϊ���ӵ�;*/
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		/*�����ӵ�ѹ��segment��seed��;*/
		UnSegment.erase(minNum);
		no_ground_cloud_property[minNum].segment_id = labelOfSeg;
		seed.push_back(no_ground_cloud->points[minNum]);
		TempSeg.segment_id = labelOfSeg;
		TempSeg.point_id.push_back(minNum);
		/*��UnSegment��ȥ����minNum��;*/
		//UnSegment.erase(minNum);
		/*�õ�������ӵ��ǿ��I1;*/
		int I_original;
		I_original = (int)no_ground_cloud_property[minNum].intensity;

		while (!seed.empty())
		{
			/*�����ӵ�����긳ֵ��searchPoint;*/
			searchPoint.x = seed.front().x; searchPoint.y = seed.front().y; searchPoint.z = seed.front().z;
			seed.pop_front();//���ӵ㵯��;
			/*Ѱ�����ӵ�������;*/
			int N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
			int I_new;
			I_new = (int)no_ground_cloud_property[pointIdxSearch[0]].intensity;
			for (int ii = 0; ii < N; ii++)
			{
				/*��������δ���ָ�����������������ж�;*/
				if (no_ground_cloud_property[pointIdxSearch[ii]].segment_id == UNSEGMENTATION)
				{
					/*�������������ӵ㷨�����ļн�;*/
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

/*�Ƴ�ѡ����������������������ͨ���ĵ�;*/
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


/*��������*/
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
	/*����ǰ���ֿ��е����е�push��MaxSegment�У�������Щ�������������ɾ�������ڸ÷��෽ʽ�£��÷���������е�clear*/
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

	vector<Segment> SegmentResultBasedOnNormal;		vector<int> PerPointSegmentID1;	//�洢segment�����ÿ�����segment_id
	vector<Segment> SegmentResultBasedOnPrinciple;		vector<int> PerPointSegmentID2;
	vector<Segment> SegmentResultBasedOnColor;			vector<int> PerPointSegmentID3;
	vector<Segment> SegmentResultBasedOnIntensity;		vector<int> PerPointSegmentID4;

	/*������������������ɫ��ǿ�Ƚ��зָ�*/
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

/*�������ָ��ں�*/
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
	/*final_result_point��ʼ��Ϊfalse*/
	for (int i = 0; i < no_ground_cloud_property.size(); i++)
	{
		no_ground_cloud_property[i].final_result_point = false;
	}

	/*���з������ĵ���final_result_point��ֵΪtrue*/
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


/*����segment�����˹�ϵ��segment��������segmentid��*/
vector<Segment>  CObjectsExtraction::BuildTopologicalRelationsBetweenSegments(vector<Segment>FinalResult, vector<PointProperty> no_ground_cloud_property, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, float Radius)
{
	/*����KD��;*/
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
				/*����segment��ÿһ����*/
				//cout << "pointid____________________________________________________________" << j << endl << endl;
				searchPoint = no_ground_cloud->points[FinalResult[i].point_id[j]];
				int N = kdtree.radiusSearch(searchPoint, Radius, pointIdxSearch, pointDistance);
				for (int n = 0; n < N; n++)
				{
					//cout << n << endl;
					/*����������ÿ����*/
					//����㲻ͬsegment���ѷ��࣬ ����non_tree�� ��finalresult��;
					//�ж��Ƿ���FinalResult[[i]���segment��neiboringsegment�У�������ڣ���push��segment������������
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

/*segment�ϲ�object*/
vector<Object> CObjectsExtraction::SegmentsMergeToObject(vector<Segment>& FinalResult)
{
	vector<Object> MultiObject;
	/*����UnSegment�Լ�UnSegment�ĵ��������洢δ�ָ�ĵ��;*/
	Object TempObj;
	set<int, less<int>>UnSegment;
	set<int, less<int>>::iterator iterUnseg;
	for (int i = 0; i < FinalResult.size(); i++)
	{
		UnSegment.insert(i);
		FinalResult[i].ObjectID = UNSEGMENTATION;
	}
	/*��������deque,���ڴ洢һ���ָ����������segment;*/
	deque <Segment>seed;
	int labelOfObj = 0;
	while (!UnSegment.empty())
	{
		iterUnseg = UnSegment.begin();
		int minNum = *iterUnseg;
		if (FinalResult[minNum].none_tree)  /*�����segment��non_tree�򲻲���object���γ�;*/
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
				seed.pop_front();//���ӵ㵯��;
				/*Ѱ������segment�������;*/
				for (int i = 0; i < TempSeg.NeiboringSegmentID.size(); i++)
				{
					/*���������Ӧ��segmentδ���ֽ�object��ָ���ǰobject�����Ѹ�segment��objectid����Ϊ��ǰobject��Ȼ������segmentpush����ǰobject��*/
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
	/*�ж�ÿ��object�Ƿ��и�;*/
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
				if (MultiObject[i].average_standard_deviation < 15) /*ƽ����ϲ��С��Ϊ�˹�����;*/
				{
					if (/*MultiObject[i].noamal_ratio > 0.5 && */MultiObject[i].NumOfPoint > 800) /*����󲿵����Է������ָ��Ϊ������;*/
						MultiObject[i].ObjectSort = 1;/*������;*/
					if (MultiObject[i].principle_ratio > 0.5)  /*����󲿵�����������ָ��Ϊ��״��ж�Ϊ·��;*/
						MultiObject[i].ObjectSort = 4;  /*·��;*/
					if (MultiObject[i].ObjectSort == 0)
						MultiObject[i].ObjectSort = 5;  /*�����ǽ����ͷ�·�Ƶ���������;*/
				}
				else/*ƽ����ϲ�ϴ�Ϊ��Ȼ����;*/
				{
					if (/*MultiObject[i].HavePole || */MultiObject[i].HeightDifferenceBeteewnGeometricalCenterAndBarycenter > 0 && MultiObject[i].height > 5 && MultiObject[i].PointDensityRatio < 0.5)
						MultiObject[i].ObjectSort = 2;	/*����*/
					else
						MultiObject[i].ObjectSort = 6;  /*�������������Ȼ����;*/
				}
			}
			else
				MultiObject[i].ObjectSort = 7; /*����С��1000��Ϊ��Ƭ;*/
		}
	}
	return MultiObject;
}