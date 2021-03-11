#include "GroundFilter.h"
#include "DataStruct.h"

#include <pcl\point_types.h>
#include <pcl\kdtree\kdtree_flann.h>

#include <vector>

using namespace std;

//CGroundFilter::CGroundFilter()
//{
//}


//CGroundFilter::~CGroundFilter()
//{
//}

/*����ά����ת����2ά����;*/
void CGroundFilter::TransformCloud3D_Cloud2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d)
{
	pcl::PointXYZ temp_point;
	for (size_t i = 0; i < cloud_3d->points.size(); ++i)
	{
		temp_point.x = cloud_3d->points[i].x;
		temp_point.y = cloud_3d->points[i].y;
		temp_point.z = 0.0;
		cloud_2d->push_back(temp_point);
	}
}

/*��ʼ�����Ƶ��������Ϊ��δ���ࡱ;*/
void CGroundFilter::InitializePointProperty(PointProperty * cloud_property, int point_number)
{
	for (int i = 0; i < point_number; i++)
	{
		cloud_property[i].sort_id = 0;
	}
}

/*����Բ�������ڵ�������С��Zֵ;*/
void CGroundFilter::CalculateCylinderNeiborhoodMaxz_Minz(float *max_z, float *min_z, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d)
{
	*min_z = 10000000.0;
	*max_z = -1000000.0;
	/*����Բ���������ڵ���С��Zֵ������Zֵ;*/
	for (int j = 0; j < pointIdxSearch.size(); j++)
	{
		if (cloud_3d->points[pointIdxSearch[j]].z < *min_z)
		{
			*min_z = cloud_3d->points[pointIdxSearch[j]].z;
		}
		if (cloud_3d->points[pointIdxSearch[j]].z > *max_z)
		{
			*max_z = cloud_3d->points[pointIdxSearch[j]].z;
		}
	}
}

/*�ж��Ƿ�Ϊ�����;*/
void CGroundFilter::JudgeGroundPoints(int point_id, float max_height_difference, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d,
	float min_z, float max_z, PointProperty * cloud_property)
{
	/*�жϵ����;*/
	if (cloud_3d->points[point_id].z - min_z < max_height_difference)
	{
		cloud_property[point_id].sort_id = 1;//�����;
		/*�ȵ�ǰ����㻹�͵ĵ�Ҳ��Ϊ�����;*/
		for (int j = 0; j < pointIdxSearch.size(); j++)
		{
			if ((cloud_3d->points[pointIdxSearch[j]].z < cloud_3d->points[point_id].z)
				&& (cloud_property[pointIdxSearch[j]].sort_id == 0))
			{
				cloud_property[pointIdxSearch[j]].sort_id = 1;
			}
		}
	}
	else
	{
		cloud_property[point_id].sort_id = 2;//�ǵ����;
		/*�ȵ�ǰ�ǵ���㻹�ߵĵ�Ҳ��Ϊ�ǵ����;*/
		for (int j = 0; j < pointIdxSearch.size(); j++)
		{
			if ((cloud_3d->points[pointIdxSearch[j]].z > cloud_3d->points[point_id].z)
				&& (cloud_property[pointIdxSearch[j]].sort_id == 0))
			{
				cloud_property[pointIdxSearch[j]].sort_id = 2;
			}
		}
	}
}



void CGroundFilter::ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PointProperty * cloud_property,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud, vector<PointProperty> *ground_cloud_property,
	pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<PointProperty>* no_ground_cloud_property,
	float cylinder_radius, float max_height_difference)
{
	/*����ά����ת���ɶ�ά�������ڽ���Բ������*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
	TransformCloud3D_Cloud2D(cloud, cloud_2d);
	/*��ʼ�������������Ϊδ����*/
	InitializePointProperty(cloud_property, cloud->size());
	/*����KD��*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	vector<int>pointIdxSearch;
	vector<float>pointDistance;
	kdtree.setInputCloud(cloud_2d);
	pcl::PointXYZ searchPoint;
	float min_z, max_z;//Բ���������ڵ�Z����Z��С;
	for (int i = 0; i < cloud->size(); i++)
	{
		//����õ�û�б�����,����з���;
		if (cloud_property[i].sort_id == 0)
		{
			searchPoint = cloud_2d->points[i];
			//Ѱ�ҵ������;
			int N = kdtree.radiusSearch(searchPoint, cylinder_radius, pointIdxSearch, pointDistance);
			/*����Բ���������ڵ���С��Zֵ������Zֵ;*/
			CalculateCylinderNeiborhoodMaxz_Minz(&max_z, &min_z, pointIdxSearch, cloud);
			/*�жϵ����;*/
			JudgeGroundPoints(i, max_height_difference, pointIdxSearch, cloud, min_z, max_z, cloud_property);
		}
	}

	/*�ѵ����ͷǵ����ֱ�����µĵ�����*/
	for (int i = 0; i < cloud->size(); i++)
	{
		if (cloud_property[i].sort_id == 1)
		{
			ground_cloud->push_back(cloud->points[i]);
			(*ground_cloud_property).push_back(cloud_property[i]);
		}
		if (cloud_property[i].sort_id == 2)
		{
			no_ground_cloud->push_back(cloud->points[i]);
			(*no_ground_cloud_property).push_back(cloud_property[i]);
		}
	}

}

