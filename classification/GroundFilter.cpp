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

/*把三维点云转换成2维点云;*/
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

/*初始化点云的类别属性为“未分类”;*/
void CGroundFilter::InitializePointProperty(PointProperty * cloud_property, int point_number)
{
	for (int i = 0; i < point_number; i++)
	{
		cloud_property[i].sort_id = 0;
	}
}

/*计算圆柱邻域内的最大和最小的Z值;*/
void CGroundFilter::CalculateCylinderNeiborhoodMaxz_Minz(float *max_z, float *min_z, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d)
{
	*min_z = 10000000.0;
	*max_z = -1000000.0;
	/*计算圆柱形邻域内的最小的Z值和最大的Z值;*/
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

/*判断是否为地面点;*/
void CGroundFilter::JudgeGroundPoints(int point_id, float max_height_difference, vector<int>pointIdxSearch, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d,
	float min_z, float max_z, PointProperty * cloud_property)
{
	/*判断地面点;*/
	if (cloud_3d->points[point_id].z - min_z < max_height_difference)
	{
		cloud_property[point_id].sort_id = 1;//地面点;
		/*比当前地面点还低的点也作为地面点;*/
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
		cloud_property[point_id].sort_id = 2;//非地面点;
		/*比当前非地面点还高的点也作为非地面点;*/
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
	/*把三维点云转换成二维点云用于建立圆柱邻域*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
	TransformCloud3D_Cloud2D(cloud, cloud_2d);
	/*初始化点云类别属性为未分类*/
	InitializePointProperty(cloud_property, cloud->size());
	/*创建KD树*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	vector<int>pointIdxSearch;
	vector<float>pointDistance;
	kdtree.setInputCloud(cloud_2d);
	pcl::PointXYZ searchPoint;
	float min_z, max_z;//圆柱形邻域内的Z最大和Z最小;
	for (int i = 0; i < cloud->size(); i++)
	{
		//如果该点没有被分类,则进行分类;
		if (cloud_property[i].sort_id == 0)
		{
			searchPoint = cloud_2d->points[i];
			//寻找点的邻域;
			int N = kdtree.radiusSearch(searchPoint, cylinder_radius, pointIdxSearch, pointDistance);
			/*计算圆柱形邻域内的最小的Z值和最大的Z值;*/
			CalculateCylinderNeiborhoodMaxz_Minz(&max_z, &min_z, pointIdxSearch, cloud);
			/*判断地面点;*/
			JudgeGroundPoints(i, max_height_difference, pointIdxSearch, cloud, min_z, max_z, cloud_property);
		}
	}

	/*把地面点和非地面点分别放入新的点云中*/
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

