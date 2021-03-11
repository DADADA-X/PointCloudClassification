#include "CalculateFeature.h"
#include "DataStruct.h"

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>
#include <pcl\features\normal_3d.h>
#include <pcl\point_representation.h>
#include <pcl\io\pcd_io.h>
#include <vector>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>

using namespace std;


//CalculateFeature::CalculateFeature()
//{
//}


//CalculateFeature::~CalculateFeature()
//{
//}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////									点属性											/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*计算三个数的最大值;*/
int calulateMax(double a1D, double a2D, double a3D)
{
	double max;
	max = a1D;
	if (a2D > a1D)
	{
		max = a2D;
	}
	if (a3D > max)
	{
		max = a3D;
	}
	if (max == a1D)
	{
		return 1;
	}
	if (max == a2D)
	{
		return 2;
	}
	if (max == a3D)
	{
		return 3;
	}
}


vector<PointProperty> CalculateFeature::CalculateGeomatricalFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, double radius, vector<PointProperty> no_ground_cloud_property)
{
	/*建立KD树*/
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	vector<int>pointIdxSearch;//存储邻域点的点号;
	vector<float>pointDistance;//存储邻域点到搜索点的距离;
	int N;//存储邻域点的个数;
	kdtree.setInputCloud(no_ground_cloud);
	pcl::PointXYZ searchPoint;//搜索点;

	for (size_t j = 0; j < no_ground_cloud->size(); j++)
	{
		no_ground_cloud_property[j].point_id = j;
		searchPoint = no_ground_cloud->points[j];
		N = kdtree.radiusSearch(searchPoint, radius, pointIdxSearch, pointDistance);
		if (N > 3)
		{
			double XAve = 0.0, YAve = 0.0, ZAve = 0.0;    //用主成分分析法（PCA）计算若干个点拟合的平面法向量;
			CvMat A;
			double *a = new double[3 * N];                 //具体计算方法是计算某个矩阵的特征值和特征向量;
			int k = 0;
			for (int i = 0; i < N; i++)                   //最小特征值对应的特征向量就是法向量,最大特征值对应的特征向量是主方向;
			{
				a[k] = no_ground_cloud->points[pointIdxSearch[i]].x;
				a[k + N] = no_ground_cloud->points[pointIdxSearch[i]].y;
				a[k + 2 * N] = no_ground_cloud->points[pointIdxSearch[i]].z;
				k++;
				XAve += no_ground_cloud->points[pointIdxSearch[i]].x;
				YAve += no_ground_cloud->points[pointIdxSearch[i]].y;
				ZAve += no_ground_cloud->points[pointIdxSearch[i]].z;
			}
			XAve /= N; YAve /= N; ZAve /= N;
			CvMat*X, *XT, *XXT, *E, *I;
			X = cvCreateMat(3, N, CV_64FC1);
			XT = cvCreateMat(N, 3, CV_64FC1);
			XXT = cvCreateMat(3, 3, CV_64FC1);
			E = cvCreateMat(3, 3, CV_64FC1);	//特征向量
			I = cvCreateMat(3, 1, CV_64FC1);	//特征值
			A = cvMat(3, N, CV_64FC1, a);
			for (int i = 0; i < 3; i++)
			{
				for (int n = 0; n < N; n++)
				{
					if (i % 3 == 0)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - XAve);
					}
					if (i % 3 == 1)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - YAve);
					}
					if (i % 3 == 2)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - ZAve);
					}
				}
			}
			cvTranspose(X, XT);
			cvMatMul(X, XT, XXT);
			cvEigenVV(XXT, E, I);
			double valuemin, valuemax;
			int nummin = 0, nummax = 0;
			valuemin = cvmGet(I, 0, 0);
			valuemax = cvmGet(I, 0, 0);
			for (int i = 0; i < 3; i++)
			{
				if (valuemin > cvmGet(I, i, 0))
				{
					valuemin = cvmGet(I, i, 0);
					nummin = i;
				}
				if (valuemax < cvmGet(I, i, 0))
				{
					valuemax = cvmGet(I, i, 0);
					nummax = i;
				}
			}
			double lamada1, lamada2, lamada3;
			for (int i = 0; i < 3; i++)
			{
				if (i == nummin)
				{
					lamada3 = cvmGet(I, nummin, 0);
				}
				else
				{
					if (i == nummax)
					{
						lamada1 = cvmGet(I, nummax, 0);
					}
					else
					{
						lamada2 = cvmGet(I, i, 0);
					}

				}
			}
			/*计算法向量;*/
			no_ground_cloud_property[j].normal_x = cvmGet(E, nummin, 0);
			no_ground_cloud_property[j].normal_y = cvmGet(E, nummin, 1);
			no_ground_cloud_property[j].normal_z = cvmGet(E, nummin, 2);
			/*计算主方向;*/
			no_ground_cloud_property[j].Principal_x = cvmGet(E, nummax, 0);
			no_ground_cloud_property[j].Principal_y = cvmGet(E, nummax, 1);
			no_ground_cloud_property[j].Principal_z = cvmGet(E, nummax, 2);
			/*计算粗糙度;*/
			no_ground_cloud_property[j].point_surface_roughness = lamada3 / (lamada1 + lamada2 + lamada3);
			/*维数特征;*/
			double a1D, a2D, a3D;
			a1D = (sqrt(lamada1) - sqrt(lamada2)) / sqrt(lamada1);
			a2D = (sqrt(lamada2) - sqrt(lamada3)) / sqrt(lamada1);
			a3D = sqrt(lamada3) / sqrt(lamada1);
			no_ground_cloud_property[j].Dimension = calulateMax(a1D, a2D, a3D);
			delete[]a;
			cvReleaseMat(&X);
			cvReleaseMat(&XT);
			cvReleaseMat(&XXT);
			cvReleaseMat(&E);
			cvReleaseMat(&I);
		}
		else
		{
			/*计算法向量;*/
			no_ground_cloud_property[j].normal_x = 0.0;
			no_ground_cloud_property[j].normal_y = 0.0;
			no_ground_cloud_property[j].normal_z = 0.0;
			/*计算主方向;*/
			no_ground_cloud_property[j].Principal_x = 0.0;
			no_ground_cloud_property[j].Principal_y = 0.0;
			no_ground_cloud_property[j].Principal_z = 0.0;
			/*计算粗糙度;*/
			no_ground_cloud_property[j].point_surface_roughness = 0.0;
			/*维数特征;*/
			no_ground_cloud_property[j].Dimension = 3;
		 }
	}
	return no_ground_cloud_property;
}

/*把特征值标准化到0~1;*/
void CalculateFeature::NormalizationFeature(vector<PointProperty>& ReslutCloudProperty)
{
	int max_intensity = -1;
	for (int i = 0; i < ReslutCloudProperty.size(); i++)
	{
		if ((int)ReslutCloudProperty[i].intensity > max_intensity)
		{
			max_intensity = (int)ReslutCloudProperty[i].intensity;
		}

		ReslutCloudProperty[i].normal_color_r = (float)ReslutCloudProperty[i].color_r / 255;
		ReslutCloudProperty[i].normal_color_g = (float)ReslutCloudProperty[i].color_g / 255;
		ReslutCloudProperty[i].normal_color_b = (float)ReslutCloudProperty[i].color_b / 255;

		float rgb = ReslutCloudProperty[i].normal_color_r * ReslutCloudProperty[i].normal_color_r + ReslutCloudProperty[i].normal_color_g * ReslutCloudProperty[i].normal_color_g + ReslutCloudProperty[i].normal_color_b * ReslutCloudProperty[i].normal_color_b;
		rgb = sqrt(rgb);

		ReslutCloudProperty[i].normal_color_r = ReslutCloudProperty[i].normal_color_r / rgb;
		ReslutCloudProperty[i].normal_color_g = ReslutCloudProperty[i].normal_color_g / rgb;
		ReslutCloudProperty[i].normal_color_b = ReslutCloudProperty[i].normal_color_b / rgb;

	}


	for (int i = 0; i < ReslutCloudProperty.size(); i++)
	{
		int I = (int)ReslutCloudProperty[i].intensity;
		ReslutCloudProperty[i].normal_intensity = (float)I / max_intensity;
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////									segment属性									/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*计算每个segment的法向量和主方向，强度平均值和颜色平均值;*/
void CalculateFeature::CalculateSegmentFeature(vector<Segment>& FinalResult, vector<PointProperty> no_ground_cloud_property, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud)
{

	for (int i = 0; i < FinalResult.size(); i++)
	{
		////////////////////////////////////////////height, width, Zmin, Zmax////////////////////////////////////////////
		float width, height;
		float Zmin = 10000000.0, Zmax = -1000000;
		float Xmin = 10000000.0, Xmax = -1000000;
		float Ymin = 10000000.0, Ymax = -1000000;
		for (int j = 0; j < FinalResult[i].point_id.size(); j++)
		{
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].x > Xmax)
			{
				Xmax = no_ground_cloud->points[FinalResult[i].point_id[j]].x;
			}
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].y > Ymax)
			{
				Ymax = no_ground_cloud->points[FinalResult[i].point_id[j]].y;
			}
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].z > Zmax)
			{
				Zmax = no_ground_cloud->points[FinalResult[i].point_id[j]].z;
			}
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].x < Xmin)
			{
				Xmin = no_ground_cloud->points[FinalResult[i].point_id[j]].x;
			}
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].y < Ymin)
			{
				Ymin = no_ground_cloud->points[FinalResult[i].point_id[j]].y;
			}
			if (no_ground_cloud->points[FinalResult[i].point_id[j]].z < Zmin)
			{
				Zmin = no_ground_cloud->points[FinalResult[i].point_id[j]].z;
			}
		}

		height = Zmax - Zmin;
		width = sqrt((Xmax - Xmin)*(Xmax - Xmin) + (Ymax - Ymin)*(Ymax - Ymin));
		FinalResult[i].SegmentProperty.height = height;
		FinalResult[i].SegmentProperty.width = width;
		FinalResult[i].SegmentProperty.Zmax = Zmax;
		FinalResult[i].SegmentProperty.Zmin = Zmin;
	}

	/////////////////////////////////////////////////////			none_tree			/////////////////////////////////////////////////////////
	vector<int> h;/*存放高度小于0.5的segment_id;*/
	for (int i = 0; i < FinalResult.size(); i++)
	{
		FinalResult[i].none_tree = false; 
		//if (FinalResult[i].NormalPrincipleColorIntensity == 1)/*如果该segment来自法向量分割，则不能参与object的形成;*/
		//{
		//	FinalResult[i].none_tree = true;
		//}
		//for (int j = 0; j < FinalResult[i].point_id.size(); j++)
		//{
		//	int p_id = FinalResult[i].point_id[j];
		//	if (no_ground_cloud_property[p_id].sort_id == 4)
		//	{
		//		if (FinalResult[i].none_tree == false)
		//		{
		//			FinalResult[i].none_tree = true;
		//		}

		//	}
		//}
	}


	/////////////////////////////////////////////////////			intensity&RGBAverage			/////////////////////////////////////////////////////////
	for (int i = 0; i < FinalResult.size(); i++)
	{
		float I = 0;
		float R = 0, G = 0, B = 0;

		for (int j = 0; j < FinalResult[i].point_id.size(); j++)
		{
			int id = FinalResult[i].point_id[j];
			I = I + no_ground_cloud_property[id].normal_intensity;

			R = R + no_ground_cloud_property[id].normal_color_r;
			G = G + no_ground_cloud_property[id].normal_color_g;
			B = B + no_ground_cloud_property[id].normal_color_b;

		}
		FinalResult[i].SegmentProperty.average_intensity = I / FinalResult[i].point_id.size(); /*计算每个segment的强度平均值;*/

		FinalResult[i].SegmentProperty.average_color_r = R / FinalResult[i].point_id.size();   /*计算每个segment的颜色平均值;*/
		FinalResult[i].SegmentProperty.average_color_g = G / FinalResult[i].point_id.size();
		FinalResult[i].SegmentProperty.average_color_b = B / FinalResult[i].point_id.size();


		/////////////////////////////////////////////////////			normal&principle			/////////////////////////////////////////////////////////
		int N = FinalResult[i].point_id.size();
		if (N > 3)
		{
			double XAve = 0.0, YAve = 0.0, ZAve = 0.0;    //用主成分分析法（PCA）计算若干个点拟合的平面法向量;
			CvMat A;
			double *a = new double[3 * N];              //具体计算方法是计算某个矩阵的特征值和特征向量;
			int k = 0;
			//最小特征值对应的特征向量就是法向量,最大特征值对应的特征向量是主方向;
			for (int j = 0; j < N; j++)
			{
				int id = FinalResult[i].point_id[j];
				a[k] = no_ground_cloud->points[id].x;
				a[k + N] = no_ground_cloud->points[id].y;
				a[k + 2 * N] = no_ground_cloud->points[id].z;
				k++;
				XAve += no_ground_cloud->points[id].x;
				YAve += no_ground_cloud->points[id].y;
				ZAve += no_ground_cloud->points[id].z;

			}
			XAve /= N; YAve /= N; ZAve /= N;
			CvMat*X, *XT, *XXT, *E, *I;
			X = cvCreateMat(3, N, CV_64FC1);
			XT = cvCreateMat(N, 3, CV_64FC1);
			XXT = cvCreateMat(3, 3, CV_64FC1);
			E = cvCreateMat(3, 3, CV_64FC1);
			I = cvCreateMat(3, 1, CV_64FC1);
			A = cvMat(3, N, CV_64FC1, a);
			for (int i = 0; i < 3; i++)
			{
				for (int n = 0; n < N; n++)
				{
					if (i % 3 == 0)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - XAve);
					}
					if (i % 3 == 1)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - YAve);
					}
					if (i % 3 == 2)
					{
						cvmSet(X, i, n, cvmGet(&A, i, n) - ZAve);
					}
				}
			}

			cvTranspose(X, XT);
			cvMatMul(X, XT, XXT);
			cvEigenVV(XXT, E, I);
			double valuemin, valuemax;
			int nummin = 0, nummax = 0;
			valuemin = cvmGet(I, 0, 0);
			valuemax = cvmGet(I, 0, 0);
			for (int i = 0; i < 3; i++)
			{
				if (valuemin > cvmGet(I, i, 0))
				{
					valuemin = cvmGet(I, i, 0);
					nummin = i;
				}
				if (valuemax < cvmGet(I, i, 0))
				{
					valuemax = cvmGet(I, i, 0);
					nummax = i;
				}
			}
			double lamada1, lamada2, lamada3;
			for (int i = 0; i < 3; i++)
			{
				if (i == nummin)
				{
					lamada3 = cvmGet(I, nummin, 0);
				}
				else
				{
					if (i == nummax)
					{
						lamada1 = cvmGet(I, nummax, 0);
					}
					else
					{
						lamada2 = cvmGet(I, i, 0);
					}

				}
			}
			/*计算每个segment的法向量;*/
			FinalResult[i].SegmentProperty.segment_normal_x = cvmGet(E, nummin, 0);
			FinalResult[i].SegmentProperty.segment_normal_y = cvmGet(E, nummin, 1);
			FinalResult[i].SegmentProperty.segment_normal_z = cvmGet(E, nummin, 2);

			float nx1 = cvmGet(E, nummin, 0);
			float ny1 = cvmGet(E, nummin, 1);
			float nz1 = cvmGet(E, nummin, 2);
			float Distance = -(nx1*XAve + ny1 * YAve + nz1 * ZAve);

			/*计算每个segment的主方向;*/
			FinalResult[i].SegmentProperty.segment_Principal_x = cvmGet(E, nummax, 0);
			FinalResult[i].SegmentProperty.segment_Principal_y = cvmGet(E, nummax, 1);
			FinalResult[i].SegmentProperty.segment_Principal_z = cvmGet(E, nummax, 2);

			float px1 = cvmGet(E, nummax, 0);
			float py1 = cvmGet(E, nummax, 1);
			float pz1 = cvmGet(E, nummax, 2);

			/////////////////////////////////////////////////////			standard_deviation			/////////////////////////////////////////////////////////
			double g;
			g = sqrt(nx1*nx1 + ny1 * ny1 + nz1 * nz1);//求系数a1,b1,c1的平方和开平方;
			double f, f1, f2, f3, f4, h;
			double standard_deviation = 0;
			for (int kk = 0; kk < N; kk++) /*计算FinalResult[i]内每个点到法向量和主方向所确定的平面的距离之和;*/
			{
				int id = FinalResult[i].point_id[kk];
				f1 = (nx1*no_ground_cloud->points[id].x);
				f2 = (ny1*no_ground_cloud->points[id].y);
				f3 = (nz1*no_ground_cloud->points[id].z);
				f4 = Distance;
				f = (f1 + f2 + f3 + f4);
				f = abs(f);
				h = (f / g);
				standard_deviation += (h*h);
			}
			standard_deviation /= N;

			/////////////////////////////////////////////////////			angle between z & n || z & p			/////////////////////////////////////////////////////////
			double nx, ny, nz;
			double px, py, pz;
			nx = 0.0; ny = 0.0; nz = 1.0; px = 0.0, py = 0.0; pz = 1.0;
			float n_n1 = nx * nx1 + ny * ny1 + nz * nz1;
			float n_n = sqrt(nx*nx + ny * ny + nz * nz);
			float n1_n1 = sqrt(nx1*nx1 + ny1 * ny1 + nz1 * nz1);
			float Cosnormal = abs(n_n1 / (n_n*n1_n1));

			float p_p1 = px * px1 + py * py1 + pz * pz1;
			float p_p = sqrt(px*px + py * py + pz * pz);
			float p1_p1 = sqrt(px1*px1 + py1 * py1 + pz1 * pz1);
			float CosPrincipalDirection = abs(p_p1 / (p_p*p1_p1));

			/////////////////////////////////////////////////////			is_pole			/////////////////////////////////////////////////////////
			if (CosPrincipalDirection > 0.867 && (FinalResult[i].NormalPrincipleColorIntensity == 2))
			{
				FinalResult[i].is_pole = true;//竖直的杆;
			}
			FinalResult[i].SegmentProperty.Cosnormal = Cosnormal;
			FinalResult[i].SegmentProperty.CosPrincipalDirection = CosPrincipalDirection;
			FinalResult[i].SegmentProperty.standard_deviation = standard_deviation * 1000;

			delete[]a;
			cvReleaseMat(&X);
			cvReleaseMat(&XT);
			cvReleaseMat(&XXT);
			cvReleaseMat(&E);
			cvReleaseMat(&I);
		}
	}

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////									object属性									/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Object> CalculateFeature::CalculateObjectFeature(vector<Object> MultiObject, vector<Segment>FinalResult, pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud)
{
	for (int i = 0; i < MultiObject.size(); i++)
	{
		int tot_num = 0;
		int count = 0;
		float width, height;
		float Zmin = 10000000.0, Zmax = -1000000;
		float Xmin = 10000000.0, Xmax = -1000000;
		float Ymin = 10000000.0, Ymax = -1000000;
		/////////////////////////////////////////////////////			z&height			/////////////////////////////////////////////////////////
		for (int n = 0; n < MultiObject[i].SegmentID.size(); n++)
		{
			if (FinalResult[MultiObject[i].SegmentID[n]].SegmentProperty.Zmax > Zmax)
			{
				Zmax = FinalResult[MultiObject[i].SegmentID[n]].SegmentProperty.Zmax;
			}
			if (FinalResult[MultiObject[i].SegmentID[n]].SegmentProperty.Zmin < Zmin)
			{
				Zmin = FinalResult[MultiObject[i].SegmentID[n]].SegmentProperty.Zmin;
			}
		}
		MultiObject[i].Zmax = Zmax;
		MultiObject[i].Zmin = Zmin;
		MultiObject[i].height = Zmax - Zmin;

		////////////////PointDensityRatio&HeightDifferenceBeteewnGeometricalCenterAndBarycenter&NumOfPoint//////////////////
		float GeometricalCenterZ, BarycenterZ = 0.0;
		GeometricalCenterZ = (Zmax + Zmin) / 2.0;			//几何中心
		for (int j = 0; j < MultiObject[i].SegmentID.size(); j++)
		{
			for (int n = 0; n < FinalResult[MultiObject[i].SegmentID[j]].point_id.size(); n++)
			{
				tot_num++;
				BarycenterZ += no_ground_cloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].z;		//重心

				if (no_ground_cloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].z < (MultiObject[i].Zmin + 4))
				{
					count++;
				}
			}
		}
		MultiObject[i].PointDensityRatio = (double)count / (double)tot_num;	//低于0.4米的部分点云数量与整棵树点云数量的比值
		BarycenterZ /= tot_num;
		MultiObject[i].NumOfPoint = tot_num;
		MultiObject[i].HeightDifferenceBeteewnGeometricalCenterAndBarycenter = BarycenterZ - GeometricalCenterZ;

		////////////////average_standard_deviation&noamal_ratio&principle_ratio//////////////////
		double d = 0;
		int n_num = 0; int p_num = 0;
		for (int k = 0; k < MultiObject[i].SegmentID.size(); k++)
		{
			int seg_id = MultiObject[i].SegmentID[k];
			d = d + FinalResult[seg_id].SegmentProperty.standard_deviation;

			if (FinalResult[seg_id].NormalPrincipleColorIntensity == 1)
			{
				n_num = n_num + FinalResult[seg_id].point_id.size();
			}

			if (FinalResult[seg_id].NormalPrincipleColorIntensity == 2)
			{
				p_num = p_num + FinalResult[seg_id].point_id.size();
			}
		}
		d = d / MultiObject[i].SegmentID.size();
		MultiObject[i].average_standard_deviation = d;

		MultiObject[i].noamal_ratio = (double)n_num / (double)MultiObject[i].NumOfPoint;
		MultiObject[i].principle_ratio = (double)p_num / (double)MultiObject[i].NumOfPoint;
	}
	return MultiObject;
}