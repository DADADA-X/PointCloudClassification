#include "viewer.h"
#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\visualization\cloud_viewer.h>


//CLiDARViewer::CLiDARViewer()
//{
//}


//CLiDARViewer::~CLiDARViewer()
//{
//}

void CLiDARViewer::DisplayPointCloudXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudold, PointProperty * point_property)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < cloudold->size(); i++)
	{
		pcl::PointXYZRGB Temp;
		Temp.x = cloudold->points[i].x; Temp.y = cloudold->points[i].y; Temp.z = cloudold->points[i].z;
		Temp.r = point_property[i].color_r;
		Temp.g = point_property[i].color_g;
		Temp.b = point_property[i].color_b;
		cloud->points.push_back(Temp);
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PointCloudXYZDisplay"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void CLiDARViewer::DisplaySegmentationResultOnMultiWindows(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud,
	vector<Segment> SegmentResultBasedOnNormal, vector<Segment> SegmentResultBasedOnPrinciple,
	vector<Segment> SegmentResultBasedOnColor, vector<Segment> SegmentResultBasedOnIntensity)
{
	//多视口显示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Segment Result Based on 4 Methods"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmentResultBasedOnNormal(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmentResultBasedOnPrinciple(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmentResultBasedOnColor(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmentResultBasedOnIntensity(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;
	srand((unsigned)time(NULL));
	for (int i = 0; i < SegmentResultBasedOnNormal.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < SegmentResultBasedOnNormal[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResultBasedOnNormal[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResultBasedOnNormal[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResultBasedOnNormal[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloudSegmentResultBasedOnNormal->points.push_back(TempPoint);
		}
	}

	for (int i = 0; i < SegmentResultBasedOnPrinciple.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < SegmentResultBasedOnPrinciple[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResultBasedOnPrinciple[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResultBasedOnPrinciple[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResultBasedOnPrinciple[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloudSegmentResultBasedOnPrinciple->points.push_back(TempPoint);
		}
	}
	for (int i = 0; i < SegmentResultBasedOnColor.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < SegmentResultBasedOnColor[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResultBasedOnColor[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResultBasedOnColor[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResultBasedOnColor[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloudSegmentResultBasedOnColor->points.push_back(TempPoint);
		}
	}
	for (int i = 0; i < SegmentResultBasedOnIntensity.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < SegmentResultBasedOnIntensity[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResultBasedOnIntensity[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResultBasedOnIntensity[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResultBasedOnIntensity[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloudSegmentResultBasedOnIntensity->points.push_back(TempPoint);
		}
	}
	int v1(0);
	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Normal", 10, 10, "v1 text", v1);
	viewer->addPointCloud(cloudSegmentResultBasedOnNormal, "normal", v1);
	int v2(0);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.25, 0.25, 0.25, v2);
	viewer->addText("Principle", 10, 10, "v2 text", v2);
	viewer->addPointCloud(cloudSegmentResultBasedOnPrinciple, "Principle", v2);

	int v3(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v3);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v3);
	viewer->addText("Color", 10, 10, "v3 text", v3);
	viewer->addPointCloud(cloudSegmentResultBasedOnColor, "Color", v3);

	int v4(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v4);
	viewer->setBackgroundColor(0.75, 0.75, 0.75, v4);
	viewer->addText("Intensity", 10, 10, "v4 text", v4);
	viewer->addPointCloud(cloudSegmentResultBasedOnIntensity, "Intensity", v4);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void CLiDARViewer::DisPlaySegmentationResult(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<Segment> SegmentResult)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;
	srand((unsigned)time(NULL));
	for (int i = 0; i < SegmentResult.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < SegmentResult[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResult[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResult[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResult[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloud->points.push_back(TempPoint);
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("DisPlaySegmentationResult"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void CLiDARViewer::DisPlaySegmentationSortResult(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud, vector<Segment> SegmentResult)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;

	for (int i = 0; i < SegmentResult.size(); i++)
	{
		if (SegmentResult[i].NormalPrincipleColorIntensity == 1)
		{
			R = 255;
			G = 0;
			B = 0;
		}
		if (SegmentResult[i].NormalPrincipleColorIntensity == 2)
		{
			R = 0;
			G = 0;
			B = 255;
		}
		if (SegmentResult[i].NormalPrincipleColorIntensity == 3)
		{
			R = 0;
			G = 255;
			B = 0;
		}
		if (SegmentResult[i].NormalPrincipleColorIntensity == 4)
		{
			R = 0;
			G = 255;
			B = 0;
		}
		for (int j = 0; j < SegmentResult[i].point_id.size(); j++)
		{
			pcl::PointXYZRGB TempPoint;
			TempPoint.x = no_ground_cloud->points[SegmentResult[i].point_id[j]].x;
			TempPoint.y = no_ground_cloud->points[SegmentResult[i].point_id[j]].y;
			TempPoint.z = no_ground_cloud->points[SegmentResult[i].point_id[j]].z;
			TempPoint.r = R;
			TempPoint.g = G;
			TempPoint.b = B;
			cloud->points.push_back(TempPoint);
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("DisPlaySegmentationSortResult"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void CLiDARViewer::DisplayMultiObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;
	srand((unsigned)time(NULL));
	for (int i = 0; i < MultiObject.size(); i++)
	{
		R = rand() % 255;
		G = rand() % 255;
		B = rand() % 255;
		for (int j = 0; j < MultiObject[i].SegmentID.size(); j++)
		{
			for (int n = 0; n < FinalResult[MultiObject[i].SegmentID[j]].point_id.size(); n++)
			{
				pcl::PointXYZRGB TempPoint;
				TempPoint.x = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].x;
				TempPoint.y = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].y;
				TempPoint.z = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].z;
				TempPoint.r = R;
				TempPoint.g = G;
				TempPoint.b = B;
				cloud->points.push_back(TempPoint);
			}
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("DisplayMultiObject"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

}

void  CLiDARViewer::DisplayMultiSortObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;
	srand((unsigned)time(NULL));
	for (int i = 0; i < MultiObject.size(); i++)
	{
		if (MultiObject[i].ObjectSort == 1)
		{
			//cout << "建筑物" << endl;
			//建筑物;
			R = 78;
			G = 188;
			B = 197;
		}
		else if (MultiObject[i].ObjectSort == 2)
		{
			//cout << "树木" << endl;
			//树木;
			R = 142;
			G = 195;
			B = 31;
		}
		else if (MultiObject[i].ObjectSort == 3)
		{
			//cout << "标牌" << endl;
			//标牌;
			R = 229;
			G = 37;
			B = 24;
		}
		else if (MultiObject[i].ObjectSort == 4)
		{
			//cout << "路灯" << endl;
			//路灯;
			R = 235;
			G = 205;
			B = 0;
		}
		else if (MultiObject[i].ObjectSort == 5)
		{
			//cout << "人工其他" << endl;
			//人工其他;
			R = 236;
			G = 114;
			B = 138;
		}
		else if (MultiObject[i].ObjectSort == 6)
		{
			//cout << "自然其他" << endl;
			//自然其他;
			R = 193;
			G = 255;
			B = 193;
		}
		else if (MultiObject[i].ObjectSort == 7)
		{
			//cout << "碎片" << endl;
			//碎片;
			R = 195;
			G = 195;
			B = 195;
		}

		for (int j = 0; j < MultiObject[i].SegmentID.size(); j++)
		{
			for (int n = 0; n < FinalResult[MultiObject[i].SegmentID[j]].point_id.size(); n++)
			{
				pcl::PointXYZRGB TempPoint;
				TempPoint.x = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].x;
				TempPoint.y = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].y;
				TempPoint.z = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].z;
				TempPoint.r = R;
				TempPoint.g = G;
				TempPoint.b = B;
				cloud->points.push_back(TempPoint);
			}
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("DisplayMultiSortObject"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}
}

void CLiDARViewer::DisplayTreeObject(pcl::PointCloud<pcl::PointXYZ>::Ptr ReslutCloud, vector<Segment>FinalResult, vector<Object> MultiObject)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	uint8_t R;
	uint8_t G;
	uint8_t B;
	srand((unsigned)time(NULL));
	for (int i = 0; i < MultiObject.size(); i++)
	{
		if (MultiObject[i].ObjectSort == 2)
		{
			R = 162;
			G = 205;
			B = 90;
		}
		else
		{
			R = 190;
			G = 190;
			B = 190;
		}
		for (int j = 0; j < MultiObject[i].SegmentID.size(); j++)
		{
			for (int n = 0; n < FinalResult[MultiObject[i].SegmentID[j]].point_id.size(); n++)
			{
				pcl::PointXYZRGB TempPoint;
				TempPoint.x = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].x;
				TempPoint.y = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].y;
				TempPoint.z = ReslutCloud->points[FinalResult[MultiObject[i].SegmentID[j]].point_id[n]].z;
				TempPoint.r = R;
				TempPoint.g = G;
				TempPoint.b = B;
				cloud->points.push_back(TempPoint);
			}
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("DisplayTreeObject"));
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

}
