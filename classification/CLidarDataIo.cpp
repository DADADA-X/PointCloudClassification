#include "Data_io.h"
#include "DataStruct.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ctime>

#include <pcl\io\pcd_io.h>
#include <pcl\point_types.h>

using namespace std;


//CLidarDataIo::CLidarDataIo()
//{
//}


//CLidarDataIo::~CLidarDataIo()
//{
//}

PointProperty *  CLidarDataIo::read_las(int * point_number, double *center_x, double *center_y, double *center_z,
	double *max_x, double *max_y, double *max_z, double *min_x, double *min_y, double *min_z,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/*las文件读取;*/
	PointCoordinate * points;
	int NumOfPoint;
	cout << "请输入待处的LiDAR数据的文件...." << endl;
	string Tempstr;
	cin >> Tempstr;
	const char*filepath;
	filepath = Tempstr.data();
	//filepath = "F:\\Dadah\\※research\\classification\\DWX\\data\\test.las";
	double TempCenterx = 0.0, TempCentery = 0.0, TempCenterz = 0.0;
	char *p;
	char MyHeaderArray[227];//存储文件头
	char MyVLRArray[54];//存储变长记录头
	char MyPointArray0[20];//存储formate0点数据
	char MyPointArray1[28];//存储formate1点数据
	char MyPointArray2[26];//存储formate2点数据
	char MyPointArray3[34];//存储formate3点数据
	char MyPointDataStartSignature[2];
	ifstream myifs(filepath, ios::binary);
	PointProperty *pointProperty;
	if (!myifs)
	{
		cout << "ERROR" << endl;
		exit(1);
	}
	else
	{
		//读取las文件头信息//
		myifs.read(MyHeaderArray, 227);
		p = &MyHeaderArray[0];
		for (int i = 0; i < 4; i++)
		{
			MyHeader.FileSignature[i] = *p;
			p++;
		}
		MyHeader.Reserved = *(unsigned long  *)p;
		p += 4;
		MyHeader.ProjectIDGUIDdata1 = *(unsigned long *)p;
		p += 4;
		MyHeader.ProjectIDGUIDdata2 = *(unsigned short *)p;
		p += 2;
		MyHeader.ProjectIDGUIDdata3 = *(unsigned short *)p;
		p += 2;
		for (int i = 0; i < 8; i++)
		{
			MyHeader.ProjectIDGUIDdata4[i] = *p;
			p++;
		}
		MyHeader.VersionMajor = *p;
		p++;
		MyHeader.VersionMinor = *p;
		p++;
		for (int i = 0; i < 32; i++)
		{
			MyHeader.SystemIdentifier[i] = *p;
			p++;
		}
		for (int i = 0; i < 32; i++)
		{
			MyHeader.GeneratingSoftware[i] = *p;
			p++;

		}
		MyHeader.FileCreationDayofYear = *(unsigned short *)p;
		p += 2;
		MyHeader.FileCreationYear = *(unsigned short *)p;
		p += 2;
		MyHeader.HeaderSize = *(unsigned short *)p;
		p += 2;
		MyHeader.OffSetToPointData = *(unsigned long *)p;
		p += 4;
		MyHeader.NumberOfVariableLengthRecords = *(unsigned long *)p;
		p += 4;
		MyHeader.PointDataFormatID = *p;
		p++;
		MyHeader.PointDataRecordLength = *(unsigned short *)p;
		p += 2;
		MyHeader.NumberOfPointRecords = *(unsigned long *)p;
		*point_number = (int)MyHeader.NumberOfPointRecords;
		p += 4;
		for (int i = 0; i < 5; i++)
		{
			MyHeader.Numberofpointsbyreturn[i] = *(unsigned long *)p;
			p += 4;
		}
		MyHeader.XScaleFactor = *(double *)p;
		p += 8;
		MyHeader.YScaleFactor = *(double *)p;
		p += 8;
		MyHeader.ZScaleFactor = *(double *)p;
		p += 8;
		MyHeader.XOffSet = *(double *)p;
		p += 8;
		MyHeader.YOffSet = *(double *)p;
		p += 8;
		MyHeader.ZOffSet = *(double *)p;
		p += 8;
		MyHeader.MaxX = *(double *)p;
		p += 8;
		MyHeader.MinX = *(double *)p;
		p += 8;
		MyHeader.MaxY = *(double *)p;
		p += 8;
		MyHeader.MinY = *(double *)p;
		p += 8;
		MyHeader.MaxZ = *(double *)p;
		p += 8;
		MyHeader.MinZ = *(double *)p;
		p += 8;
		//读取las VARIABLE LENGTH RECORD HEADER信息//
		for (int j = 0; j < MyHeader.NumberOfVariableLengthRecords; j++)
		{
			myifs.read(MyVLRArray, 54);
			p = &MyVLRArray[0];
			MyVLR.RecordSignature = *(unsigned short *)p;
			p += 2;
			for (int i = 0; i < 16; i++)
			{
				MyVLR.UserID[i] = *p;
				p++;
			}
			MyVLR.RecordID = *(unsigned short *)p;
			p += 2;
			MyVLR.RecordLengthAfterHeader = *(unsigned short *)p;
			p += 2;
			for (int i = 0; i < 32; i++)
			{
				MyVLR.Description[i] = *p;
				p++;
			}
			char *a = new char[MyVLR.RecordLengthAfterHeader];
			myifs.read(a, MyVLR.RecordLengthAfterHeader);
			delete[]a;
		}
		myifs.seekg(MyHeader.OffSetToPointData, ios::beg);
		//读取las的point信息//
		points = new PointCoordinate[*point_number];
		pointProperty = new PointProperty[*point_number];

		if (MyHeader.PointDataFormatID == 0)
		{
			for (int j = 0; j < MyHeader.NumberOfPointRecords; j++)
			{
				myifs.read(MyPointArray0, 20);
				p = &MyPointArray0[0];
				myPointData0.X = *(long*)p;
				points[j].x = (myPointData0.X*MyHeader.XScaleFactor) + MyHeader.XOffSet;//
				TempCenterx += points[j].x;
				p += 4;
				myPointData0.Y = *(long*)p;
				points[j].y = (myPointData0.Y *MyHeader.YScaleFactor) + MyHeader.YOffSet;//
				TempCentery += points[j].y;
				p += 4;
				myPointData0.Z = *(long*)p;
				points[j].z = (myPointData0.Z *MyHeader.ZScaleFactor) + MyHeader.ZOffSet;//
				TempCenterz += points[j].z;
				p += 4;
				pointProperty[j].intensity = *(unsigned short *)p;
				p += 2;
				char temp0 = *p;
				/*points[j].ReturnNumber=7&temp0;
				points[j].NumberofReturns=(56&temp0)>>3;*/
				myPointData0.ScanDirectionFlag = (64 & temp0) >> 6;
				myPointData0.EdgeofFlightLine = (128 & temp0) >> 7;
				p++;
				myPointData0.Classification = *p;
				p++;
				myPointData0.ScanAngleRank = *p;
				p++;
				myPointData0.UserData = *p;
				p++;
				myPointData0.PointSourceID = *(unsigned short *)p;
			}
		}
		if (MyHeader.PointDataFormatID == 1)
		{
			for (int j = 0; j < MyHeader.NumberOfPointRecords; j++)
			{
				myifs.read(MyPointArray1, 28);
				p = &MyPointArray1[0];
				myPointData1.X = *(long*)p;
				points[j].x = (myPointData1.X*MyHeader.XScaleFactor) + MyHeader.XOffSet;//
				TempCenterx += points[j].x;
				p += 4;
				myPointData1.Y = *(long*)p;
				points[j].y = (myPointData1.Y *MyHeader.YScaleFactor) + MyHeader.YOffSet;//
				TempCentery += points[j].y;
				p += 4;
				myPointData1.Z = *(long*)p;
				points[j].z = (myPointData1.Z *MyHeader.ZScaleFactor) + MyHeader.ZOffSet;//
				TempCenterz += points[j].z;
				p += 4;
				pointProperty[j].intensity = *(unsigned short *)p;
				p += 2;
				char temp1 = *p;
				/* points[j].ReturnNumber=7&temp1;
				 points[j].NumberofReturns=(56&temp1)>>3;*/
				myPointData1.ScanDirectionFlag = (64 & temp1) >> 6;
				myPointData1.EdgeofFlightLine = (128 & temp1) >> 7;
				p++;
				myPointData1.Classification = *p;
				p++;
				myPointData1.ScanAngleRank = *p;
				p++;
				myPointData1.UserData = *p;
				p++;
				myPointData1.PointSourceID = *(unsigned short *)p;
				p += 2;
				myPointData1.GPSTime = *(double*)p;
			}
		}
		if (MyHeader.PointDataFormatID == 2)
		{

			for (int j = 0; j < MyHeader.NumberOfPointRecords; j++)
			{
				myifs.read(MyPointArray2, 26);
				p = &MyPointArray2[0];
				myPointData0.X = *(long*)p;
				points[j].x = (myPointData0.X*MyHeader.XScaleFactor) + MyHeader.XOffSet;//
				TempCenterx += points[j].x;
				p += 4;
				myPointData0.Y = *(long*)p;
				points[j].y = (myPointData0.Y *MyHeader.YScaleFactor) + MyHeader.YOffSet;//
				TempCentery += points[j].y;
				p += 4;
				myPointData0.Z = *(long*)p;
				points[j].z = (myPointData0.Z *MyHeader.ZScaleFactor) + MyHeader.ZOffSet;//
				TempCenterz += points[j].z;
				p += 4;
				pointProperty[j].intensity = *(unsigned short *)p;
				p += 2;
				char temp0 = *p;
				/*points[j].ReturnNumber=7&temp0;
				points[j].NumberofReturns=(56&temp0)>>3;*/
				myPointData0.ScanDirectionFlag = (64 & temp0) >> 6;
				myPointData0.EdgeofFlightLine = (128 & temp0) >> 7;
				p++;
				myPointData0.Classification = *p;
				p++;
				myPointData0.ScanAngleRank = *p;
				p++;
				myPointData0.UserData = *p;
				p++;
				myPointData0.PointSourceID = *(unsigned short *)p;
				int R, G, B;
				p += 2;
				pointProperty[j].color_r = (uint8_t)(*(unsigned short *)p);
				p += 2;
				pointProperty[j].color_g = (uint8_t)(*(unsigned short *)p);
				p += 2;
				pointProperty[j].color_b = (uint8_t)(*(unsigned short *)p);
			}
		}
		if (MyHeader.PointDataFormatID == 3)
		{
			for (int j = 0; j < MyHeader.NumberOfPointRecords; j++)
			{
				myifs.read(MyPointArray3, 34);
				p = &MyPointArray3[0];
				myPointData1.X = *(long*)p;
				points[j].x = (myPointData1.X*MyHeader.XScaleFactor) + MyHeader.XOffSet;//
				TempCenterx += points[j].x;
				p += 4;
				myPointData1.Y = *(long*)p;
				points[j].y = (myPointData1.Y *MyHeader.YScaleFactor) + MyHeader.YOffSet;//
				TempCentery += points[j].y;
				p += 4;
				myPointData1.Z = *(long*)p;
				points[j].z = (myPointData1.Z *MyHeader.ZScaleFactor) + MyHeader.ZOffSet;//
				TempCenterz += points[j].z;
				p += 4;
				pointProperty[j].intensity = *(unsigned short *)p;
				p += 2;
				char temp1 = *p;
				/*points[j].ReturnNumber=7&temp1;
				points[j].NumberofReturns=(56&temp1)>>3;*/
				myPointData1.ScanDirectionFlag = (64 & temp1) >> 6;
				myPointData1.EdgeofFlightLine = (128 & temp1) >> 7;
				p++;
				myPointData1.Classification = *p;
				p++;
				myPointData1.ScanAngleRank = *p;
				p++;
				myPointData1.UserData = *p;
				p++;
				myPointData1.PointSourceID = *(unsigned short *)p;
				p += 2;
				myPointData1.GPSTime = *(double*)p;
				int R, G, B;
				p += 2;
				pointProperty[j].color_r = (uint8_t)(*(unsigned short *)p);
				p += 2;
				pointProperty[j].color_g = (uint8_t)(*(unsigned short *)p);
				p += 2;
				pointProperty[j].color_b = (uint8_t)(*(unsigned short *)p);
			}
		}
	}
	myifs.close();
	TempCenterx /= MyHeader.NumberOfPointRecords;
	TempCentery /= MyHeader.NumberOfPointRecords;
	TempCenterz /= MyHeader.NumberOfPointRecords;
	*center_x = TempCenterx;
	*center_y = TempCentery;
	*center_z = TempCenterz;
	*max_x = MyHeader.MaxX - *center_x;
	*max_y = MyHeader.MaxY - *center_y;
	*max_z = MyHeader.MaxZ - *center_z;
	*min_x = MyHeader.MinX - *center_x;
	*min_y = MyHeader.MinY - *center_y;
	*min_z = MyHeader.MinZ - *center_z;
	CopyPoint(*point_number, points, cloud, *center_x, *center_y, *center_z);
	return pointProperty;
}

void CLidarDataIo::CopyPoint(int point_number, PointCoordinate * points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	double center_x, double center_y, double center_z)
{
	/*将重心化坐标赋值给点云cloud;*/
	for (int i = 0; i < point_number; i++)
	{
		pcl::PointXYZ temp_point;
		temp_point.x = points[i].x - center_x;
		temp_point.y = points[i].y - center_y;
		temp_point.z = points[i].z - center_z;
		cloud->push_back(temp_point);
	}
}
