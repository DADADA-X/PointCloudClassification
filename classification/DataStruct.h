#pragma once
#ifndef LAS_DATA_STRUCT_
#define LAS_DATA_STRUCT_

#include <vector>
#include <pcl\point_types.h>

using namespace std;

/*las格式的数据格式，支持LAS1.0 1.1 1.2*/
  //文件头227个字节//
struct HEADERBLOCK
{
	char FileSignature[4];
	unsigned long  Reserved;
	unsigned long  ProjectIDGUIDdata1;
	unsigned short ProjectIDGUIDdata2;
	unsigned short ProjectIDGUIDdata3;
	unsigned char ProjectIDGUIDdata4[8];
	unsigned char VersionMajor;//主版本号1
	unsigned char VersionMinor;//副版本号.0
	char SystemIdentifier[32];
	char GeneratingSoftware[32];
	unsigned short  FileCreationDayofYear;
	unsigned short  FileCreationYear;
	unsigned short  HeaderSize;//文件头的大小
	unsigned long OffSetToPointData;//从文件开头到点数据记录偏移的字节数
	unsigned long NumberOfVariableLengthRecords;//变长记录的个数
	unsigned char PointDataFormatID;//点数据记录的格式号
	unsigned short PointDataRecordLength;//每个点记录的字节数
	unsigned long  NumberOfPointRecords;//This field contains the total number of point records within the file.
	unsigned long Numberofpointsbyreturn[5];/* This field contains an array of the total point records per return.
	The first unsigned long value will be the total number of records from the first return, and the
		second contains the total number for return two, and so forth up to five returns. */

	double  XScaleFactor;//x的比例尺或单位
	double  YScaleFactor;//y的比例尺或单位
	double  ZScaleFactor;//z的比例尺或单位
	double  XOffSet;
	double  YOffSet;
	double  ZOffSet;
	double  MaxX;
	double  MinX;
	double  MaxY;
	double  MinY;
	double  MaxZ;
	double  MinZ;
};
//变长记录头54 个字节//
struct VARISBLELENGTHRECORDS
{
	unsigned short RecordSignature;
	char UserID[16];
	unsigned short RecordID;
	unsigned short RecordLengthAfterHeader;//变长记录在边长记录头后的字节数
	char Description[32];
};
/*点记录的格式0*/
struct  POINTDATARECORDFORMAT0
{
	/*The X, Y, and Z values are stored as long integers.  The X, Y, and Z values are
	used in conjunction with the scale values and the offset values to determine the
	coordinate for each point.
	Xcoordinate = (Xrecord * Xscale) + Xoffset
	Ycoordinate = (Yrecord * Yscale) + Yoffset
	Zcoordinate = (Zrecord * Zscale) + Zoffset */
	long X;//
	long Y;//
	long Z;//
	unsigned short Intensity;//反射强度
	unsigned short  ReturnNumber;//总回波次数
	unsigned short  NumberofReturns;//本次回波数
	unsigned short  ScanDirectionFlag;/*A bit value of 1 is a positive scan direction, and a
	bit value of 0 is a negative scan direction (where positive scan direction is a scan moving
		from the left side of the in-track direction to the right side and negative the opposite). */
	unsigned short  EdgeofFlightLine;/*The Edge of Flight Line data bit has a value of 1 only when the point is at
	the end of a scan.  It is the last point on a given scan line before it changes direction. */
	unsigned char Classification;//
	char ScanAngleRank;/*The Scan Angle Rank is a signed one-byte number with a valid range from -90
					   to +90.  The Scan Angle Rank is the angle (rounded to the nearest integer in the absolute value
					   sense) at which the laser point was output from the laser system including the roll of the aircraft.
					   The scan angle is within 1 degree of accuracy from +90 to –90 degrees.  The scan angle is an
					   angle based on 0 degrees being nadir, and –90 degrees to the left side of the aircraft in the
					   direction of flight.*/
	unsigned char UserData;
	unsigned short PointSourceID;

};
/*点记录的格式1*/
struct  POINTDATARECORDFORMAT1
{
	long X;
	long Y;
	long Z;
	unsigned short Intensity;
	unsigned short  ReturnNumber;
	unsigned short  NumberofReturns;
	unsigned short  ScanDirectionFlag;
	unsigned short  EdgeofFlightLine;
	unsigned char Classification;
	char ScanAngleRank;
	unsigned char UserData;
	unsigned short PointSourceID;
	double GPSTime;//gps时间;
};
/*点记录的格式2*/
struct  POINTDATARECORDFORMAT2
{
	long X;
	long Y;
	long Z;
	unsigned short Intensity;
	unsigned short  ReturnNumber;
	unsigned short  NumberofReturns;
	unsigned short  ScanDirectionFlag;
	unsigned short  EdgeofFlightLine;
	unsigned char Classification;
	char ScanAngleRank;
	unsigned char UserData;
	unsigned short PointSourceID;
	unsigned short R;
	unsigned short G;
	unsigned short B;
};
/*点记录的格式3*/
struct  POINTDATARECORDFORMAT3
{
	long X;
	long Y;
	long Z;
	unsigned short Intensity;
	unsigned short  ReturnNumber;
	unsigned short  NumberofReturns;
	unsigned short  ScanDirectionFlag;
	unsigned short  EdgeofFlightLine;
	unsigned char Classification;
	char ScanAngleRank;
	unsigned char UserData;
	unsigned short PointSourceID;
	unsigned short R;
	unsigned short G;
	unsigned short B;
	double GPSTime;//gps时间;
};


/*自定义结构体*/
//LiDAR点的坐标;
struct PointCoordinate
{
	double x;
	double y;
	double z;
};

struct  PointProperty
{
	/*RGB & normalization*/
	uint8_t color_r;            float normal_color_r;
	uint8_t color_g;            float normal_color_g;
	uint8_t color_b;            float normal_color_b;

	/*intensity & normalizaion*/
	unsigned short intensity;   float normal_intensity;

	/*classification:0表示未分类,1表示地面,2表示非地面,3表示树木,4表草坪*/
	uint8_t sort_id;

	/*normal*/
	float normal_x;
	float normal_y;
	float normal_z;

	/*principal*/
	float Principal_x;
	float Principal_y;
	float Principal_z;

	/*SuperVoxel的维数特征，1为线状分布，2为面状分布，3为球状分布;*/
	uint8_t Dimension;

	int point_id;
	float point_surface_roughness;
	int  segment_id;
	vector<int>NeiboringSegmentID;
	vector<int> NeiboringPointsID;

	bool final_result_point;/*如果该点不在FinalResult中，该值为false;*/
	


};

//block特征
struct SegmentFeature
{
	/*segment的法向量;*/
	float segment_normal_x;
	float segment_normal_y;
	float segment_normal_z;

	/*segment的主方向;*/
	float segment_Principal_x;
	float segment_Principal_y;
	float segment_Principal_z;

	/*segment的颜色平均值;*/
	float average_color_r;
	float average_color_g;
	float average_color_b;

	/*segment的平均强度;*/
	float average_intensity;

	float width;
	float height;
	float Zmin;
	float Zmax;

	/*平面拟合残差;*/
	double standard_deviation;

	/*segment法向量与（0,0,1）夹角余弦;*/
	float Cosnormal;

	/*segment主方向与（0,0,1）夹角余弦;*/
	float CosPrincipalDirection;
};

//分割
struct Segment
{
	vector<int> point_id;
	int segment_id;/*-1未分割，0, 1, 2, 3...分割号*/
	int NormalPrincipleColorIntensity;/*1代表该区域来自按法向量分割，2代表该区域来自按主方向分割，3代表该区域来自按颜色分割，4代表该区域来自按强度分割;*/
	vector<int> EdgePointsIDOfOneSegment;
	SegmentFeature SegmentProperty;
	vector<int> NeiboringSegmentID;
	int ObjectID;
	bool is_pole;/*true:竖直的杆;*/
	bool none_tree;  /*如果该segment来自法向量分割或是低矮的草坪，则不参与object的形成;*/
	bool  is_sod;/*是草坪值为true;*/
};

//分数
struct FitnessScore
{
	int point_id;
	int segment_id;

	float score_normal;    /*在法向量通道点point_id对于区域segment_id的得分;*/
	float score_principle; /*在主方向通道点point_id对于区域segment_id的得分;*/
	float score_color;     /*在颜色通道点point_id对于区域segment_id的得分;*/
	float score_intensity; /*在强度通道点point_id对于区域segment_id的得分;*/
	float max_score;
	float min_score;
};

//object
struct Object
{
	vector<int> SegmentID;
	int ObjectID;
	int ObjectSort;/*0表示未分类,1建筑物,2表示行树,3表示交通标志牌,4路灯,5表示其他人工目标(有规则)，6其他自然目标（不规则）;7：点数小于1000的碎片;*/
	float width;
	float height;
	float Zmin;
	float Zmax;
	float PointDensityRatio;
	float HeightDifferenceBeteewnGeometricalCenterAndBarycenter;
	int NumOfPoint;
	/*每个objece中segment的standard_deviation的平均值;*/
	double average_standard_deviation;
	/*每个object中来自法向量分割的segment点数之和占该object总点数的比例;*/
	double noamal_ratio;
	/*每个object中来自主方向分割的segment点数之和占该object总点数的比例;*/
	double principle_ratio;
	/*如果该object里面有segment来自主方向分割，则有杆状物;*/
	bool HavePole;

};

#endif // !LAS_DATA_STRUCT_