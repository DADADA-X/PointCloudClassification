#pragma once
#ifndef LAS_DATA_STRUCT_
#define LAS_DATA_STRUCT_

#include <vector>
#include <pcl\point_types.h>

using namespace std;

/*las��ʽ�����ݸ�ʽ��֧��LAS1.0 1.1 1.2*/
  //�ļ�ͷ227���ֽ�//
struct HEADERBLOCK
{
	char FileSignature[4];
	unsigned long  Reserved;
	unsigned long  ProjectIDGUIDdata1;
	unsigned short ProjectIDGUIDdata2;
	unsigned short ProjectIDGUIDdata3;
	unsigned char ProjectIDGUIDdata4[8];
	unsigned char VersionMajor;//���汾��1
	unsigned char VersionMinor;//���汾��.0
	char SystemIdentifier[32];
	char GeneratingSoftware[32];
	unsigned short  FileCreationDayofYear;
	unsigned short  FileCreationYear;
	unsigned short  HeaderSize;//�ļ�ͷ�Ĵ�С
	unsigned long OffSetToPointData;//���ļ���ͷ�������ݼ�¼ƫ�Ƶ��ֽ���
	unsigned long NumberOfVariableLengthRecords;//�䳤��¼�ĸ���
	unsigned char PointDataFormatID;//�����ݼ�¼�ĸ�ʽ��
	unsigned short PointDataRecordLength;//ÿ�����¼���ֽ���
	unsigned long  NumberOfPointRecords;//This field contains the total number of point records within the file.
	unsigned long Numberofpointsbyreturn[5];/* This field contains an array of the total point records per return.
	The first unsigned long value will be the total number of records from the first return, and the
		second contains the total number for return two, and so forth up to five returns. */

	double  XScaleFactor;//x�ı����߻�λ
	double  YScaleFactor;//y�ı����߻�λ
	double  ZScaleFactor;//z�ı����߻�λ
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
//�䳤��¼ͷ54 ���ֽ�//
struct VARISBLELENGTHRECORDS
{
	unsigned short RecordSignature;
	char UserID[16];
	unsigned short RecordID;
	unsigned short RecordLengthAfterHeader;//�䳤��¼�ڱ߳���¼ͷ����ֽ���
	char Description[32];
};
/*���¼�ĸ�ʽ0*/
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
	unsigned short Intensity;//����ǿ��
	unsigned short  ReturnNumber;//�ܻز�����
	unsigned short  NumberofReturns;//���λز���
	unsigned short  ScanDirectionFlag;/*A bit value of 1 is a positive scan direction, and a
	bit value of 0 is a negative scan direction (where positive scan direction is a scan moving
		from the left side of the in-track direction to the right side and negative the opposite). */
	unsigned short  EdgeofFlightLine;/*The Edge of Flight Line data bit has a value of 1 only when the point is at
	the end of a scan.  It is the last point on a given scan line before it changes direction. */
	unsigned char Classification;//
	char ScanAngleRank;/*The Scan Angle Rank is a signed one-byte number with a valid range from -90
					   to +90.  The Scan Angle Rank is the angle (rounded to the nearest integer in the absolute value
					   sense) at which the laser point was output from the laser system including the roll of the aircraft.
					   The scan angle is within 1 degree of accuracy from +90 to �C90 degrees.  The scan angle is an
					   angle based on 0 degrees being nadir, and �C90 degrees to the left side of the aircraft in the
					   direction of flight.*/
	unsigned char UserData;
	unsigned short PointSourceID;

};
/*���¼�ĸ�ʽ1*/
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
	double GPSTime;//gpsʱ��;
};
/*���¼�ĸ�ʽ2*/
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
/*���¼�ĸ�ʽ3*/
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
	double GPSTime;//gpsʱ��;
};


/*�Զ���ṹ��*/
//LiDAR�������;
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

	/*classification:0��ʾδ����,1��ʾ����,2��ʾ�ǵ���,3��ʾ��ľ,4���ƺ*/
	uint8_t sort_id;

	/*normal*/
	float normal_x;
	float normal_y;
	float normal_z;

	/*principal*/
	float Principal_x;
	float Principal_y;
	float Principal_z;

	/*SuperVoxel��ά��������1Ϊ��״�ֲ���2Ϊ��״�ֲ���3Ϊ��״�ֲ�;*/
	uint8_t Dimension;

	int point_id;
	float point_surface_roughness;
	int  segment_id;
	vector<int>NeiboringSegmentID;
	vector<int> NeiboringPointsID;

	bool final_result_point;/*����õ㲻��FinalResult�У���ֵΪfalse;*/
	


};

//block����
struct SegmentFeature
{
	/*segment�ķ�����;*/
	float segment_normal_x;
	float segment_normal_y;
	float segment_normal_z;

	/*segment��������;*/
	float segment_Principal_x;
	float segment_Principal_y;
	float segment_Principal_z;

	/*segment����ɫƽ��ֵ;*/
	float average_color_r;
	float average_color_g;
	float average_color_b;

	/*segment��ƽ��ǿ��;*/
	float average_intensity;

	float width;
	float height;
	float Zmin;
	float Zmax;

	/*ƽ����ϲв�;*/
	double standard_deviation;

	/*segment�������루0,0,1���н�����;*/
	float Cosnormal;

	/*segment�������루0,0,1���н�����;*/
	float CosPrincipalDirection;
};

//�ָ�
struct Segment
{
	vector<int> point_id;
	int segment_id;/*-1δ�ָ0, 1, 2, 3...�ָ��*/
	int NormalPrincipleColorIntensity;/*1������������԰��������ָ2������������԰�������ָ3������������԰���ɫ�ָ4������������԰�ǿ�ȷָ�;*/
	vector<int> EdgePointsIDOfOneSegment;
	SegmentFeature SegmentProperty;
	vector<int> NeiboringSegmentID;
	int ObjectID;
	bool is_pole;/*true:��ֱ�ĸ�;*/
	bool none_tree;  /*�����segment���Է������ָ���ǵͰ��Ĳ�ƺ���򲻲���object���γ�;*/
	bool  is_sod;/*�ǲ�ƺֵΪtrue;*/
};

//����
struct FitnessScore
{
	int point_id;
	int segment_id;

	float score_normal;    /*�ڷ�����ͨ����point_id��������segment_id�ĵ÷�;*/
	float score_principle; /*��������ͨ����point_id��������segment_id�ĵ÷�;*/
	float score_color;     /*����ɫͨ����point_id��������segment_id�ĵ÷�;*/
	float score_intensity; /*��ǿ��ͨ����point_id��������segment_id�ĵ÷�;*/
	float max_score;
	float min_score;
};

//object
struct Object
{
	vector<int> SegmentID;
	int ObjectID;
	int ObjectSort;/*0��ʾδ����,1������,2��ʾ����,3��ʾ��ͨ��־��,4·��,5��ʾ�����˹�Ŀ��(�й���)��6������ȻĿ�꣨������;7������С��1000����Ƭ;*/
	float width;
	float height;
	float Zmin;
	float Zmax;
	float PointDensityRatio;
	float HeightDifferenceBeteewnGeometricalCenterAndBarycenter;
	int NumOfPoint;
	/*ÿ��objece��segment��standard_deviation��ƽ��ֵ;*/
	double average_standard_deviation;
	/*ÿ��object�����Է������ָ��segment����֮��ռ��object�ܵ����ı���;*/
	double noamal_ratio;
	/*ÿ��object������������ָ��segment����֮��ռ��object�ܵ����ı���;*/
	double principle_ratio;
	/*�����object������segment����������ָ���и�״��;*/
	bool HavePole;

};

#endif // !LAS_DATA_STRUCT_