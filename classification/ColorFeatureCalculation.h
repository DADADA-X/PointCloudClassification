#ifndef COLOR
#define COLOR

#include "DataStruct.h"
#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

class ColorFeature
{
public:
//	ColorFeature();
//	~ColorFeature();
	PointProperty * PointCloudSmooth_Median(int  NumOfNeiboringPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PointProperty * cloud_property);

};



#endif // !COLOR

