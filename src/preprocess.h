#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM};

class Preprocess
{
    public:
    float time_unit_scale;
    int time_unit;
    int lidar_type;

    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

};


#endif