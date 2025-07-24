#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out){
    switch (time_unit)
    {
        case SEC:
        time_unit_scale = 1.e3f;
        break;
        case MS:
        time_unit_scale = 1.f;
        break;
        case US:
        time_unit_scale = 1.e-3f;
        break;
        case NS:
        time_unit_scale = 1.e-6f;
        break;
        default:
        time_unit_scale = 1.f;
        break;
    }
}