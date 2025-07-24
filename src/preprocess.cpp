#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

}

void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

}

void sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

}


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

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case MARSIM:
    sim_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;

}