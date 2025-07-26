#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess(): feature_enabled(0), lidar_type(OUST64), blind(0.01), point_filter_num(1)
{
  N_SCANS = 6;
  
}

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
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++){
      double range = pl_orig.points[i].x * pl_orig.points[i].x +
               pl_orig.points[i].y * pl_orig.points[i].y +
               pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind * blind) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      // 57.3 converts radians to degrees
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 180;
      if (yaw_angle <= -180.0) yaw_angle += 180;
      added_pt.curvature = pl_orig.points[i].t + time_unit_scale;

      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }

    }
      for (int j = 0; j < N_SCANS; j++){
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        std::vector<orgtype> &types = types[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++){
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i+1].x;
          vy = pl[i].y - pl[i+1].y;
          vz = pl[i].z - pl[i+1].z;
          types[i].dista = vx*vx + vy*vy + vz*vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl,types);
      }
    }   
  else
  {

  }
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