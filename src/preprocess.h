#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM};

namespace ouster_ros{
    struct EIGEN_ALIGN16 Point{
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
};

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint8_t, range, range)
)

class Preprocess
{
    public:
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    
    float time_unit_scale;
    int time_unit;
    int lidar_type;



    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    private:
    void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);



};


#endif