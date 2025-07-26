#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    orgtype()
    {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

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
    Preprocess();
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128];
    std::vector<orgtype> typess[128];

    float time_unit_scale;
    int time_unit;
    int lidar_type;
    bool feature_enabled;
    double blind;
    int point_filter_num;
    double N_SCANS;


    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    private:
    void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    double vx, vy, vz;
    



};


#endif