#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_last_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_last_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

bool sync_packages(MeasureGroup &meas){
    return true;
}


void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {}

MeasureGroup Measures;

int main(int argc, char **argv) {
    // registers this process as a ROS Node with the master
    ros::init(argc, argv, "talker");
    // creates a handle whose namespace is the node's private namespace
    ros::NodeHandle nh;
    // starts an internal thread pool so that subscription/service callbacks are executed in the bg
    ros::AsyncSpinner spinner(0);
      
    ros::Rate loop_rate(500);
    //standard_pcl_cbk is the callback function (not a variable) that will be invoked 
    //for every sensor_msgs/PointCloud2 message arriving on lid_topic
    ros::Subscriber sub_pcl = nh.subscribe("/os_cloud_node/points",200000, standard_pcl_cbk);

    while (ros::ok()) {
        if(!sync_packages(Measures)) break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}