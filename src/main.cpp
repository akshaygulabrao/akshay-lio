#include <omp.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#define MAXN                (720000)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

int scan_count = 0;
std::shared_ptr<Preprocess> p_pre(new Preprocess());

double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], \
s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], \
s_plot10[MAXN], s_plot11[MAXN];

double last_timestamp_lidar;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if(msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg,ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
    
}

int publish_count = 0;
double time_diff_lidar_to_imu = 0.0;
double timediff_lidar_wrt_imu = 0.0;
bool time_sync_en = false;
double last_timestamp_imu = -1.0;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    std::cout << "[imu_cbk] Received IMU data at time "
              << msg_in->header.stamp.toSec() << std::endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();

    if(timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);  

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
    
};

MeasureGroup Measures;

bool sync_packages(MeasureGroup &meas){
    if (lidar_buffer.empty() || imu_buffer.empty()){
        return false;
    }

    return true;
}





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
    ros::Subscriber sub_imu = nh.subscribe("/os_cloud_node/imu", 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}