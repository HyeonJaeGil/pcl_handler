#include <velodyne_pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include "../include/velodyne_handler.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/voxel_grid.h>


class Velodyne2Laserscan : public VelodyneHandler
{
public:
    Velodyne2Laserscan(ros::NodeHandle* nodehandle);
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);
    double min_z_;
    double max_z_;
    double max_height;
    double min_height;
    double angle_min ; 
    double angle_max ;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    bool use_inf = true;
    double inf_epsilon = 1.0;

protected:
private:
    ros::NodeHandle nh_;

};

Velodyne2Laserscan::Velodyne2Laserscan(ros::NodeHandle* nodehandle)
    :nh_(*nodehandle)
{
    cloud_in_topic_ = "/velodyne_points";
    laser_out_topic_ = "/velo_scan";
    nh_.param<double>("min_z", min_z_, -0.4);
    nh_.param<double>("max_z", max_z_,  0.1);
    nh_.param<double>("max_height", max_height, 0.0);
    nh_.param<double>("min_height", min_height, -1.1);
    nh_.param<double>("angle_min" , angle_min, -3.141592);
    nh_.param<double>("angle_max" , angle_max, 3.141592); 
    nh_.param<double>("angle_increment", angle_increment, 0.003);
    nh_.param<double>("time_increment", time_increment, 0.0);
    nh_.param<double>("scan_time", scan_time, 0.1);
    nh_.param<double>("range_min", range_min, 0.4);
    nh_.param<double>("range_max", range_max, 100.0);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>(laser_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &Velodyne2Laserscan::cloud_cb, this);   

    ROS_INFO("Start velodyne_to_laserscan node ...");
}

void Velodyne2Laserscan::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr proj_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    // pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for ( auto& pt : pcl_cloud_ptr->points)
    {
        if (pt.z <= max_z_ && pt.z >= min_z_){
            pt.z = 0;
            proj_cloud_ptr->push_back(velodyne_pcl::PointXYZIRT(pt));      
        }
    }

    // pcl::VoxelGrid<velodyne_pcl::PointXYZIRT> voxelgrid;
    // voxelgrid.setInputCloud (proj_cloud_ptr);              
    // voxelgrid.setLeafSize (0.01f, 0.01f, 0.01f); //leaf size  
    // voxelgrid.filter (*new_new_pcl_cloud_ptr);          


    sensor_msgs::PointCloud2 projected_cloud;
    pcl::toROSMsg(*proj_cloud_ptr, projected_cloud);
    // projected_cloud.header = in_cloud->header;
    // projected_cloud.header.stamp = in_cloud->header.stamp;

    // pub_.publish(projected_cloud);


    // double max_height = 0.0;
    // double min_height = -1.1;
    // double angle_min = -3.14; 
    // double angle_max = 3.14;
    // double angle_increment = 0.003;
    // double scan_time= 0.1;
    // double range_min= 0.4;
    // double range_max = 100.0;
    // bool use_inf = true;
    // double inf_epsilon = 1.0;

    // build laserscan output
    sensor_msgs::LaserScan output;
    // output.header = projected_cloud.header;
    // output.header.stamp = projected_cloud.header.stamp;
    // output.header.frame_id = "velodyne";
    output.header = in_cloud->header;
    output.header.stamp = in_cloud->header.stamp;
    output.header.frame_id = in_cloud->header.frame_id;

    output.angle_min = angle_min;  
    output.angle_max = angle_max;
    output.angle_increment =angle_increment; 
    output.time_increment = time_increment;
    output.scan_time = scan_time; 
    output.range_min = range_min;   
    output.range_max = range_max;  

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    sensor_msgs::PointCloud2ConstPtr projected_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(projected_cloud);
    // sensor_msgs::PointCloud2Ptr cloud;


    // projected_pc2_ptr = in_cloud;
    // projected_pc2_ptr = &projected_cloud;


    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*projected_pc2_ptr, "x"), iter_y(*projected_pc2_ptr, "y"),
        iter_z(*projected_pc2_ptr, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
        ROS_INFO("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;
        }

        if (*iter_z > max_height || *iter_z < min_height)
        {
        ROS_INFO("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height, max_height);
        continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min)
        {
        ROS_INFO("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min, *iter_x,
                        *iter_y, *iter_z);
        continue;
        }
        if (range > range_max)
        {
        ROS_INFO("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max, *iter_x,
                        *iter_y, *iter_z);
        continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
        ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
        }

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - output.angle_min) / output.angle_increment;
        if (range < output.ranges[index])
        {
        output.ranges[index] = range;
        }
    }
    pub_.publish(output);

    };


int main(int argc, char** argv){
    ros::init(argc, argv, "velodyne_to_laserscan");
    ros::NodeHandle nh("~");
    Velodyne2Laserscan v2l(&nh);
    ros::spin();

    return 0;
}
