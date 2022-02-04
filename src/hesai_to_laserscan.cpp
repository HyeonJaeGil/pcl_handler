#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include "../include/hesai_handler.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/voxel_grid.h>


class Hesai2Laserscan : public HesaiHandler
{
public:
    Hesai2Laserscan(ros::NodeHandle* nodehandle);
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

Hesai2Laserscan::Hesai2Laserscan(ros::NodeHandle* nodehandle)
    :nh_(*nodehandle)
{
    cloud_in_topic_ = "/hesai/pandar";
    laser_out_topic_ = "/hesai_scan";
    nh_.param<double>("min_z", min_z_, -1.5);
    nh_.param<double>("max_z", max_z_,  0.0);
    nh_.param<double>("max_height", max_height, 0.0);
    nh_.param<double>("min_height", min_height, -1.1);
    nh_.param<double>("angle_min" , angle_min, -3.141592);
    nh_.param<double>("angle_max" , angle_max, 3.141592); 
    nh_.param<double>("angle_increment", angle_increment, 0.003);
    nh_.param<double>("time_increment", time_increment, 0.0);
    nh_.param<double>("scan_time", scan_time, 0.1);
    nh_.param<double>("range_min", range_min, 0.5);
    nh_.param<double>("range_max", range_max, 50.0);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>(laser_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &Hesai2Laserscan::cloud_cb, this);   

    ROS_INFO("Start Hesai_to_laserscan node ...");
}

void Hesai2Laserscan::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr proj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_proj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for ( auto& pt : pcl_cloud_ptr->points)
    {
        double distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if (distance > 1.0 && pt.z <= max_z_ && pt.z >= min_z_){
            pt.z = 0;
            proj_cloud_ptr->push_back(pcl::PointXYZI(pt));      
        }
    }

    pcl::PCLPointCloud2 proj_pointcloud2, proj_pointcloud2_filtered;
    pcl::toPCLPointCloud2(*proj_cloud_ptr, proj_pointcloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelgrid;
    voxelgrid.setInputCloud(boost::make_shared<pcl::PCLPointCloud2>(proj_pointcloud2));
    voxelgrid.setLeafSize (0.05f, 0.05f, 0.05f); //leaf size  
    voxelgrid.filter (proj_pointcloud2_filtered);          
    pcl::fromPCLPointCloud2(proj_pointcloud2_filtered, *new_proj_cloud_ptr);

    sensor_msgs::PointCloud2 projected_cloud;
    pcl::toROSMsg(*new_proj_cloud_ptr, projected_cloud);


    // build laserscan output
    sensor_msgs::LaserScan output;
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
    uint32_t ranges_size 
        = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    sensor_msgs::PointCloud2ConstPtr projected_pc2_ptr 
        = boost::make_shared<sensor_msgs::PointCloud2>(projected_cloud);

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> 
        iter_x(*projected_pc2_ptr, "x"), 
        iter_y(*projected_pc2_ptr, "y"),
        iter_z(*projected_pc2_ptr, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
        ROS_INFO("rejected for nan in point(%f, %f, %f)\n", 
                    *iter_x, *iter_y, *iter_z);
        continue;
        }

        if (*iter_z > max_height || *iter_z < min_height)
        {
        ROS_INFO("rejected for height %f not in range (%f, %f)\n", 
                    *iter_z, min_height, max_height);
        continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min)
        {
        ROS_INFO("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", 
                    range, range_min, *iter_x, *iter_y, *iter_z);
        continue;
        }
        if (range > range_max)
        {
        ROS_INFO("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", 
                    range, range_max, *iter_x, *iter_y, *iter_z);
        continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
        ROS_INFO("rejected for angle %f not in range (%f, %f)\n", 
                    angle, output.angle_min, output.angle_max);
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
    ros::init(argc, argv, "Hesai_to_laserscan");
    ros::NodeHandle nh("~");
    Hesai2Laserscan v2l(&nh);
    ros::spin();

    return 0;
}
