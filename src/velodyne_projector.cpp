#include <velodyne_pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include "../include/velodyne_handler.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/voxel_grid.h>


class VelodyneProjector : public VelodyneHandler
{
public:
    VelodyneProjector();
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);
    double min_z_;
    double max_z_;

protected:
private:
    ros::NodeHandle nh_;

};

VelodyneProjector::VelodyneProjector() 
{
    cloud_in_topic_ = "velodyne_points";
    cloud_out_topic_= "velodyne_points_projected";
    nh_.param<double>("min_z", min_z_, -0.4);
    nh_.param<double>("max_z", max_z_,  0.1);
    // pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("velo_scan", 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &VelodyneProjector::cloud_cb, this);   

    ROS_INFO("Start velodyne_projector node ...");
}

void VelodyneProjector::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for ( auto& pt : pcl_cloud_ptr->points)
    {
        if (pt.z <= max_z_ && pt.z >= min_z_){
            pt.z = 0;
            new_pcl_cloud_ptr->push_back(velodyne_pcl::PointXYZIRT(pt));      
        }
    }

    // pcl::VoxelGrid<velodyne_pcl::PointXYZIRT> voxelgrid;
    // voxelgrid.setInputCloud (new_pcl_cloud_ptr);              
    // voxelgrid.setLeafSize (0.01f, 0.01f, 0.01f); //leaf size  
    // voxelgrid.filter (*new_new_pcl_cloud_ptr);          


    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*new_pcl_cloud_ptr, out_cloud);
    out_cloud.header = in_cloud->header;
    out_cloud.header.stamp = in_cloud->header.stamp;

    // pub_.publish(out_cloud);


    double max_height = 0.0;
    double min_height = -1.1;
    double angle_min = -3.14; 
    double angle_max = 3.14;
    double angle_increment = 0.003;
    double scan_time= 0.1;
    double range_min= 0.4;
    double range_max = 100.0;
    bool use_inf = true;
    double inf_epsilon = 1.0;

    // build laserscan output
    sensor_msgs::LaserScan output;
    output.header = out_cloud.header;
    output.header.stamp = out_cloud.header.stamp;
    output.header.frame_id = "velodyne";

    output.angle_min = angle_min;  
    output.angle_max = angle_max;
    output.angle_increment =angle_increment; 
    output.time_increment = 0.0;
    output.scan_time = scan_time; 
    output.range_min = range_min;   
    output.range_max = range_max;  

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    sensor_msgs::PointCloud2ConstPtr cloud_out = boost::make_shared<sensor_msgs::PointCloud2>(out_cloud);
    sensor_msgs::PointCloud2Ptr cloud;


    // cloud_out = in_cloud;
    // cloud_out = &out_cloud;


    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
        iter_z(*cloud_out, "z");
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
    ros::init(argc, argv, "velodyne_projector");
    VelodyneProjector vp;
    ros::spin();

    return 0;
}
