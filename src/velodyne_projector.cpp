// #include <ros/ros.h>
#include <velodyne_pcl/point_types.h>
// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
// #include <iostream>
#include <cmath>
#include "../include/velodyne_handler.h"

class velodyneProjector : public velodyneHandler
{
public:
    velodyneProjector();
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);

protected:
private:

};

velodyneProjector::velodyneProjector() 
    :velodyneHandler("velodyne_points", "velodyne_points_projected")
{
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &velodyneProjector::cloud_cb, this);   

    ROS_INFO("Start velodyne_projector node ...");
}

void velodyneProjector::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for ( auto& pt : pcl_cloud_ptr->points)
    {
        if (pt.z <= 1.5 && pt.z >= -0.5){
            pt.z = 0;
            new_pcl_cloud_ptr->push_back(velodyne_pcl::PointXYZIRT(pt));      
        }
    }


    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*new_pcl_cloud_ptr, out_cloud);
    out_cloud.header = in_cloud->header;

    pub_.publish(out_cloud);


};


int main(int argc, char** argv){
    ros::init(argc, argv, "velodyne_projector");
    velodyneProjector vp;
    ros::spin();

    return 0;
}