// #include <ros/ros.h>
#include <velodyne_pcl/point_types.h>
// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
// #include <iostream>
#include <cmath>
#include "../include/velodyne_handler.h"

class velodyneFilter : public velodyneHandler
{
public:
    velodyneFilter();
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);

protected:
private:

};

velodyneFilter::velodyneFilter() 
    :velodyneHandler("velodyne_points", "velodyne_points_filtered")
{
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &velodyneFilter::cloud_cb, this);   

    ROS_INFO("Start velodyne_filter node ...");
}

void velodyneFilter::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for (const auto& pt : pcl_cloud_ptr->points)
    {
        double distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        float angle = atan2(pt.y, pt.x);
        if(! ( angle >= DEG2RAD(140) || angle <= DEG2RAD(-140) ) || distance >= 1.0 || pt.z >= 0.3)
            new_pcl_cloud_ptr->push_back(velodyne_pcl::PointXYZIRT(pt));      
    }

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*new_pcl_cloud_ptr, out_cloud);
    out_cloud.header = in_cloud->header;

    pub_.publish(out_cloud);


};


int main(int argc, char** argv){
    ros::init(argc, argv, "velodyne_filter");
    velodyneFilter vf;
    ros::spin();

    return 0;
}