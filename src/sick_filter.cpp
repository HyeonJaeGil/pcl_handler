#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"

class sickFilter
{
public:
    sickFilter();
    laser_geometry::LaserProjection projector;
    std::string scan_in_topic_;
    std::string cloud_out_topic_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    void scan_cb(const boost::shared_ptr<const sensor_msgs::LaserScan> in_cloud);

protected:
    ros::NodeHandle nh_;

private:

};

sickFilter::sickFilter() 
    :scan_in_topic_("scan"), cloud_out_topic_("sick_points")
{
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    sub_ = nh_.subscribe(scan_in_topic_, 1, &sickFilter::scan_cb, this);   

    ROS_INFO("Start sick_fliter node ...");
}

void sickFilter::scan_cb(const boost::shared_ptr<const sensor_msgs::LaserScan> in_scan)
{
    sensor_msgs::PointCloud2 ros_pointcloud2;
    projector.projectLaser(*in_scan, ros_pointcloud2,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    // ros_pointcloud2.header.frame_id = "map";

    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(ros_pointcloud2, pcl_pointcloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for (const auto& pt : pcl_cloud_ptr->points)
    {
        double distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        float angle = atan2(pt.y, pt.x);
        if(! ( angle >= DEG2RAD(135) || angle <= DEG2RAD(135) ) || distance >= 0.4)
            new_pcl_cloud_ptr->push_back(pcl::PointXYZI(pt));      
    }

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*new_pcl_cloud_ptr, out_cloud);
    out_cloud.header = in_scan->header;

    pub_.publish(out_cloud);
};


int main(int argc, char** argv){
    ros::init(argc, argv, "sick_fliter");
    sickFilter sf;
    ros::spin();

    return 0;
}