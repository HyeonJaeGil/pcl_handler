#include <velodyne_pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include "../include/velodyne_handler.h"

class VelodyneProjector : public VelodyneHandler
{
public:
    VelodyneProjector();
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);
    double min_z_;
    double max_z_;

protected:
private:

};

VelodyneProjector::VelodyneProjector() 
{
    nh_.param<std::string>("cloud_in_topic_", cloud_in_topic_, "velodyne_points");
    nh_.param<std::string>("cloud_out_topic_", cloud_out_topic_, "velodyne_points_projected");
    nh_.param<double>("min_z", min_z_, -0.8);
    nh_.param<double>("max_z", max_z_,  2.0);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &VelodyneProjector::cloud_cb, this);   

    ROS_INFO("Start velodyne_projector node ...");
}

void VelodyneProjector::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for ( auto& pt : pcl_cloud_ptr->points)
    {
        if (pt.z <= max_z_ && pt.z >= min_z_){
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
    VelodyneProjector vp;
    ros::spin();

    return 0;
}
