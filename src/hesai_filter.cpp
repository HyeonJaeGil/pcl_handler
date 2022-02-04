#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <cmath>
#include "../include/hesai_handler.h"


class HesaiFilter : public HesaiHandler
{
public:
    double deg_limit_;
    double backward_min_distance_;
    double backward_min_height_;
    HesaiFilter();
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);



protected:
private:
    ros::NodeHandle nh_;

};

HesaiFilter::HesaiFilter() 
{
    // nh_.getParam("cloud_in_topic", cloud_in_topic_);
    // nh_.getParam("cloud_out_topic", cloud_out_topic_);
    cloud_in_topic_ = "/hesai/pandar";
    cloud_out_topic_= "/hesai/pandar/filtered";
    // nh_.param<std::string>("cloud_in_topic", cloud_in_topic_, "velodyne_points");
    // nh_.param<std::string>("cloud_out_topic", cloud_out_topic_, "velodyne_points_filtered");
    nh_.param<double>("deg_limit", deg_limit_, 100.0);
    nh_.param<double>("backward_min_distance", backward_min_distance_, 1.0);
    nh_.param<double>("backward_min_height", backward_min_height_, 0.3);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 1);
    sub_ = nh_.subscribe(cloud_in_topic_, 1, &HesaiFilter::cloud_cb, this);   

    ROS_INFO("Start hesai_filter node ...");
}

void HesaiFilter::cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud)
{
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*in_cloud, pcl_pointcloud2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);

    for (const auto& pt : pcl_cloud_ptr->points)
    {
        double distance = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        double angle = atan2(pt.y, pt.x);
        // if(! ( angle >= DEG2RAD(deg_limit_) || angle <= DEG2RAD(-1*deg_limit_) ) || distance >= backward_min_distance_ || pt.z >= backward_min_height_)
        if(! ( angle >= DEG2RAD(deg_limit_)) && distance > 1.0 )
            new_pcl_cloud_ptr->push_back(pcl::PointXYZI(pt));      
    }

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*new_pcl_cloud_ptr, out_cloud);
    out_cloud.header = in_cloud->header;

    pub_.publish(out_cloud);


};


int main(int argc, char** argv){
    ros::init(argc, argv, "hesai_filter");
    HesaiFilter hf;
    ros::spin();

    return 0;
}