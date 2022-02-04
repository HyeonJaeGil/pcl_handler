#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>


class HesaiHandler
{
public:
    HesaiHandler(){};
    HesaiHandler(std::string in_topic, std::string out_topic)
        :cloud_in_topic_(in_topic), cloud_out_topic_(out_topic) {};
    // virtual ~HesaiHandler();
    std::string cloud_in_topic_;
    std::string cloud_out_topic_;
    std::string laser_out_topic_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> in_cloud);

protected:
    // ros::NodeHandle nh_;

private:

};

