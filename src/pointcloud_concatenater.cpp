#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>

class PointcloudConcatenater
{
protected:
    ros::NodeHandle nh_;
private:
    std::string filename_;
    std::string directory_;
    pcl::PCLPointCloud2 merged_cloud_; 
public:
    std::string cloud_topic_;
    ros::Subscriber sub_;

    void 
    cloud_cb (const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud)
    {
        if ((cloud->width * cloud->height) == 0)
            return;

        if (merged_cloud_.width * merged_cloud_.height == 0)
            merged_cloud_ = std::move(*cloud);
        else
            merged_cloud_.concatenate(merged_cloud_, *cloud);
        
        ROS_INFO("Recieved %d pointclouds, Total merged %d pointclouds",
                (int)cloud->width * cloud->height,
                (int)merged_cloud_.width * merged_cloud_.height);

    }

    PointcloudConcatenater()
        :directory_("/home/jay/")
    {
        ros::NodeHandle priv_nh("~");
        priv_nh.getParam("directory", directory_);
        if(!priv_nh.getParam("filename", filename_))
            filename_ = "test.pcd";
        if(!priv_nh.getParam("cloud_topic", cloud_topic_))
            cloud_topic_ = "velodyne_points";
        
        sub_ = nh_.subscribe(cloud_topic_, 1, &PointcloudConcatenater::cloud_cb, this);
        ROS_INFO ("Listening for incoming data on topic %s",
                nh_.resolveName (cloud_topic_).c_str ());
    };

    ~PointcloudConcatenater()
    {
        std::stringstream ss;
        pcl::PCDWriter writer;
        writer.writeASCII (directory_ + std::string(filename_), merged_cloud_);
        ROS_INFO("Wrote pcd file.");
    };

};

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_concatenater", 
                ros::init_options::AnonymousName);

    PointcloudConcatenater pc;
    ros::spin();

    return 0;
};