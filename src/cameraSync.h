//ros include files
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//Image class needed to use image_transport framework
class cameraSync
{
  ros::NodeHandle nh_;
  sensor_msgs::Image image_;
  sensor_msgs::PointCloud2 cloud_;
  
  public:
    cameraSync(){}

    void imageCallback(const sensor_msgs::Image::ConstPtr& image);

    void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    void syncData();
};
