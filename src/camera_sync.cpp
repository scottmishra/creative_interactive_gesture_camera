#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <fstream>

//ros include files
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudToImage
{
public:

  PointCloudToImage(ros::NodeHandle& nh): it_(nh_)
  {
    nh_ = nh;
  }

  void setUp() 
  {
    image_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
    image_pub_ = it_.advertise("points2_image", 1);
  }
  
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud,const sensor_msgs::ImageConstPtr& image )
  { 
    if ((input_cloud->width * input_cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*input_cloud, *image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }   
    image_->header.frame_id = image->header.frame_id;
    image_->header.stamp = image->header.stamp;
    image_pub_.publish (image_); //publish our cloud image
  } 
  
private:

  ros::NodeHandle nh_; //node handle object
  sensor_msgs::Image::Ptr image_; //cache the image message
 
  image_transport::ImageTransport it_; //image_transport object
  image_transport::Publisher image_pub_; //image message publisher
};

PointCloudToImage* pc2i;

void syncTime(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& input_cloud, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  ROS_INFO("INSIDE syncData2 callback");
  
  pc2i->cloud_cb(input_cloud,image);
  
}
/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  //initialize ros
  ros::init (argc, argv, "find_keypoints");
  ros::NodeHandle nh;
  
  pc2i =  new PointCloudToImage(nh);
  pc2i->setUp();
  //initialize image transport object
  image_transport::ImageTransport it(nh);
 
  //Adding in syncing subscribers
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "rgb_image", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "point_cloud", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> ci_sub(nh, "camera_info",1);
  
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, pc_sub, ci_sub);
  sync.registerCallback(boost::bind(&syncTime, _1, _2, _3));
  
  //loop while ros core is operational or Ctrl-C is used
  while(ros::ok()){
    ros::spin();
  }
    
  return 0;
}
