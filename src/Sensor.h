#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <fstream>

//ros include files
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <DepthSense.hxx>

using namespace std;
using namespace cv;
using namespace DepthSense;

class Sensor
{
public:
  Sensor(ros::NodeHandle& nh);
  ~Sensor(void);

  void setup();

private:
  void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data);
  void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data);
  void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data);
  void configureAudioNode();
  void configureColorNode();
  void configureDepthNode();
  void configureNode(Node node);
  void onNodeConnected(Device device, Device::NodeAddedData data);
  void onNodeDisconnected(Device device, Device::NodeRemovedData data);

  void onDeviceConnected(Context context, Context::DeviceAddedData data);
  void onDeviceDisconnected(Context context, Context::DeviceRemovedData data);
  
  void spin();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_image;
  image_transport::Publisher pub_cloud_image;
  
  ros::Publisher pub_cloud;
  ros::Publisher pub_uv;
  ros::Publisher pub_camera_info;
  
  sensor_msgs::Image cloud_image;
  sensor_msgs::Image image;
  sensor_msgs::CameraInfo camera_info;
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_uv;
        
  Context device_context;
  DepthNode d_node;
  ColorNode c_node;
  AudioNode a_node;

  bool device_found;
  ProjectionHelper *proj_helper;
  StereoCameraParameters stereo_param;
        
  cv::Mat color_img;
  cv::Mat depth_img;
  
  uint32_t g_aFrames;
  uint32_t g_cFrames;
  uint32_t g_dFrames;
};


