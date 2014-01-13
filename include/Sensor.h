#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <fstream>

//ros include files
#include <ros/ros.h>
#include <std_msgs/Int32.h> //used for debugging purposes
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <creative_interactive_gesture_camera/Mapping2D.h> //Custom message for passing corresponding 2D points for each 3D point
#include <creative_interactive_gesture_camera/Point2D.h>//Custom message for passing corresponding 2D points for each 3D point
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
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
  void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data); //Called when new Audio Sample occurs, set at some Freq
  void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data); //Called when camera registers a new Color Image, set to some Freq
  void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data); //Called when camera registeres a new Depth Image, set to some Freq
  void configureAudioNode(); 
  void configureColorNode(); //Used to set up camera parameters, ie sample rate, resolution
  void configureDepthNode(); //Used to set up camera parameters, ie sample rate, resolution
  void configureNode(Node node); //Used to Camera environment
  void onNodeConnected(Device device, Device::NodeAddedData data); //Handles camera connection to pc
  void onNodeDisconnected(Device device, Device::NodeRemovedData data); //Handles camera shutdown

  void onDeviceConnected(Context context, Context::DeviceAddedData data);
  void onDeviceDisconnected(Context context, Context::DeviceRemovedData data);
  
  void spin();

private:
  ros::NodeHandle nh_; //used to talk to ROS MASTER
  image_transport::ImageTransport it_; //ROS prefered method of transmitting image data
  
  image_transport::Publisher pub_image; //publishes full image information
  image_transport::Publisher pub_image_mono; //publishes greyscale image
  image_transport::Publisher pub_cloud_image; //publishes depth image
  
  ros::Publisher pub_cloud; //publishes point cloud in XYZRGB format
  ros::Publisher pub_cloud_xyz; //publishes point cloud in XYZ format
  ros::Publisher pub_camera_info; //publishes depth camera information
  ros::Publisher pub_camera_mono_info; 
  ros::Publisher pub_test; //used for debugging
  ros::Publisher pub_2d_3d_mapping; //publishes 2d pixel location for each 3d pointcloud pixel
  
  sensor_msgs::Image cloud_image; //image containers
  sensor_msgs::Image image; 
  sensor_msgs::Image image_mono;
  
  sensor_msgs::CameraInfo camera_info;  //Camera info container
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud; //PointCloud containers
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  
  Eigen::Matrix4f transform; //used to rotate cloud into proper reference frame
  
  creative_interactive_gesture_camera::Point2D mappingPoint; //hold x,y point data as float32
  creative_interactive_gesture_camera::Mapping2D mapping; //3d-2d mapping
        
  Context device_context; //Nodes to allow access to their respective camera component 
  DepthNode d_node;
  ColorNode c_node;
  AudioNode a_node;

  bool device_found;
  ProjectionHelper *proj_helper;
  StereoCameraParameters stereo_param;
        
  cv::Mat color_img; //CV image containers
  cv::Mat mono_img;
  cv::Mat depth_img;
  
  uint32_t g_aFrames;
  uint32_t g_cFrames;
  uint32_t g_dFrames;
};


