#include <stdio.h>
#include <vector>
#include <exception>

//ros include files
#include <ros/ros.h>

//image include files
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <DepthSense.hxx>

//Image class needed to use image_transport framework
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  
  public:
    ImageConverter():it_(nh_){};

    void imageCb();

    void addColorMapData();
};
