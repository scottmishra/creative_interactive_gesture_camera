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

using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZRGB> cloudRGB_;
ros::Publisher pub_cloudRGB;

void syncData2(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
  cloudRGB_.header.frame_id = "/base_link";
  if(input_cloud->height != image->height || input_cloud->width != image->width)
  {
    ROS_DEBUG("Image and PointCloud are not the same size");
  }
  
  if(input_cloud->header.seq != image->header.seq)
  {
    ROS_DEBUG("Image and PointCloud are not at the same time");
  }
  int h = input_cloud->height;
  int w = input_cloud->width;
  int count_pcl = 0;
  int count_rgb = 0;
  uint32_t rgb;
  uint8_t r, g, b;
  cloudRGB_.height = h;
  cloudRGB_.width = w;
  cloudRGB_.is_dense = true;
  cloudRGB_.points.resize(w*h); 

  pcl::fromROSMsg(*input_cloud,cloudRGB_);

  for(int i = 0;i < h;i++){
    for(int j = 0;j < w; j++){
      //cloudRGB_.points[count_pcl].x = input_cloud->data[count_rgb];
      //cloudRGB_.points[count_pcl].y = input_cloud->data[count_rgb+1];
	 	  //cloudRGB_.points[count_pcl].z = input_cloud->data[count_rgb+2];
	 	  r = image->data[count_rgb];
	 	  g = image->data[count_rgb+1];
	 	  b = image->data[count_rgb+2];
	 	  rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	 	  cloudRGB_.points[count_pcl].rgb = *reinterpret_cast<float*>(&rgb);
	 	  count_rgb += 3;
	 	  //count_pcl++;
	  }
  }
    
  pub_cloudRGB.publish(cloudRGB_);
}
/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
    //initialize ros
    ros::init (argc, argv, "sync_depth");
    ros::NodeHandle nh;
    
    //initialize image transport object
    image_transport::ImageTransport it(nh);
    
    //create cameraSync object relating to current ros handle
    //cameraSync cs(nh);
    //Adding in syncing subscribers
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "rgb_data", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "points2", 1);
    
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(4), image_sub, pc_sub);
    sync.registerCallback(boost::bind(&syncData2, _1, _2));
    //end syncing subscriber
    
    //initialize publishers
    pub_cloudRGB = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("pointsRGB", 1);
          
    //loop while ros core is operational or Ctrl-C is used
    while(ros::ok()){
      ros::spin();
    }
    
    return 0;
}
