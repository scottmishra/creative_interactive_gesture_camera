#include "cameraSync.h"


void cameraSync::imageCallback(const sensor_msgs::Image::ConstPtr&  msg)
{
  this->image_.header = msg->header;
  this->image_.height = msg->height;
  this->image_.width  = msg->width;
  this->image_.data   = msg->data;
}

void cameraSync::depthCallback(const sensor_msgs::PointCloud2::ConstPtr&  msg)
{
  this->cloud_.header = msg->header;
  this->cloud_.height = msg->height;
  this->cloud_.width  = msg->width;
  this->cloud_.data   = msg->data;
  
}

void cameraSync::syncData()
{
  if(this->cloud_.header.seq - this->image_.header.seq > 2)
  {
    ROS_DEBUG("Depth Cloud and Image are out os sync");
  }

}
