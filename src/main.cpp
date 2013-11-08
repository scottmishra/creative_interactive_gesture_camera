#include "Sensor.h"

void setupViewer(int argc, char* argv[]);

int main(int argc, char* argv[])
{
  //initialize ros
  ros::init (argc, argv, "depth_cam");
  ros::NodeHandle nh;
 
  ROS_INFO("STARTING SETUP of SENSOR CLASS");
  
  Sensor sensor(nh);
  sensor.Sensor::setup();
  
  ROS_INFO("FINISHED SETUP of SENSOR CLASS");
  
  
  
  return 0;
}
