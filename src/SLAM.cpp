#include <poseEstimate.h>


void setupViewer(int argc, char* argv[]);

int main(int argc, char* argv[])
{
  //initialize ros
  ros::init (argc, argv, "pose_estimate");
  ros::NodeHandle nh;
  
  ROS_INFO("STARTING SETUP of poseEstimate CLASS");
  poseEstimate pose(nh);
  pose.poseEstimate::setup();
  ROS_INFO("FINISHED SETUP of poseEstimate CLASS");
  
  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
