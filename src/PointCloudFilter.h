#include <iostream>
//ros includes
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
//point cloud includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/registration/icp.h>
//pcl filter includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PointCloudFilter
{
  public:
    PointCloudFilter(ros::NodeHandle& nh);
    ~PointCloudFilter(void);

    void callback(const PointCloud::ConstPtr& cloud);
  
  private:
    ros::NodeHandle nh_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new;
    
    ros::Publisher pub;
    ros::Subscriber sub;
    int counter;
    int saveCounter;
};
