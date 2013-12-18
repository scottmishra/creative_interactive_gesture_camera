//ros include files
//#include <mrpt/utils/CTimeLogger.h> //Profiling
#include<time.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <creative_interactive_gesture_camera/Mapping2D.h> //Custom message for passing corresponding 2D points for each 3D point
#include <creative_interactive_gesture_camera/Point2D.h>//Custom message for passing corresponding 2D points for each 3D point
#include <image_transport/image_transport.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/LU>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/gp3.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/keypoints/narf_keypoint.h>

#include <g2o/core/linear_solver.h>

#include "GraphOptimizer_G2O.h"
#include "FrameRGBD.h"

#include "VisualFeatureMatcher_Generic.h"
//Using ICP from PCL
#include "ICPPoseRefiner_PCL.h"
//Using RANSAC for 3D transformation estimation
#include "Visual3DRigidTransformationEstimator_RANSAC.h"

#include "KeyframeLoopDetector.h"
#include "Miscellaneous.h"
#include "PointCloudDownsampler.h"

// Types
typedef pcl::PointNormal PointNT; //representing normal coordinates and the surface curvature estimate
typedef pcl::PointCloud<PointNT> PointCloudNT;
//typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PFHSignature125 FeatureT;
//typedef pcl::FPFHEstimationOMP<pcl::PointXYZRGB,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

class poseEstimate
{
  public:
  //constructor used to initialize the node handle
  poseEstimate(ros::NodeHandle& nh);
  ~poseEstimate(void){};
  //setup function will initialize all the boost shared pointers along with the 
  // publishers and subscribers
  void setup();
  
  private:
  //callback function that is called with sub_cloud_in_ recieves a pointcloud msg
  void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
  
  //testing different callback functions
  void pointCloudCallback2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
  
  //pulling in pointcloud, full image, and mapping informaiton
  void pointCloudCallback3(const sensor_msgs::ImageConstPtr& image, 
                           const sensor_msgs::PointCloud2ConstPtr& cloudMsg//,
                           //const creative_interactive_gesture_camera::Mapping2D::ConstPtr& mapping
                           );
                           
  //pulling in camera sensor information, populating K matrix and distortion matrix, etc
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);
  //publish out an optimized graph point cloud                          
  void computeOptimzedGraph();
  
  //compute cluster extractino from pointclouds
  void computeClusterExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
  
  //compute SIFT keypoints
  void computeSIFTKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,pcl::PointCloud<pcl::PointXYZRGB> &siftResults );
  
  //compute NARF Keypoints (Normal Aligned Radial Features)
  void computeNARFKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,pcl::PointCloud<pcl::PointXYZRGB> &narfResults);
  
  //compute Cloud Normals
  void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,pcl::PointCloud<pcl::Normal> &cloud_normal);
  
  //compute trianglation
  void computeTrianglation();
  
  //reduce noise on the pointcloud
  void removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
                           
  //find fast pfh between two frames
  void estimateFPFH (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& src,
                     const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                     pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src);
                    
  void findCorrespondence(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                          const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                          pcl::Correspondences &all_correspondences);
                          
  void rejectBadCorrespondence(const pcl::CorrespondencesPtr &all_correspondences,
                                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
                                             pcl::Correspondences &remaining_correspondences);
                                             
  void computeICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,
                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_tgt,
                  Eigen::Matrix4f &transform);
                  
  void getCurrentKeyPoints(const cv::Mat& image, 
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::Mat& descriptors,
                           std::vector<float>& descriptors_aux);
                  
  void getCurrentFrameRGBD(FrameRGBD& frameRGBD); 
  
  void Mat2Quat(cv::Mat Mat, cv::Mat& Quat);    
  
                                 
  //Determine relation between src and tgt point clouds
  /*void findCorrespondences (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud &src,
                            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud &tgt, 
                            pcl::registration::Correspondences &all_correspondences);
  void rejectBadCorrespondences (const pcl::registration::CorrespondencesPtr &all_correspondences,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& &src,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& &tgt,
                                 pcl::registration::Correspondences &remaining_correspondences);*/
  
  //ros node handle object
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_; //ROS prefered method of transmitting image data
  
  ros::Publisher pub_cloud_comb_;
  ros::Publisher pub_cloud_cluster_;
  ros::Publisher pub_cloud_sift_;
  ros::Publisher pub_test_;
  
  ros::Subscriber sub_cloud_in_;
  ros::Subscriber camera_info;
  
  //Set Up Synchronizing objects
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2 > pc_sub;
  message_filters::Subscriber<creative_interactive_gesture_camera::Mapping2D> ci_sub;
  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;// , creative_interactive_gesture_camera::Mapping2D
  
  message_filters::Synchronizer<MySyncPolicy> sync;
  
  // Point clouds XYZRGB objects
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMapOpt_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_filtered_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_new_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_new_filtered_;
  
  //will publish this point cloud XYZRGB object out to ros
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_aligned_;
  
  //pointcloud normal objects, hold the estimated normals of the points in the cloud
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_new_;
  
  //Feature objects used to hold computed features for scene
  FeatureCloudT::Ptr cloud_features_; 
  FeatureCloudT::Ptr cloud_features_new_; 
  pcl::PointCloud<pcl::PointWithScale>::Ptr siftResult_;
  
  //LUM Graph Manager Object
  pcl::registration::LUM<pcl::PointXYZRGB>::Ptr lum_;
  //Explict Loop Closing Heurisitc
  pcl::registration::ELCH<pcl::PointXYZRGB>::Ptr eclh_;
  
  //GraphOptimizer Object
  GraphOptimizer_G2O graph;
  
  //Create an FrameRGBD object ptr
  FrameRGBD* currentFrame;
  
  //Create pointcloud downsampling object ptr
  PointCloudDownsampler* downsampler;
  
  //Create Loop Detector object
  KeyframeLoopDetector loopDetector;
  
  //Create Feater Matcher Object pointer
  VisualFeatureMatcher_Generic* matcher;
  
  //Visual 3D rigid transformation estimator
  Visual3DRigidTransformationEstimator_RANSAC* transformationEstimator;
  
  //ICP Pose Refiner
  ICPPoseRefiner_PCL* poseRefiner;
     
  //cv::Mat to hold current descriptors
  cv::Mat currentDescriptors;
  
  //cv::Mat to hold the K matrix
  cv::Mat K_;
  
  //pose vectors containers
  Eigen::Matrix4f pose;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > relativePoses;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > accPoses;
  std::vector<Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6 > > > informationMatrices;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > accOptimizedPoses;
  
  //used to determine number of frames 
  int counter;
  
  std::vector<int> fromIndexes;
  int toIndex;
};
