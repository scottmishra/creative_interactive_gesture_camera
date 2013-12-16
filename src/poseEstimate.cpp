#include <poseEstimate.h>

using namespace pcl;
using namespace message_filters;

const int LOOP_DETECTION_THRESHOLD = 40;
const int KEYFRAME_INLIERS_THRESHOLD = 100;

poseEstimate::poseEstimate(ros::NodeHandle& nh) : it_(nh_), sync(MySyncPolicy(10), image_sub, pc_sub)//, ci_sub
{
  //setup ros node handle
  nh_ = nh;
  globalMap_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  globalMapOpt_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_in_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_cluster_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_in_filtered_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_in_new_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_in_new_filtered_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  cloud_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  cloud_normals_new_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  
  cloud_features_ = FeatureCloudT::Ptr(new FeatureCloudT);
  cloud_features_new_ = FeatureCloudT::Ptr(new FeatureCloudT);
  
  siftResult_ = pcl::PointCloud<pcl::PointWithScale>::Ptr(new pcl::PointCloud<pcl::PointWithScale>);
  
  object_aligned_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
  //initialize downsampler for pointclouds
  downsampler = new PointCloudDownsampler(4);
  
  loopDetector = KeyframeLoopDetector(LOOP_DETECTION_THRESHOLD);
  
  //initialize feature matcher
  matcher = new VisualFeatureMatcher_Generic(cv::DescriptorMatcher::create( "BruteForce" ),"NoneFilter");
  
  //initialize RANSAC estimator
  transformationEstimator = new Visual3DRigidTransformationEstimator_RANSAC();
  
  //initialize icp pose refining object
  poseRefiner = new ICPPoseRefiner_PCL();
  
  counter = 0;
  pose = Eigen::Matrix4f::Identity ();
}


void poseEstimate::setup()
{
  pub_cloud_comb_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("global_map", 1);
  pub_cloud_cluster_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("clustered_points", 1);
  pub_cloud_sift_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("SIFT_points",1 );
  pub_test_ = nh_.advertise<std_msgs::Float32> ("Main2_Test",1);
  
  sub_cloud_in_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("points2", 1, &poseEstimate::pointCloudCallback, this);
 
  //Adding in syncing subscribers
  image_sub.subscribe(nh_, "grayscale_image", 1);
  pc_sub.subscribe(nh_, "point_cloud", 1);
  ci_sub.subscribe(nh_, "mapping_2d",1);
  
  camera_info = nh_.subscribe<sensor_msgs::CameraInfo>("camera_info",1,&poseEstimate::cameraInfoCallback, this);
   
  //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, pc_sub, ci_sub);//
  sync.registerCallback(boost::bind(&poseEstimate::pointCloudCallback3, this,  _1, _2));//, _3
}

void poseEstimate::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  if(K_.empty())
  {
    K_.create(3,3,CV_64F);
    K_.at<double>(0,0) = cam_info->K[0];
    K_.at<double>(0,1) = cam_info->K[1];
    K_.at<double>(0,2) = cam_info->K[2];
    K_.at<double>(1,0) = cam_info->K[3];
    K_.at<double>(1,1) = cam_info->K[4];
    K_.at<double>(1,2) = cam_info->K[5];
    K_.at<double>(2,0) = cam_info->K[6];
    K_.at<double>(2,1) = cam_info->K[7];
    K_.at<double>(2,2) = cam_info->K[8];
  }
  
}

void poseEstimate::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{  
}

void poseEstimate::pointCloudCallback2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
}

void poseEstimate::pointCloudCallback3(const sensor_msgs::ImageConstPtr& image, 
                                       const sensor_msgs::PointCloud2ConstPtr& cloudMsg//,
                                       //const creative_interactive_gesture_camera::Mapping2D::ConstPtr& mapping
                                       )//
{
  /*Want to add new frame to G2O graph object everytime the correspondence is within a certain threshold
   *This was we can move to graph optimization and loop closure
  */
   
  // pull out keypoints and descriptors
  cv::namedWindow("Correspondence",CV_WINDOW_AUTOSIZE);
  cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);

  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //set 2d feature detection parameters for each image.
  matcher->resHeight = image->height;
  matcher->resWidth = image->width;
  transformationEstimator->resHeight = cloudMsg->height;
  transformationEstimator->resWidth = cloudMsg->width;
  
  Eigen::Matrix4f keyframeRelativePose = Eigen::Matrix4f::Identity();
  //begin feature detection
  std::vector<cv::KeyPoint> currentKeypoints; //vector of keypoints
  cv::Mat currentDescriptors; //Matrix to hold descriptor locations
  std::vector<float> currentDescriptors_aux;
  //pull out keypoints and descriptors
  poseEstimate::getCurrentKeyPoints(cv_ptr->image,currentKeypoints,currentDescriptors,currentDescriptors_aux);
  ROS_INFO("currentDescriptors rows: %d, cols: %d", currentDescriptors.rows,currentDescriptors.cols);
  //filter down pointcloud 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloudMsg,*cloud); //convert pointcloud from ROS Msg to pcl data type
     
  //Get the last vertice added to the graph
  int graphSize = graph.getVertexIdx();
  
  if(graphSize > 0) //check that we have already added one frame
  {
    if(graphSize == 1)
      ROS_INFO("BEGIN MAIN LOOP");
    
    currentFrame = new FrameRGBD();
    *(currentFrame->pointCloudPtr) = *cloud;
    downsampler->downsamplePointCloud(currentFrame->pointCloudPtr, currentFrame->downsampledPointCloudPtr);
    currentFrame->timeStamp = cloud->header.stamp;
    currentFrame->intensityImage = cv_ptr->image;
    
    //Begin Feature matching
    std::vector<cv::DMatch> matches; //matched container
    matcher->match(loopDetector.descriptorsListPointer()->back(),currentDescriptors,matches);
    
    //Begin Outlier Removal
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<char> matchesMask;
    static int numberInliers;
    numberInliers = matcher->outlierRemovalFundamentalMat(loopDetector.keypointsListPointer()->back(),currentKeypoints,matches,matchesMask,3.0);
    
    //Get Robust 2D matched points
    matcher->get2DMatchedPoints(loopDetector.keypointsListPointer()->back(),currentKeypoints,matches,points1,points2,numberInliers,matchesMask);
    
    cv::Mat H2 = cv::findHomography(points1, points2, CV_RANSAC);
    
    //Using OpenCV to find the fundementalMatrix between the two images
    cv::Mat F = cv::findFundamentalMat(points1,points2, CV_FM_RANSAC, 0.1,0.99);
    //Using Fundemental Matrix to back out rotatino matrix
    while(K_.empty())
    {
      sleep(10);
    }
    
    cv::Mat E = K_.t()*F*K_; //finding essential matrix
    //Find Rotation using 2D image
    cv::SVD svd(E,cv::SVD::MODIFY_A);
    cv::Matx33d W(0,-1,0,   //HZ 9.13
                  1, 0,0,
                  0, 0,1);
    cv::Matx33d Winv( 0,1,0,
                     -1,0,0,
                      0,0,1);
    cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
    cv::Mat t = svd.u.col(2); //u3
    
    cv::Matx34d P1( R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2) );
                    
    std::cout << "Testing P1: " << std::endl << cv::Mat(P1) << std::endl;
    
    std::cout << "Testing H2: " << std::endl << cv::Mat(H2) << std::endl;
    
    cv::Mat drawImg;
    cv::drawMatches(loopDetector.keyframesPointer()->back()->intensityImage, loopDetector.keypointsListPointer()->back(), currentFrame->intensityImage, currentKeypoints,
                    matches, drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255),matchesMask);
    cv::putText(drawImg,"Press enter to stop grabbing frames",cv::Point(20,450),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);
    cv::imshow( "correspondences", drawImg );
    cv::waitKey(100);
    //Estimate the 3D rigid transformation between frame 1 and 2
    Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
    ROS_INFO("Size matching points1: %d, points2: %d", points1.size(), points2.size());
    ROS_INFO("RANSAC height: %d, width: %d", transformationEstimator->resHeight, transformationEstimator->resWidth);
    if(points1.size() != 0 && points2.size() != 0)
    {
      transformationEstimator->estimateVisual3DRigidTransformation(points1,points2,loopDetector.keyframesPointer()->back()->pointCloudPtr,currentFrame->pointCloudPtr,H);
    }
    else
    {
      ROS_INFO("NO GOOD POINTS SKIPPING FRAME");
      delete currentFrame;
      return;
    }  
    
    //Perform ICP to refine the pose approximation
    poseRefiner->refinePose(*loopDetector.keyframesPointer()->back(),*currentFrame,H); //H is the inverse of the relative pose
    ROS_INFO ("    |numberInliers: %d | \n",numberInliers);
    
    if(numberInliers<KEYFRAME_INLIERS_THRESHOLD)
    {
      //Update the current pose with the rigid 3D transformation estimated from the previous and
      //current keyframes
      Eigen::FullPivLU<Eigen::Matrix4f> lu(H);
      if(!lu.isInvertible())
      {
        ROS_INFO("H is not invertible, deleting frame and continuing");
        delete currentFrame;
        return;
      }
      
      keyframeRelativePose = H.inverse();//luOfH
      double translationNorm = std::sqrt(std::pow(keyframeRelativePose(0,3),2)+std::pow(keyframeRelativePose(1,3),2)+std::pow(keyframeRelativePose(2,3),2));
      if(keyframeRelativePose.norm() > 2.002|| translationNorm > 0.10)
      {
        ROS_INFO("H is too big, skipping frame");
        delete currentFrame;
        return;
      }
      
      pose = pose*keyframeRelativePose;
      //Detect loops
      ROS_INFO("Start Loop Detection");
      loopDetector.addKeyframe(currentFrame);
      loopDetector.addKeypoints(currentKeypoints);
      loopDetector.addDescriptors(currentDescriptors);
      loopDetector.accumulateRelativePose(keyframeRelativePose);
      loopDetector.detectLoop(*matcher,*transformationEstimator,
                              *poseRefiner,
                              relativePoses,
                              informationMatrices,
                              fromIndexes,toIndex);
      loopDetector.getCurrentPose(pose); //Update the current pose with the current keyframe pose
      ROS_INFO("Finished Loop Detection");
      //Accumulate non-optimized poses
      accPoses.push_back(pose);
      
      //Add the keyframe vertex to the graph
      graph.addVertex(pose);
      
      //Add the keyframe edges to the graph
      for(int i=0;i<relativePoses.size();i++)
      {
          graph.addEdge(fromIndexes[i],toIndex,relativePoses[i],informationMatrices[i]);
      }
      
      //Optimize the graph
      graph.optimizeGraph();
      
      //Set the optimized poses to the loop detector
      graph.getPoses(accOptimizedPoses);
      loopDetector.setPoses(accOptimizedPoses);
      //Update the optimized pose
      pose=accOptimizedPoses.back();
      ROS_INFO ("POSE SIZE: %d", pose.size());
      ROS_INFO ("    | %6.3f %6.3f %6.3f | \n", pose (0,0), pose (0,1), pose (0,2));
      ROS_INFO ("R = | %6.3f %6.3f %6.3f | \n", pose (1,0), pose (1,1), pose (1,2));
      ROS_INFO ("    | %6.3f %6.3f %6.3f | \n", pose (2,0), pose (2,1), pose (2,2));
      ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >\n", pose (0,3), pose (1,3), pose (2,3));

    }
    else
    {
      //Free the current frame
      delete currentFrame;
    }

  }
  else
  {
    //Adding first cloud
    *globalMap_ = *cloud;
    globalMap_->header.frame_id = cloud->header.frame_id;
    
    //set up initial poses for first frame
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f keyframeRelativePose = Eigen::Matrix4f::Identity();
    
    //store first frame in currentFrame object
    currentFrame = new FrameRGBD();
    
    //give frame the point cloud
    *(currentFrame->pointCloudPtr) = *cloud;
    currentFrame->intensityImage = cv_ptr->image;
    downsampler->downsamplePointCloud(currentFrame->pointCloudPtr, currentFrame->downsampledPointCloudPtr);
    currentFrame->timeStamp = cloud->header.stamp;
    
    //add initial frame to loopDetector object
    loopDetector.addKeyframe(currentFrame);
    loopDetector.addPose(pose);
    loopDetector.addKeypoints(currentKeypoints);
    loopDetector.addDescriptors(currentDescriptors);
    
    //Add first non-optimized pose
    accPoses.push_back(pose);
    
    //add first pose to g2o optimizer graph
    graph.addVertex(pose);
    pub_cloud_comb_.publish(globalMap_);
    ROS_INFO("Matcher Object Height: %d and Width: %d ",matcher->resHeight, matcher->resWidth);
    ROS_INFO("Captured 1st Image: Press Enter to continue "); 
    std::cin.get();
  }
  
  if(graphSize == 10)
    poseEstimate::computeOptimzedGraph();
  
    
}

void poseEstimate::computeOptimzedGraph()
{
  Miscellaneous::generateGlobalMapPtr(*loopDetector.keyframesPointer(),accPoses,globalMap_); 
  graph.getPoses(accOptimizedPoses);
  Miscellaneous::generateGlobalMapPtr(*loopDetector.keyframesPointer(),accOptimizedPoses,globalMapOpt_);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (globalMapOpt_);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.10, 2.0);
  pass.filter (*cloud_new);
  //removing noise, as we did before
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_new);
  sor.setMeanK (25);
  sor.setStddevMulThresh (0.75);
  sor.filter (*cloud_new);
  cloud_new->header = globalMap_->header;
  pub_cloud_comb_.publish(cloud_new);
  
  ROS_INFO("Dispalying Stitched Point Cloud: Press Enter to continue "); 
  std::cin.get();

}

void poseEstimate::computeSIFTKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src, pcl::PointCloud<pcl::PointXYZRGB> &siftResults )
{
  // Parameters for sift computation
  const float min_scale = 0.1f;
  const int n_octaves = 6;
  const int n_scales_per_octave = 10;
  const float min_contrast = 0.5f;
  
  // Estimate the sift interest points using Intensity values from RGB values
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> sift;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(cloud_src);
  sift.compute(siftResults);  
}

void poseEstimate::computeNARFKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,pcl::PointCloud<pcl::PointXYZRGB> &narfResults)
{
  // Keypoints are estimated where surface is stable and finds object borders
}

void poseEstimate::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src, pcl::PointCloud<pcl::Normal> &cloud_normal)
{
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ()); //KdTree that takes in PointXYZRGB clouds
  normalEstimator.setSearchMethod(tree); //sets the search method of the normal estimator to KdTree
  //use neighbors in a sphere of radius set in meters
  normalEstimator.setRadiusSearch(0.05); //5cm radius sphere
  normalEstimator.setInputCloud(cloud_src);
  normalEstimator.compute(cloud_normal);
}

void poseEstimate::estimateFPFH (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& src, 
                                 const pcl::PointCloud<pcl::Normal>::Ptr &normals_src,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                                 pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_src)
{
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (keypoints_src);
  fpfh_est.setInputNormals (normals_src);
  fpfh_est.setRadiusSearch (1); // 1m
  fpfh_est.setSearchSurface (src);
  fpfh_est.compute (fpfhs_src);
}

void poseEstimate::findCorrespondence(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
                                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
                                      pcl::Correspondences &all_correspondences)
{
  pcl::registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
  est.setInputSource (fpfhs_src);
  est.setInputTarget (fpfhs_tgt);
  est.determineReciprocalCorrespondences (all_correspondences);

}

void poseEstimate::rejectBadCorrespondence(const pcl::CorrespondencesPtr &all_correspondences,
                                           const PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
                                           const PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
                                           pcl::Correspondences &remaining_correspondences)    
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputCloud<pcl::PointXYZRGB> (keypoints_src);
  rej.setInputTarget<pcl::PointXYZRGB> (keypoints_tgt);
  rej.setMaximumDistance (1);    // 1m
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
}

void poseEstimate::computeICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src,
                              const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_tgt,
                              Eigen::Matrix4f &transform )
{
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; //create Iterative Closest Point Object
  PointCloud<pcl::PointXYZRGB> icpResults;
  icp.setInputSource(cloud_src); //Set cloud to that will be rotated
  icp.setInputTarget(cloud_tgt); //Set cloud that cloud_src should look like after rotatoni
  icp.align(icpResults); //Compute transformation of point clouds
  transform = icp.getFinalTransformation();
}                                 

void poseEstimate::computeTrianglation()
{
  // Concatenate the XYZRGB and normal fields*
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*cloud_in_filtered_, *cloud_normals_, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
  
  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);
  
  // Initialize triangulation objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;
  
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);
  
  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles); 
}

void poseEstimate::removeNoise(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_src, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_src);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloud_out);
}

void poseEstimate::computeClusterExtraction(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> vg; //VoxelGrid Filter to downsample cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>); //filtered cloud container
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>); //filtered cloud container
  
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered); //fill in cloud_filtered with downsampled cloud
  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;//seg object 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inlier points object shared ptr
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //model coefficient object shared ptr
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()); //cloud shared ptr object to hold planes
 
  seg.setOptimizeCoefficients (true); //we want to optimize coefficients
  seg.setModelType (pcl::SACMODEL_PLANE);//we want to look for planes
  seg.setMethodType (pcl::SAC_RANSAC); //use RANSAC method
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03); //this is the distance threshold, which determines which points are the inliers 

  int nr_points = (int) cloud_filtered->points.size (); // se
  while (cloud_filtered->points.size () > 0.3 * nr_points) // go through until under 30% of points remain
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients); //segment and store 
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (30000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_cluster->header.frame_id = cloud->header.frame_id;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    pub_cloud_cluster_.publish(cloud_cluster);
    std::cin.get();
    j++;
  }
  
}

void poseEstimate::getCurrentKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux)
{
  cv::ORB orb(256);
  orb(image,cv::Mat(),keypoints,descriptors);
  
}

void getCurrentFrameRGBD(FrameRGBD& frameRGBD)
{
}
