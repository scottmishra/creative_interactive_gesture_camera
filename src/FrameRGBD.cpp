#include "FrameRGBD.h"

FrameRGBD::FrameRGBD()
{
  pointCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  downsampledPointCloudPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

FrameRGBD::~FrameRGBD(){}
/* Not needed if using pcl::registration::icp functionality
void copyGICPPointSetFromPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pclpointcloudptr,dgc::gicp::GICPPointSet& gicppointset)
{
    for(int i=0;i<pclpointcloudptr->size();i++)
    {
        dgc::gicp::GICPPoint point;
        point.x=pclpointcloudptr->points[i].x;
        point.y=pclpointcloudptr->points[i].y;
        point.z=pclpointcloudptr->points[i].z;
        gicppointset.AppendPoint(point);
    }
}*/

//Compute normal and covariance
void FrameRGBD::computeICPNormalMatrices(const double icp_epsilon)
{
    /*gicpPointSet = dgc::gicp::GICPPointSet();

    //Copy the downsampled point cloud to the GICP point cloud structure
    copyGICPPointSetFromPointCloudPtr(downsampledPointCloudPtr,gicpPointSet);

    // build kdtrees and normal matrices
    gicpPointSet.SetGICPEpsilon(gicp_epsilon);
    gicpPointSet.BuildKDTree();
    gicpPointSet.ComputeMatrices();*/
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
}
