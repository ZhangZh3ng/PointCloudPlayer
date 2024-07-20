#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "wildplace_dataset.h"

bool WildPlaceDataset::LoadPointCloud(const std::string& directory,
                                      sensor_msgs::PointCloud2& cloud) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_xyz;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud_xyzi;

  // 读取.pcd文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(directory, pcl_cloud_xyz) == -1) {
    PCL_ERROR("Couldn't read file %s \n", directory.c_str());
    return false;
  }

  // Wild Place Dataset 的点云不带intensity, 因此设置intensity为0
  pcl_cloud_xyzi.points.resize(pcl_cloud_xyz.points.size());
  for (size_t i = 0; i < pcl_cloud_xyz.points.size(); ++i) {
    pcl_cloud_xyzi.points[i].x = pcl_cloud_xyz.points[i].x;
    pcl_cloud_xyzi.points[i].y = pcl_cloud_xyz.points[i].y;
    pcl_cloud_xyzi.points[i].z = pcl_cloud_xyz.points[i].z;
    pcl_cloud_xyzi.points[i].intensity = 0.0; 
  }

  pcl_cloud_xyzi.width = pcl_cloud_xyz.width;
  pcl_cloud_xyzi.height = pcl_cloud_xyz.height;
  pcl_cloud_xyzi.is_dense = pcl_cloud_xyz.is_dense;
  pcl::toROSMsg(pcl_cloud_xyzi, cloud);
  
  return true;
}