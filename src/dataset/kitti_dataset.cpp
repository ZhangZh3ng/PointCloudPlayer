#include "kitti_dataset.h"

bool KittiDataset::LoadPointCloud(const std::string& directory,
                                  sensor_msgs::PointCloud2& cloud) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  std::ifstream lidar_data_file;
  lidar_data_file.open(directory, std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Empty file read End..." << std::endl;
    return false;
  }

  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));

  pcl_cloud.clear();
  pcl::PointXYZI point;
  for (std::size_t i = 0; i < lidar_data_buffer.size(); i += 4) {
    point.x = lidar_data_buffer[i];
    point.y = lidar_data_buffer[i + 1];
    point.z = lidar_data_buffer[i + 2];
    point.intensity = lidar_data_buffer[i + 3];
    pcl_cloud.push_back(point);
  }

  pcl::toROSMsg(pcl_cloud, cloud);
  return true;
}
