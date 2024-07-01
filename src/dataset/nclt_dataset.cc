#include "nclt_dataset.h"

bool NCLTDataset::LoadPointCloud(const std::string& directory,
                                  sensor_msgs::PointCloud2& cloud) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  std::ifstream lidar_data_file;
  lidar_data_file.open(directory, std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Empty file read End..." << std::endl;
    return false;
  }

  constexpr float scaling = 0.005;  // 5 mm
  constexpr float offset = -100.0;

  while (!lidar_data_file.eof()) {
    uint16_t x, y, z;
    uint8_t intensity, label;

    lidar_data_file.read(reinterpret_cast<char*>(&x), sizeof(uint16_t));
    lidar_data_file.read(reinterpret_cast<char*>(&y), sizeof(uint16_t));
    lidar_data_file.read(reinterpret_cast<char*>(&z), sizeof(uint16_t));
    lidar_data_file.read(reinterpret_cast<char*>(&intensity), sizeof(uint8_t));
    lidar_data_file.read(reinterpret_cast<char*>(&label),
               sizeof(uint8_t));  // label byte, but we skip it

    pcl::PointXYZI point;
    point.x = static_cast<float>(x) * scaling + offset;
    point.y = static_cast<float>(y) * scaling + offset;
    // z value of nclt dataset is negative.
    point.z = -(static_cast<float>(z) * scaling + offset);
    point.intensity = static_cast<float>(intensity);

    pcl_cloud.push_back(point);
  }

  pcl::toROSMsg(pcl_cloud, cloud);
  return true;
}
