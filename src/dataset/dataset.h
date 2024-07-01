#ifndef DATASET_H
#define DATASET_H

#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>

class Dataset {
 public:
  virtual ~Dataset() = default;

  // 读取路径中的点云文件，返回是否读取成功
  virtual bool LoadPointCloud(const std::string& directory, sensor_msgs::PointCloud2& cloud) = 0;

  // 获取点云文件的后缀
  virtual std::string PointCloudSuffix() const = 0;
};

#endif  // DATASET_H
