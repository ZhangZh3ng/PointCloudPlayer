#ifndef NCLTDATASET_H
#define NCLTDATASET_H

#include <fstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "dataset.h"

class NCLTDataset : public Dataset {
 public:
  bool LoadPointCloud(const std::string& directory, sensor_msgs::PointCloud2& cloud) override;
  std::string PointCloudSuffix() const override { return ".bin"; }
};

#endif  // NCLTDATASET_H
