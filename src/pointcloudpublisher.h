#ifndef POINTCLOUDPUBLISHER_H
#define POINTCLOUDPUBLISHER_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <QMutex>
#include <QThread>

#include "dataset/dataset.h"

class PointCloudPublisher : public QThread {
  Q_OBJECT

 public:
  PointCloudPublisher(ros::NodeHandle nh, QObject* parent = nullptr);
  ~PointCloudPublisher();

  void run() override;
  void Stop();
  void Pause();
  void Resume();
  void EnableLooping();
  void DisableLooping();
  void JumpToIndex(int index);
  void SetFilePaths(const std::vector<std::string>& paths);
  void SetDataset(std::shared_ptr<Dataset> other);
  void SetFrequency(int freq);

 signals:
  void LogMessage(const QString& message);
  void ProgressUpdated(int value);
  void ReportPublishDone();

 private:
  ros::NodeHandle nh_;
  QMutex mutex_;
  bool is_running_;
  bool is_paused_;
  bool is_looping_;
  int curr_index_;
  std::vector<std::string> file_paths_;
  std::shared_ptr<Dataset> dataset_;

  std::string topic_name_point_cloud_;
  std::string frame_name_point_cloud_;
  ros::Publisher pub_point_cloud_;
  sensor_msgs::PointCloud2 point_cloud_msg_;
  ros::Rate timer;

  void PublishOnce();
};

#endif  // POINTCLOUDPUBLISHER_H
