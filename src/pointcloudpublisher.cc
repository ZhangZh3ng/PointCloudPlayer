#include "pointcloudpublisher.h"
#include <QDebug>

PointCloudPublisher::PointCloudPublisher(ros::NodeHandle nh, QObject* parent)
    : nh_(nh),
      QThread(parent),
      is_running_(false),
      is_paused_(false),
      is_looping_(false),
      curr_index_(0),
      timer(10) {

  nh.param<std::string>("topic_name_point_cloud", topic_name_point_cloud_,
                        "point_cloud");
  nh.param<std::string>("frame_name_point_cloud", frame_name_point_cloud_,
                        "lidar");
  point_cloud_msg_.header.frame_id = frame_name_point_cloud_;

  pub_point_cloud_ =
      nh.advertise<sensor_msgs::PointCloud2>(topic_name_point_cloud_, 100);
}

PointCloudPublisher::~PointCloudPublisher() {
  Stop();
  wait();
}

void PointCloudPublisher::run() {
  is_running_ = true;
  while (is_running_) {
    if (is_paused_) {
      QThread::msleep(100);
      continue;
    }
    if (curr_index_ >= static_cast<int>(file_paths_.size())) {
      is_running_ = false;
      emit LogMessage("Publish all data, exit.");
      ReportPublishDone();
      break;
    }

    PublishOnce();
    timer.sleep();

    if (!is_looping_) {
      ++curr_index_;
    }
  }
}

void PointCloudPublisher::Stop() {
  QMutexLocker locker(&mutex_);
  is_running_ = false;
}

void PointCloudPublisher::Pause() {
  QMutexLocker locker(&mutex_);
  is_paused_ = true;
}

void PointCloudPublisher::Resume() {
  QMutexLocker locker(&mutex_);
  is_paused_ = false;
}

void PointCloudPublisher::EnableLooping() {
  QMutexLocker locker(&mutex_);
  is_looping_ = true;
}

void PointCloudPublisher::DisableLooping() {
  QMutexLocker locker(&mutex_);
  is_looping_ = false;
}

void PointCloudPublisher::JumpToIndex(int index) {
  QMutexLocker locker(&mutex_);
  curr_index_ = index;
}

void PointCloudPublisher::SetFilePaths(const std::vector<std::string>& paths) {
  QMutexLocker locker(&mutex_);
  file_paths_ = paths;
}

void PointCloudPublisher::SetDataset(std::shared_ptr<Dataset> other) {
  QMutexLocker locker(&mutex_);
  dataset_ = other;
  emit LogMessage("Dataset updated.");
}

void PointCloudPublisher::SetFrequency(int freq) {
  QMutexLocker locker(&mutex_);
  timer = ros::Rate(freq);
}

void PointCloudPublisher::PublishOnce() {
  const auto& pc_path = file_paths_[curr_index_];

  if (dataset_ == nullptr) {
    emit LogMessage("dataset nullptr");
  } else {
    dataset_->LoadPointCloud(pc_path, point_cloud_msg_);
    point_cloud_msg_.header.frame_id = frame_name_point_cloud_;
    point_cloud_msg_.header.stamp = ros::Time::now();
    pub_point_cloud_.publish(point_cloud_msg_);
  }

  QString message = QString("Publish from: %1")
                        .arg(QString::fromStdString(pc_path));
  emit LogMessage(message);
  emit ProgressUpdated(curr_index_);
}