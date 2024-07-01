#include "pointcloudpublisher.h"
#include <QDebug>

PointCloudPublisher::PointCloudPublisher(QObject* parent)
    : QThread(parent),
      running(false),
      paused(false),
      looping(false),
      currentIndex(0) {}

PointCloudPublisher::~PointCloudPublisher() {
  stop();
  wait();
}

void PointCloudPublisher::run() {
  running = true;
  while (running) {
    if (paused) {
      QThread::msleep(100);
      continue;
    }
    if (currentIndex >= static_cast<int>(filePaths.size())) {
        running = false;
        emit logMessage("Publish all data, exit.");
        reportPublishDone();
        break;
    }

    // 在这里添加发布点云的代码
    QString message = QString("Publish from: %1")
                          .arg(QString::fromStdString(filePaths[currentIndex]));
    emit logMessage(message);
    emit progressUpdated(currentIndex);

    QThread::sleep(1);  // 模拟发布间隔

    if (!looping) {
      ++currentIndex;
    }
  }
}

void PointCloudPublisher::stop() {
  QMutexLocker locker(&mutex);
  running = false;
}

void PointCloudPublisher::pause() {
  QMutexLocker locker(&mutex);
  paused = true;
}

void PointCloudPublisher::resume() {
  QMutexLocker locker(&mutex);
  paused = false;
}

void PointCloudPublisher::enableLooping() {
  QMutexLocker locker(&mutex);
  looping = true;
}

void PointCloudPublisher::disableLooping() {
  QMutexLocker locker(&mutex);
  looping = false;
}

void PointCloudPublisher::jumpToIndex(int index) {
  QMutexLocker locker(&mutex);
  currentIndex = index;
}

void PointCloudPublisher::setFilePaths(const std::vector<std::string>& paths) {
  QMutexLocker locker(&mutex);
  filePaths = paths;
  maxIndex = filePaths.size() - 1;
}

void PointCloudPublisher::SetDataset(std::unique_ptr<Dataset> other) {
    QMutexLocker locker(&mutex);
    dataset = std::move(other);
    emit logMessage("Dataset updated.");
}

void PointCloudPublisher::setFrequency(int freq) {
    QMutexLocker locker(&mutex);
    frequency = freq;
}