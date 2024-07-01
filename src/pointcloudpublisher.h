#ifndef POINTCLOUDPUBLISHER_H
#define POINTCLOUDPUBLISHER_H

#include <QThread>
#include <QMutex>
#include <vector>
#include <string>
#include <memory>

#include "dataset/dataset.h"

class PointCloudPublisher : public QThread
{
    Q_OBJECT

public:
    PointCloudPublisher(QObject *parent = nullptr);
    ~PointCloudPublisher();

    void run() override;
    void stop();
    void pause();
    void resume();
    void enableLooping();
    void disableLooping();
    void jumpToIndex(int index);
    void setFilePaths(const std::vector<std::string>& paths);
    void SetDataset(std::unique_ptr<Dataset> other);
    void setFrequency(int freq);
    
    signals:
    void logMessage(const QString& message);
    void progressUpdated(int value);
    void reportPublishDone();

private:
    QMutex mutex;
    bool running;
    bool paused;
    bool looping;
    int currentIndex;
    int maxIndex;
    int frequency;
    std::vector<std::string> filePaths;
    std::unique_ptr<Dataset> dataset;
};

#endif // POINTCLOUDPUBLISHER_H
