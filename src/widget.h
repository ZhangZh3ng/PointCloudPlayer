#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <QSlider>
#include <QLabel>
#include <QSpinBox>
#include "pointcloudpublisher.h"

#include "dataset/kitti_dataset.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_datasetComboBox_currentIndexChanged(int index);
    void on_fileButton_clicked();
    void on_runStopButton_clicked();
    void on_startButton_clicked();
    void on_pauseButton_clicked();
    void on_loopButton_clicked();
    void on_progressBar_valueChanged(int value);
    void on_frequencySpinBox_valueChanged(int value);

    void appendLogMessage(const QString& message);

private:
    enum DataSetType {
        KITTI,
        NCLT
    };

    Ui::Widget *ui;
    PointCloudPublisher *publisherThread;
    std::vector<std::string> pc_pathes_;
    bool isPlaying;
    bool isLooping;
    int currentIndex;
    DataSetType dataset_type;

    void UpdateFolder(std::vector<std::string>& bin_file);
    void updatePlayStatus();
    void updateProgressLabel();
    void updateProgressBar(int value);
    void publishDone();

    void ToRunState();
    void ToStopState();

    void UpdatePublisherThread();
};
#endif // WIDGET_H
