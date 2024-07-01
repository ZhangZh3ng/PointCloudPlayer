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
#include "dataset/dataset.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(ros::NodeHandle nh, QWidget *parent = nullptr);
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

    void AppendLogMessage(const QString& message);

private:
    Ui::Widget *ui_;
    PointCloudPublisher *publisher_thread_;
    std::vector<std::string> file_pathes_;
    bool is_playing_;
    bool is_looping_;
    int curr_index_;
    std::shared_ptr<Dataset> dataset_;

    void UpdateFolder(std::vector<std::string>& bin_file);
    void UpdatePlayStatus();
    void UpdateProgressLabel();
    void UpdateProgressBar(int value);
    void PublishDone();

    void ToRunState();
    void ToStopState();

    void ResetPublisherThread();
};
#endif // WIDGET_H
