#include "widget.h"
#include "QFileDialog"
#include "QtDebug"
#include "qstring.h"
#include "ui_widget.h"

#include <filesystem>

#include "dataset/kitti_dataset.h"
#include "pointcloudpublisher.h"

Widget::Widget(QWidget* parent)
    : QWidget(parent),
      ui(new Ui::Widget),
      publisherThread(new PointCloudPublisher(this)),
      isPlaying(false),
      isLooping(false),
      currentIndex(0),
      dataset_type(DataSetType::KITTI) {
  ui->setupUi(this);

  // connect(ui->fileButton, &QPushButton::clicked, this,
  // &Widget::on_fileButton_clicked); connect(ui->runStopButton,
  // &QPushButton::clicked, this, &Widget::on_runStopButton_clicked);
  // connect(ui->startButton, &QPushButton::clicked, this,
  // &Widget::on_startButton_clicked); connect(ui->pauseButton,
  // &QPushButton::clicked, this, &Widget::on_pauseButton_clicked);
  // connect(ui->loopButton, &QPushButton::clicked, this,
  // &Widget::on_loopButton_clicked); connect(ui->progressBar,
  // &QSlider::valueChanged, this, &Widget::on_progressBar_valueChanged);
  // connect(ui->jumpSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this,
  // &Widget::on_jumpSpinBox_valueChanged);
  connect(publisherThread, &PointCloudPublisher::logMessage, this,
          &Widget::appendLogMessage);
  connect(publisherThread, &PointCloudPublisher::progressUpdated, this,
          &Widget::updateProgressBar);
  connect(publisherThread, &PointCloudPublisher::reportPublishDone, this,
          &Widget::publishDone);

  ui->runStopButton->setEnabled(false);
  ui->startButton->setEnabled(false);
  ui->pauseButton->setEnabled(false);
  ui->loopButton->setEnabled(false);
}

Widget::~Widget() {
  delete publisherThread;
}

void Widget::UpdateFolder(std::vector<std::string>& bin_file) {
  pc_pathes_ = bin_file;
  std::sort(pc_pathes_.begin(), pc_pathes_.end());
  ui->outputTextEdit->append(
      QString("Success: Found %1 .bin files in the selected folder.")
          .arg(pc_pathes_.size()));

  ui->progressBar->setMinimum(0);
  ui->progressBar->setMaximum(pc_pathes_.size() - 1);
  ui->progressBar->setValue(0);
  ui->runStopButton->setEnabled(true);
}

void Widget::on_datasetComboBox_currentIndexChanged(int index) {
  switch (index) {
    case 0:
      dataset_type = DataSetType::KITTI;
      break;
    // 你可以在这里添加其他数据集的选项
    case 1:
      dataset_type = DataSetType::NCLT;
      break;
    default:
      dataset_type = DataSetType::KITTI;
      break;
  }
  ui->outputTextEdit->append(QString("Dataset type changed to: %1")
                                 .arg(ui->datasetComboBox->currentText()));
}

void Widget::on_fileButton_clicked() {
  QString folder =
      QFileDialog::getExistingDirectory(this, "Select Point Cloud Folder");

  if (folder.isEmpty()) {
    ui->outputTextEdit->append("Error: No folder selected.");
    return;
  }

  std::vector<std::string> bin_files;
  std::string std_folder = folder.toStdString();

  try {
    std::filesystem::path pc_path(std_folder);

    if (!std::filesystem::exists(pc_path) ||
        !std::filesystem::is_directory(pc_path)) {
      ui->outputTextEdit->append(
          "Error: Selected path is not a valid directory.");
      return;
    }

    int file_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(pc_path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".bin") {
        bin_files.push_back(entry.path().string());
        ++file_count;
      }
    }

    if (file_count == 0) {
      ui->outputTextEdit->append(
          "Error: No .bin files found in the selected folder.");
    } else {
      // Success Case:
      UpdateFolder(bin_files);
    }
  } catch (const std::filesystem::filesystem_error& e) {
    ui->outputTextEdit->append(QString("Filesystem error: %1").arg(e.what()));
  }
}

void Widget::on_runStopButton_clicked() {
  if (isPlaying) {
    publisherThread->stop();
    publisherThread->wait();
    isPlaying = false;
    ToStopState();
  } else {
    if (!pc_pathes_.empty()) {
      isPlaying = true;
      publisherThread->setFilePaths(pc_pathes_);
      publisherThread->start();
      ToRunState();
      UpdatePublisherThread();
    } else {
      ui->outputTextEdit->append("Error: No point cloud files to play.");
    }
  }
}

void Widget::on_startButton_clicked() {
  publisherThread->resume();
  ui->outputTextEdit->append("Playback resumed.");
}

void Widget::on_pauseButton_clicked() {
  publisherThread->pause();
  ui->outputTextEdit->append("Playback paused.");
}

void Widget::on_loopButton_clicked() {
  isLooping = !isLooping;
  if (isLooping) {
    ui->loopButton->setText("Looping");
    publisherThread->enableLooping();
  } else {
    ui->loopButton->setText("Loop");
    publisherThread->disableLooping();
  }
}

void Widget::on_progressBar_valueChanged(int value) {
  currentIndex = value;
  updatePlayStatus();
  updateProgressLabel();
}

void Widget::on_frequencySpinBox_valueChanged(int value) {
    publisherThread->setFrequency(value);
    ui->outputTextEdit->append(QString("Publishing frequency set to: %1 Hz").arg(value));
}

void Widget::updateProgressLabel() {
  QString progressText =
      QString("%1 : %2").arg(currentIndex).arg(pc_pathes_.size());
  ui->progressLabel->setText(progressText);
}

void Widget::appendLogMessage(const QString& message) {
  ui->outputTextEdit->append(message);
}

void Widget::updatePlayStatus() {
  if (currentIndex < 0 || currentIndex >= static_cast<int>(pc_pathes_.size())) {
    ui->outputTextEdit->append("Error: Index out of bounds.");
    return;
  }
  QString progressText =
      QString("%1 : %2").arg(currentIndex).arg(pc_pathes_.size());
  // ui->progressBar->setText(progressText);
  publisherThread->jumpToIndex(currentIndex);
}

void Widget::UpdatePublisherThread() {
  // update dataset type
  switch (dataset_type) {
    case DataSetType::KITTI:
      publisherThread->SetDataset(std::make_unique<KittiDataset>());
      break;

    default:
      break;
  }

  // path
  publisherThread->setFilePaths(pc_pathes_);
  // index
  publisherThread->jumpToIndex(0);
}

void Widget::updateProgressBar(int value) {
  currentIndex = value;
  ui->progressBar->setValue(value);
  updateProgressLabel();
}

void Widget::publishDone() {
  publisherThread->stop();
  publisherThread->wait();
  isPlaying = false;
  ui->outputTextEdit->append("Playback stopped.");
  ui->runStopButton->setText("Run");
  ui->startButton->setEnabled(false);
  ui->pauseButton->setEnabled(false);
  ui->loopButton->setEnabled(false);
}

void Widget::ToRunState() {
  ui->outputTextEdit->append("Playback started.");
  ui->runStopButton->setText("Stop");
  ui->startButton->setEnabled(true);
  ui->pauseButton->setEnabled(true);
  ui->loopButton->setEnabled(true);

  ui->fileButton->setEnabled(false);
  ui->datasetComboBox->setEnabled(false);

  // Reset progress bar
  updateProgressBar(0);
}

void Widget::ToStopState() {
  ui->outputTextEdit->append("Playback stopped.");
  ui->runStopButton->setText("Run");
  ui->startButton->setEnabled(false);
  ui->pauseButton->setEnabled(false);
  ui->loopButton->setEnabled(false);

  ui->fileButton->setEnabled(true);
  ui->datasetComboBox->setEnabled(true);
}
