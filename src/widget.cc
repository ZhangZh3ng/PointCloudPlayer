#include "widget.h"
#include "QFileDialog"
#include "QtDebug"
#include "qstring.h"
#include "ui_widget.h"

#include <filesystem>

#include "pointcloudpublisher.h"
#include "dataset/kitti_dataset.h"
#include "dataset/nclt_dataset.h"

Widget::Widget(ros::NodeHandle nh, QWidget* parent)
    : QWidget(parent),
      ui_(new Ui::Widget),
      publisher_thread_(new PointCloudPublisher(nh, this)),
      is_playing_(false),
      is_looping_(false),
      curr_index_(0),
      dataset_(std::make_shared<KittiDataset>()) {
  ui_->setupUi(this);

  setWindowTitle("Point Cloud Player");
  ui_->datasetComboBox->addItem("KITTI");
  ui_->datasetComboBox->addItem("NCLT");
  ui_->datasetComboBox->addItem("MulRan");

  connect(publisher_thread_, &PointCloudPublisher::LogMessage, this,
          &Widget::AppendLogMessage);
  connect(publisher_thread_, &PointCloudPublisher::ProgressUpdated, this,
          &Widget::UpdateProgressBar);
  connect(publisher_thread_, &PointCloudPublisher::ReportPublishDone, this,
          &Widget::PublishDone);

  ui_->runStopButton->setEnabled(false);
  ui_->startButton->setEnabled(false);
  ui_->pauseButton->setEnabled(false);
  ui_->loopButton->setEnabled(false);
}

Widget::~Widget() {
  delete publisher_thread_;
}

void Widget::UpdateFolder(std::vector<std::string>& bin_file) {
  file_pathes_ = bin_file;
  std::sort(file_pathes_.begin(), file_pathes_.end());
  ui_->outputTextEdit->append(
      QString("Success: Found %1 .bin files in the selected folder.")
          .arg(file_pathes_.size()));

  ui_->progressBar->setMinimum(0);
  ui_->progressBar->setMaximum(file_pathes_.size() - 1);
  ui_->progressBar->setValue(0);
  ui_->runStopButton->setEnabled(true);
}

void Widget::on_datasetComboBox_currentIndexChanged(int index) {
  switch (index) {
    case 0:
      dataset_ = std::make_shared<KittiDataset>();
      break;
    case 1:
      dataset_ = std::make_shared<NCLTDataset>();
      break;
    case 2:
      // mulran actually same with kitti
      dataset_ = std::make_shared<KittiDataset>();
    default:
      dataset_ = std::make_shared<KittiDataset>();
      break;
  }
  ui_->outputTextEdit->append(QString("Dataset type changed to: %1")
                                 .arg(ui_->datasetComboBox->currentText()));
}

void Widget::on_fileButton_clicked() {
  QString folder =
      QFileDialog::getExistingDirectory(this, "Select Point Cloud Folder");

  if (folder.isEmpty()) {
    ui_->outputTextEdit->append("Error: No folder selected.");
    return;
  }

  std::vector<std::string> bin_files;
  std::string std_folder = folder.toStdString();
  std::string pc_suffix = dataset_->PointCloudSuffix();
  try {
    std::filesystem::path pc_path(std_folder);

    if (!std::filesystem::exists(pc_path) ||
        !std::filesystem::is_directory(pc_path)) {
      ui_->outputTextEdit->append(
          "Error: Selected path is not a valid directory.");
      return;
    }

    int file_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(pc_path)) {
      if (entry.is_regular_file() && entry.path().extension() == pc_suffix) {
        bin_files.push_back(entry.path().string());
        ++file_count;
      }
    }

    if (file_count == 0) {
      ui_->outputTextEdit->append(
          "Error: No .bin files found in the selected folder.");
    } else {
      // Success Case:
      UpdateFolder(bin_files);
    }
  } catch (const std::filesystem::filesystem_error& e) {
    ui_->outputTextEdit->append(QString("Filesystem error: %1").arg(e.what()));
  }
}

void Widget::on_runStopButton_clicked() {
  if (is_playing_) {
    publisher_thread_->Stop();
    publisher_thread_->wait();
    is_playing_ = false;
    ToStopState();
  } else {
    if (!file_pathes_.empty()) {
      is_playing_ = true;
      publisher_thread_->SetFilePaths(file_pathes_);
      publisher_thread_->start();
      ToRunState();
      ResetPublisherThread();
    } else {
      ui_->outputTextEdit->append("Error: No point cloud files to play.");
    }
  }
}

void Widget::on_startButton_clicked() {
  publisher_thread_->Resume();
  ui_->outputTextEdit->append("Playback resumed.");
}

void Widget::on_pauseButton_clicked() {
  publisher_thread_->Pause();
  ui_->outputTextEdit->append("Playback paused.");
}

void Widget::on_loopButton_clicked() {
  is_looping_ = !is_looping_;
  if (is_looping_) {
    ui_->loopButton->setText("Looping");
    publisher_thread_->EnableLooping();
  } else {
    ui_->loopButton->setText("Loop");
    publisher_thread_->DisableLooping();
  }
}

void Widget::on_progressBar_valueChanged(int value) {
  curr_index_ = value;
  UpdatePlayStatus();
  UpdateProgressLabel();
}

void Widget::on_frequencySpinBox_valueChanged(int value) {
    publisher_thread_->SetFrequency(value);
    ui_->outputTextEdit->append(QString("Publishing frequency set to: %1 Hz").arg(value));
}

void Widget::UpdateProgressLabel() {
  QString progressText =
      QString("%1 : %2").arg(curr_index_).arg(file_pathes_.size());
  ui_->progressLabel->setText(progressText);
}

void Widget::AppendLogMessage(const QString& message) {
  ui_->outputTextEdit->append(message);
}

void Widget::UpdatePlayStatus() {
  if (curr_index_ < 0 || curr_index_ >= static_cast<int>(file_pathes_.size())) {
    ui_->outputTextEdit->append("Error: Index out of bounds.");
    return;
  }
  QString progressText =
      QString("%1 : %2").arg(curr_index_).arg(file_pathes_.size());
  // ui->progressBar->setText(progressText);
  publisher_thread_->JumpToIndex(curr_index_);
}

void Widget::ResetPublisherThread() {
  // update dataset type
  publisher_thread_->SetDataset(dataset_);
  // path
  publisher_thread_->SetFilePaths(file_pathes_);
  // index
  publisher_thread_->JumpToIndex(0);
  // frequency
  publisher_thread_->SetFrequency(ui_->frequencySpinBox->value());
}

void Widget::UpdateProgressBar(int value) {
  curr_index_ = value;
  ui_->progressBar->setValue(value);
  UpdateProgressLabel();
}

void Widget::PublishDone() {
  publisher_thread_->Stop();
  publisher_thread_->wait();
  is_playing_ = false;
  ui_->outputTextEdit->append("Playback stopped.");
  ui_->runStopButton->setText("Run");
  ui_->startButton->setEnabled(false);
  ui_->pauseButton->setEnabled(false);
  ui_->loopButton->setEnabled(false);
}

void Widget::ToRunState() {
  ui_->outputTextEdit->append("Playback started.");
  ui_->runStopButton->setText("Stop");
  ui_->startButton->setEnabled(true);
  ui_->pauseButton->setEnabled(true);
  ui_->loopButton->setEnabled(true);

  ui_->fileButton->setEnabled(false);
  ui_->datasetComboBox->setEnabled(false);

  // Reset progress bar
  UpdateProgressBar(0);
}

void Widget::ToStopState() {
  ui_->outputTextEdit->append("Playback stopped.");
  ui_->runStopButton->setText("Run");
  ui_->startButton->setEnabled(false);
  ui_->pauseButton->setEnabled(false);
  ui_->loopButton->setEnabled(false);

  ui_->fileButton->setEnabled(true);
  ui_->datasetComboBox->setEnabled(true);
}
