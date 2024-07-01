#include <QApplication>
#include "widget.h"

#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "point_cloud_player");

  ros::NodeHandle nh;
  QApplication a(argc, argv);
  Widget w(nh);
  w.show();
  return a.exec();
}
