#include "widget.h"
#include <QApplication>

#ifdef USE_ROS
#include <ros/ros.h>
#endif

int main(int argc, char *argv[])
{
#ifdef USE_ROS
    ros::init(argc, argv, "point_cloud_player");
#endif

    QApplication a(argc, argv);
    Widget w;
    w.show();
    return a.exec();
}
