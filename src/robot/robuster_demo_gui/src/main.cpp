#include "robusterdemo.h"
#include <QApplication>
#if ROSFLAG
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#endif
int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
#if ROSFLAG
  ros::init(argc, argv, "read_tf");
#endif
  RobusterDemo w;

  w.show();

  return a.exec();
}
