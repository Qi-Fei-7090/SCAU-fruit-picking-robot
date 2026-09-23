#ifndef ROBUSTERDEMO_H
#define ROBUSTERDEMO_H

#include <QMainWindow>
#include <QScreen>
#include <QProcess>
#include <QDebug>
#include <QCoreApplication>
#include <QApplication>
#include <QWidget>
#include<QMessageBox>
#include <QMouseEvent>
#define ROSFLAG 1
#if ROSFLAG
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#endif
//点击按钮需要检查导航功能是否开启。
//点击按钮需要检查是否开启过其他服务。
//重启ros服务->启动导航服务->选定功能->run。
//考虑到功能冲突，不做异步处理。
namespace Ui
{
class RobusterDemo;
}

class RobusterDemo : public QMainWindow
{
  Q_OBJECT

public:
  explicit RobusterDemo(QWidget* parent = nullptr);
  ~RobusterDemo();

private slots:
  //
  //
  void readStandardOutput();
  void readStandardError();
  void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void palletizingSeting();

private:
  Ui::RobusterDemo* ui;
  QProcess* mProcess;
  QProcess* detachedProcess;  //码垛设置逻辑;
  QString yamlPath;
  bool m_running;
  void process(QString cmd);
  bool isNavigationState();
  void exec_bash_Memory(QString bash);
  QByteArray exec_bash_return(QString bash);
  // true 打开新终端
  // false 不开新终端但是不阻塞
  void exec_bash_Memory(QString bash, bool detached);
  void palletizingSetingInit();  //码垛设置初始化
  void palletizingSetingWidget();
  // void goPalletizing();
  void grasp();

  // void goSorting();
  // void goApple();
  bool promptState(QString str);
  void checkNavigationState();
#if ROSFLAG
tf::TransformListener transform_listener_;
tf::StampedTransform transform_;
 void getRobotPose(geometry_msgs::Pose &pose);
#endif
};

#endif  // ROBUSTERDEMO_H
