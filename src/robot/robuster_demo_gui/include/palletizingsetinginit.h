#ifndef PALLETIZINGSETINGINIT_H
#define PALLETIZINGSETINGINIT_H

#include <QWidget>
#include <QCheckBox>
#include <QPushButton>
#include <QtWidgets/QTableWidget>
#include <QLabel>
#include "task.h"
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include<QMessageBox>
#define YAML_CPP 1
#if YAML_CPP
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <QList>
#include <QSlider>
#endif
#define ROSFLAG 1
#if ROSFLAG
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#endif
class PalletizingSetingstencildata
{  // 模版
public:
  QString pick_pose;
  QString place_pose;
  QString arm_detect_pose;
  QString arm_motion_place;
  QString arm_home_pose;
  QString arm_place_pose;
  QString place_marker_id;
  QString placement;

};
class QPalletizingSetingstencildata
{  // 模版
public:
  QList<double> pick_pose;
  QList<double> place_pose;
  QList<double> arm_detect_pose;
  QList<double> arm_motion_place;
  QList<double> arm_home_pose;
  QList<double>  arm_place_pose;
  int place_marker_id;
  int placement;
  int detect_method;
  float move_forward_distance;
};

namespace Ui
{
class PalletizingSetingInit;
}

class PalletizingSetingInit : public QWidget
{
  Q_OBJECT

public:
  QString yamlPath;
  QPalletizingSetingstencildata qWritePalletizingSetingstencildata;
  explicit PalletizingSetingInit(QWidget* parent = nullptr);
  PalletizingSetingInit();
  ~PalletizingSetingInit();
  void readYaml(QString path);
  void writeYaml(QString path, int choose);
  void editCell(QString Messager, int row , int col );
  void qWritePalletizingSetingstencildataClear();
  void initTaskTable();
  void initTaskYaml();
  void stateCallback(const std_msgs::Int64::ConstPtr& state);
  void addTaskItem(geometry_msgs::Pose pose, std::string action);
  void addinitTskItem(geometry_msgs::Pose pose, std::string action);
  void updateTask(geometry_msgs::Pose pose, std::string action);
  protected Q_SLOTS:
  void addTask();
  void deleteTask();
  void saveTaskConfig();
  void clickComboBox(QString text);
  void executeTask();
  void checkRpeat();
  
signals:
  void myClicked(const int& row, const int& col);

private:
  Ui::PalletizingSetingInit* ui;
  //QSlider* distance_horizontalSlider;
  PalletizingSetingstencildata palletizingSetingstencildata;
  QPalletizingSetingstencildata qPalletizingSetingstencildata;
  QTableWidget *task_table_;
  QCheckBox *repeat_checkbox_;
  QLabel *state_label_;
  QPushButton *add_btn_, *delete_btn_, *reset_btn_, *save_btn_, *execute_btn_;

  Task task_list_;
  std::string task_config_param_;
  std::string task_config_file_;
  ros::ServiceClient task_client_;
  ros::ServiceClient reload_task_client_;
  ros::NodeHandle nh_;
  ros::Publisher task_exec_;
  ros::Publisher task_stop_;
  ros::Subscriber state_sub_;

  int64_t cur_state_;
  void horizontalSliderInit(QWidget* parent = nullptr);
  double NormalizeAngle(const double angle) ;
#if ROSFLAG

  tf::TransformListener transform_listener_;
  tf::StampedTransform transform_;
  bool getRobotPose(geometry_msgs::Pose& pose);
#endif

  void initSize();
  void initColoer();
  void initConnect();
  void init();
};

#endif  // PALLETIZINGSETINGINIT_H
