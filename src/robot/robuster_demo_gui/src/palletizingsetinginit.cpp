#include "palletizingsetinginit.h"
#include "ui_palletizingsetinginit.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <QDebug>
#include <QComboBox>
#include <QSlider>
#include <QLayout>
#include <QLabel>
PalletizingSetingInit::PalletizingSetingInit(QWidget* parent) : QWidget(parent), ui(new Ui::PalletizingSetingInit)
{
  ui->setupUi(this);
  task_config_file_ = getenv("HOME") + string("/rpp_data/config/bt_task_manager/task_grasp.yaml");
  std::cout<<"task config file"<<task_config_file_<<endl;
  
  task_exec_ = nh_.advertise<std_msgs::Empty>("/controller_node/dashboard/start",1);
  task_stop_ = nh_.advertise<std_msgs::Empty>("/controller_node/dashboard/cancel",1);
  state_sub_ = nh_.subscribe<std_msgs::Int64>("/control/state", 10, &PalletizingSetingInit::stateCallback, this);
  QVBoxLayout *root_layout = new QVBoxLayout;
  task_table_ = new QTableWidget;
    
  root_layout->addWidget(task_table_);
  QHBoxLayout *repateAndState_layout = new QHBoxLayout;
  repeat_checkbox_ = new QCheckBox("Repeat");
  repeat_checkbox_->setCheckState(Qt::Unchecked);
  repateAndState_layout->addWidget(repeat_checkbox_);
  state_label_ = new QLabel;
  repateAndState_layout->addWidget(state_label_);
  
  root_layout->addLayout(repateAndState_layout);

  QHBoxLayout *manipulate_layout = new QHBoxLayout;
  add_btn_ = new QPushButton("Add");
  manipulate_layout->addWidget(add_btn_);
  delete_btn_ = new QPushButton("Delecte");
  manipulate_layout->addWidget(delete_btn_);
  reset_btn_ = new QPushButton("Reset");
  // manipulate_layout->addWidget(reset_btn_);
  save_btn_ = new QPushButton("Save");
  manipulate_layout->addWidget(save_btn_);
  execute_btn_ = new QPushButton("Execute");
  manipulate_layout->addWidget(execute_btn_);
  root_layout->addLayout(manipulate_layout);

  setLayout(root_layout);

  initTaskTable();

  connect(add_btn_, SIGNAL(clicked()), this, SLOT(addTask()));
  connect(delete_btn_, SIGNAL(clicked()), this, SLOT(deleteTask()));
  connect(save_btn_, SIGNAL(clicked()), this, SLOT(saveTaskConfig()));
  connect(execute_btn_, SIGNAL(clicked()), this, SLOT(executeTask()));
  connect(repeat_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkRpeat()));
}
void PalletizingSetingInit::checkRpeat() {
    task_list_.setRepeatFlag(repeat_checkbox_->isChecked());
  }

void  PalletizingSetingInit::stateCallback(const std_msgs::Int64::ConstPtr& state)
{
  cur_state_ = state->data;
    if (cur_state_ <= 0) { // grasp task stopped.
      execute_btn_->setText("Execute");
    } else {
      execute_btn_->setText("Stop");
    }

}
void PalletizingSetingInit::executeTask()
   {

    int16_t task_op = 0;
    std_msgs::Empty msg;
    if (execute_btn_->text() == QString::fromLocal8Bit("Execute")) {
      ROS_INFO("Start task");
      task_exec_.publish(msg);
      execute_btn_->setText("Stop");
    } else if (execute_btn_->text() == QString::fromLocal8Bit("Stop")) {
      ROS_INFO("Stop task");
      task_stop_.publish(msg);
      execute_btn_->setText("Execute");
    }

  }



void PalletizingSetingInit::saveTaskConfig()
{
  YAML::Node config;
    ROS_INFO("Save task list to %s", task_config_file_.c_str());
    ofstream fout(task_config_file_);
    config["name"] = task_list_.getTaskName();
    config["repeat"] = task_list_.getRepeatFlag();
    for (int i = 0; i < task_list_.meta_tasks_.size(); i++) {
      YAML::Node task;
      // ROS_INFO("There have %ld tasks, cur %dth task", task_list_.meta_tasks_.size(), i);
      for (int j = 0; j < task_list_.meta_tasks_[i].size(); j++) {
        // ROS_INFO("There have %ld sub tasks, cur %dth sub task (%s)", task_list_.meta_tasks_[i].size(), j,
        //               task_list_.meta_tasks_[i][j].getTaskType().c_str());
        if (task_list_.meta_tasks_[i][j].getTaskType() == "nav") {
          geometry_msgs::Pose pose = task_list_.meta_tasks_[i][j].getTaskPose();
	  // geometry_msgs::Pose pose1;
    //       this->getRobotPose(pose1);
	  double yaw = tf::getYaw(pose.orientation);
	  yaw = yaw * 180 /M_PI;
          std::string positionString = "[" + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " + std::to_string(yaw) + "]";
          task["position"] = YAML::Load(positionString);
        } else if (task_list_.meta_tasks_[i][j].getTaskType() == "action") {
          task["action"] = task_list_.meta_tasks_[i][j].getTaskAction();
        }
      }
      config["tasks"].push_back(task);
    }
    // std::cout << config << std::endl;
    fout << config;
    fout.close();

    /// 保存后,更新 control 中的任务列表
    std_srvs::Empty msg;
    if (reload_task_client_.call(msg)) {
      ROS_INFO("Call reload_task service success.");
    } else {
      ROS_WARN("Call reload_task service failed.");
    }
}

void PalletizingSetingInit::deleteTask()
{
   int row_index = task_table_->currentRow();
    ROS_INFO("delete task %d", row_index);
    if (row_index != -1) {
      task_table_->removeRow(row_index);
      // ROS_INFO("line %d, task list size: %ld", row_index, task_list_.meta_tasks_.size());
      if (row_index < task_list_.meta_tasks_.size()) {
        task_list_.meta_tasks_.erase(task_list_.meta_tasks_.begin() + row_index);
      }
    }
    saveTaskConfig();
}
bool promptState(QString str)
{
  QMessageBox errorDialog;
  errorDialog.setIcon(QMessageBox::Critical);  // 设置对话框图标为错误图标
  errorDialog.setWindowTitle("Error");         // 设置对话框标题
  errorDialog.setText(str);                    // 设置对话框显示的错误消息
  errorDialog.exec();                          // 显示对话框，阻塞程序执行直到对话框关闭
}
void PalletizingSetingInit::addTask()
{
  //warn  当前为了调试添加一个点 
  geometry_msgs::Pose pose; 
  if(!getRobotPose(pose))
  {
    if (!promptState("请先打开导航功能"))
    {
      return;
    }
  }  

  // 弹窗,提示当前位置及配置 action
  // pose.orientation.w = 1.0;
  // pose.orientation.z = 2.0;
  // pose.orientation.x = 3.0;
  // pose.orientation.y = 4.0;
  // pose.position.x = 5.0;
  // pose.position.y = 6.0;
  // pose.position.z = 7.0;
  addTaskItem(pose, string("None"));

    vector<MetaTask> task;
    MetaTask meta_task_nav;
    meta_task_nav.setTaskPose(pose);
    meta_task_nav.setTaskType("nav");
    task.push_back(meta_task_nav);
    task_list_.meta_tasks_.push_back(task);
}
void PalletizingSetingInit::addinitTskItem(geometry_msgs::Pose pose, std::string action)
{
  int row_count = task_table_->rowCount(); // 当前行数
    ROS_INFO("Current task rows: %d", row_count);
    task_table_->insertRow(row_count); // 增加一行
    // QString("%1").arg(f, 0, ‘f’, 6) // float to qstring
    QString item_str = QString::number(pose.position.x ,'f', 4) + ",   "
                      + QString::number(pose.position.y ,'f', 4) + ",   "
                      + QString::number(pose.orientation.w ,'f', 4);
    task_table_->setItem(row_count, 0, new QTableWidgetItem(item_str));
    QComboBox *action_combobox = new QComboBox;
    action_combobox->addItem("None");
    action_combobox->addItem("pick");
    action_combobox->addItem("place");  
    task_table_->setCellWidget(row_count, 1, action_combobox);
    action_combobox->setCurrentText(QString::fromStdString(action));
    connect(action_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(clickComboBox(QString)));

}

void PalletizingSetingInit::addTaskItem(geometry_msgs::Pose pose, std::string action) 
{
    int row_count = task_table_->rowCount(); // 当前行数
    ROS_INFO("Current task rows: %d", row_count);
    task_table_->insertRow(row_count); // 增加一行

    double yaw = tf::getYaw(pose.orientation);
    yaw = yaw * 180 /M_PI;
    // QString("%1").arg(f, 0, ‘f’, 6) // float to qstring
    QString item_str = QString::number(pose.position.x ,'f', 4) + ",   "
                      + QString::number(pose.position.y ,'f', 4) + ",   "
                      + QString::number(yaw ,'f', 4);
    task_table_->setItem(row_count, 0, new QTableWidgetItem(item_str));
    QComboBox *action_combobox = new QComboBox;
    action_combobox->addItem("None");
    action_combobox->addItem("pick");
    action_combobox->addItem("place");  
    task_table_->setCellWidget(row_count, 1, action_combobox);
    action_combobox->setCurrentText(QString::fromStdString(action));
    connect(action_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(clickComboBox(QString)));
  }
void PalletizingSetingInit::clickComboBox(QString text) 
{
    QComboBox *comBox_ = dynamic_cast<QComboBox*>(this->sender());
    if(NULL == comBox_) {
      return;
    }
    
    int x = comBox_->frameGeometry().x();
    int y = comBox_->frameGeometry().y();
    QModelIndex index = task_table_->indexAt(QPoint(x, y));
    int row = index.row();
    int column = index.column();
    std::cout << "Set line " << row << "and column " << column << " to " << text.toStdString() << std::endl;

    if (row >= task_list_.meta_tasks_.size()) {
      return ;
    }

    if (task_list_.meta_tasks_[row].size() < 2) { // 当前任务只包含 nav
      if (text == QString::fromLocal8Bit("None")) {
        return ;
      }
      MetaTask meta_task_action;
      meta_task_action.setTaskType("action");
      meta_task_action.setTaskAction(text.toStdString());
      task_list_.meta_tasks_[row].push_back(meta_task_action);
    } else {
      if (text == QString::fromLocal8Bit("None")) {
        task_list_.meta_tasks_[row].erase(task_list_.meta_tasks_[row].begin() + 1);
      } else {
        task_list_.meta_tasks_[row][1].setTaskAction(text.toStdString());
      }
    }
  }

void PalletizingSetingInit::initTaskYaml()
{
  std::ifstream file(getenv("HOME") + string("/.robuster/maps/control_tasks.yaml"));
  if(file.good())
  {
    // std::cout<<"yaml is exist"<<endl;
  }else{
    // std::cout<<"yaml is  not exist"<<endl;
    //  if (!promptState("yaml 文件不存在"))
    // {
      return;
    // }
  }
  // 读取YAML文件
    YAML::Node config = YAML::LoadFile("/home/robuster/.robuster/maps/control_tasks.yaml");

    // 打印解析后的数据
    // 遍历tasks数组
    for (const auto& task : config["tasks"]) {
        std::cout << "  - Position: ";

        std::cout << task["position"][0].as<double>() << " ";
        std::cout << task["position"][1].as<double>() << " ";
        std::cout << task["position"][2].as<double>() << " ";
        geometry_msgs::Pose pose; 
        pose.position.x = task["position"][0].as<double>();
        pose.position.y = task["position"][1].as<double>();
        pose.orientation.w = task["position"][2].as<double>();
         if (task["action"]) {
            std::cout << "    Action: " << task["action"].as<std::string>() << std::endl;
            addinitTskItem(pose, string(task["action"].as<std::string>()));
        } else {
            std::cout << "    Action: No action specified." << std::endl;
            addinitTskItem(pose, string("None"));
        }
    

        vector<MetaTask> task_nav;
        MetaTask meta_task_nav;
        meta_task_nav.setTaskPose(pose);
        meta_task_nav.setTaskType("nav");
        task_nav.push_back(meta_task_nav);
        task_list_.meta_tasks_.push_back(task_nav);

    }
}
void PalletizingSetingInit::initTaskTable()
{
    task_table_->setColumnCount(2); // 列数
    // task_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    task_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    task_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    task_table_->setColumnWidth(1, 20);
    task_table_->verticalHeader()->setHidden(false); // 是否隐藏序号列
    QStringList header;
    header << "Pose(X, Y, Yaw)" << "Action";
    task_table_->setHorizontalHeaderLabels(header);
    initTaskYaml();
}
void PalletizingSetingInit::horizontalSliderInit(QWidget* parent)
{
  QSlider* distance_horizontalSlider = new QSlider(Qt::Horizontal, parent);

  // 设置范围
  distance_horizontalSlider->setMinimum(0);
  distance_horizontalSlider->setMaximum(2000);
  connect(distance_horizontalSlider, &QSlider::valueChanged, [&](int value) {
    //假定精度为mm
    float distance = value / 1000.0;
    this->editCell(QString::number(distance) + " m", 7, 1);
    this->editCell(" 未保存", 7, 4);
    this->qWritePalletizingSetingstencildata.move_forward_distance = distance;
    // qDebug() << distance;
  });
  // 将 widget 设置为表格的单元组件
  ui->palletizing_tableWidget->setCellWidget(7, 2, distance_horizontalSlider);
}

void PalletizingSetingInit::qWritePalletizingSetingstencildataClear()
{
  // 清除数据
  if (!qWritePalletizingSetingstencildata.pick_pose.isEmpty())
  {
    qWritePalletizingSetingstencildata.pick_pose.clear();
  }
}
PalletizingSetingInit::PalletizingSetingInit()
{
  init();
}
void PalletizingSetingInit::init()
{
  ui->palletizing_tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
  this->initSize();
  this->initColoer();
  this->initConnect();
}
PalletizingSetingInit::~PalletizingSetingInit()
{
  delete ui;
}
void PalletizingSetingInit::initSize()
{
  ui->palletizing_tableWidget->setColumnWidth(0, 240);
  ui->palletizing_tableWidget->setColumnWidth(1, 240);
}
void PalletizingSetingInit::initColoer()
{
  for (int row = 0; row < ui->palletizing_tableWidget->rowCount(); ++row)
  {
    for (int col = 0; col < ui->palletizing_tableWidget->columnCount(); ++col)
    {
      QTableWidgetItem* item = ui->palletizing_tableWidget->item(row, col);
      if (item)
      {
        item->setTextAlignment(Qt::AlignCenter);  // 文字居中
      }
    }
  }
}
PalletizingSetingstencildata demoPalletizingSetingstencildata = { .pick_pose = "[-8.112,12.111,0]",
                                                                  .place_pose = "[-11.302, 14.169,180]",
                                                                  .arm_detect_pose = "[0.6, 0, 0.85, 180,0,0]" ,};


void PalletizingSetingInit::writeYaml(QString path, int choose)
{
#if YAML_CPP
  if (choose < 0 || choose > 7)
  {
    qDebug() << "choose is error !!!!";
    return;
  }

  // path = "/home/robot/ros_qt_ws/src/mobile_manipulation/param/mobile_manipulation.yaml";

  // 读取 YAML 文件
  YAML::Node config = YAML::LoadFile(path.toStdString());

  switch (choose)
  {
    case 0:
      if (qWritePalletizingSetingstencildata.pick_pose.isEmpty())
      {
        std::cout<< "yaml error!!";
        return;
      }
      // 检查是否存在 pick_pose
      if (config["mobile_manipulation"]["pick_pose"])
      {
        std::cout<<"-------------------------"<<endl;
        // 获取原始的 pick_pose 值
        YAML::Node pick_pose = config["mobile_manipulation"]["pick_pose"];
        // 修改 pick_pose 的值
        pick_pose[0] = qWritePalletizingSetingstencildata.pick_pose[0];  // 修改 x 坐标
        pick_pose[1] = qWritePalletizingSetingstencildata.pick_pose[1];  // 修改 y 坐标
        pick_pose[2] = qWritePalletizingSetingstencildata.pick_pose[2];  // 修改 z 坐标
      }
      else
      {
        qDebug() << "pick_pose not found in YAML!";
      }
      break;
    case 1:
      if (qWritePalletizingSetingstencildata.place_pose.isEmpty())
      {
        qDebug() << "yaml error!!";
        return;
      }
      // 检查是否存在 place_pose
      if (config["mobile_manipulation"]["place_pose"])
      {
        // 获取原始的 place_pose 值
        YAML::Node place_pose = config["mobile_manipulation"]["place_pose"];
        // 修改 pick_pose 的值
        place_pose[0] = qWritePalletizingSetingstencildata.place_pose[0];  // 修改 x 坐标
        place_pose[1] = qWritePalletizingSetingstencildata.place_pose[1];  // 修改 y 坐标
        place_pose[2] = qWritePalletizingSetingstencildata.place_pose[2];  // 修改 z 坐标
      }
      else
      {
        qDebug() << "place_pose not found in YAML!";
      }
      break;
    case 2:
      if (qWritePalletizingSetingstencildata.arm_detect_pose.isEmpty())
      {
        qDebug() << "yaml error!!";
        return;
      }
      // 检查是否存在 arm_detect_pose
      if (config["mobile_manipulation"]["arm_detect_pose"])
      {
        // 获取原始的 arm_detect_pose 值
        YAML::Node arm_detect_pose = config["mobile_manipulation"]["arm_detect_pose"];
        // 修改 arm_detect_pose 的值
        arm_detect_pose[0] = qWritePalletizingSetingstencildata.arm_detect_pose[0];  // 修改
        arm_detect_pose[1] = qWritePalletizingSetingstencildata.arm_detect_pose[1];  // 修改
        arm_detect_pose[2] = qWritePalletizingSetingstencildata.arm_detect_pose[2];  // 修改
        arm_detect_pose[3] = qWritePalletizingSetingstencildata.arm_detect_pose[3];  // 修改
        arm_detect_pose[4] = qWritePalletizingSetingstencildata.arm_detect_pose[4];  // 修改
        arm_detect_pose[5] = qWritePalletizingSetingstencildata.arm_detect_pose[5];  // 修改
      }
      else
      {
        qDebug() << "arm_detect_pose not found in YAML!";
      }
      break;
    case 3:
      if (qWritePalletizingSetingstencildata.arm_motion_place.isEmpty())
      {
        qDebug() << "yaml error!!";
        return;
      }
      // 检查是否存在 arm_motion_place
      if (config["mobile_manipulation"]["arm_motion_place"])
      {
        // 获取原始的 arm_motion_place 值
        YAML::Node arm_motion_place = config["mobile_manipulation"]["arm_motion_place"];
        // 修改 arm_motion_place 的值
        arm_motion_place[0] = qWritePalletizingSetingstencildata.arm_motion_place[0];  // 修改
        arm_motion_place[1] = qWritePalletizingSetingstencildata.arm_motion_place[1];  // 修改
        arm_motion_place[2] = qWritePalletizingSetingstencildata.arm_motion_place[2];  // 修改
        arm_motion_place[3] = qWritePalletizingSetingstencildata.arm_motion_place[3];  // 修改
        arm_motion_place[4] = qWritePalletizingSetingstencildata.arm_motion_place[4];  // 修改
        arm_motion_place[5] = qWritePalletizingSetingstencildata.arm_motion_place[5];  // 修改
      }
      else
      {
        qDebug() << "arm_detect_pose not found in YAML!";
      }
      break;
    case 4:
      // 检查是否存在 place_marker_id
      if (config["mobile_manipulation"]["place_marker_id"])
      {
        // 获取原始的 place_marker_id 值
        YAML::Node place_marker_id = config["mobile_manipulation"]["place_marker_id"];
        // 修改 arm_motion_place 的值
        place_marker_id = qWritePalletizingSetingstencildata.place_marker_id;  // 修改
      }
      else
      {
        qDebug() << "place_marker_id not found in YAML!";
      }
      break;
    case 5:
      // 检查是否存在 placement
      if (config["mobile_manipulation"]["placement"])
      {
        // 获取原始的 place_marker_id 值
        YAML::Node placement = config["mobile_manipulation"]["placement"];
        // 修改 placement 的值
        placement = qWritePalletizingSetingstencildata.placement;  // 修改
      }
      else
      {
        qDebug() << "placement not found in YAML!";
      }
      break;
    case 6:
      // 检查是否存在 placement
      if (config["sorting"]["detect_method"])
      {
        // 获取原始的 place_marker_id 值
        YAML::Node detect_method = config["sorting"]["detect_method"];
        // 修改 placement 的值
        detect_method = qWritePalletizingSetingstencildata.detect_method;  // 修改
      }
      else
      {
        qDebug() << "placement not found in YAML!";
      }
      break;
    case 7:
      // 检查是否存在 placement
      if (config["mobile_manipulation"]["move_forward_distance"])
      {
        // 获取原始的 place_marker_id 值
        YAML::Node move_forward_distance = config["mobile_manipulation"]["move_forward_distance"];
        // 修改 placement 的值
        move_forward_distance = qWritePalletizingSetingstencildata.move_forward_distance;  // 修改
      }
      else
      {
        qDebug() << "placement not found in YAML!";
      }
      break;
    default:
      break;
  }
  // 保存修改后的 YAML 到文件
  std::ofstream fout(path.toStdString());
  if (fout.is_open())
  {
    fout << config;
    fout.close();
  }
  else
  {
    qDebug() << "Error opening file for writing!";
  }
#endif
}

void PalletizingSetingInit::readYaml(QString path)
{
  // pass
  // 更新表格
  // pick_pose: [-8.112,12.111,0]
  // place_pose: [-11.302, 14.169,180]
  // arm_detect_pose: [0.6, 0, 0.85, 180,0,0]
  // arm_motion_place: [0.4, 0, 0.70, 180, 0, 0]
  // place_marker_id: 0
  // placement: 1 # 1: 2*2*2 cube; 2: 3*2*1 triangle
  // path = "/home/robot/ros_qt_ws/src/mobile_manipulation/param/mobile_manipulation.yaml";
#if YAML_CPP
  // 读取并解析YAML文件
  YAML::Node config = YAML::LoadFile(path.toStdString());
  // pick_pose
  ROS_INFO("read yaml________---");
  YAML::Node pickPose = config["mobile_manipulation"]["pick_pose"];
  std::cout<<"pose size"<<pickPose<<endl;
  if (pickPose.size() == 3)
  {
    for (int i = 0; i < pickPose.size(); i++)
    {
      qPalletizingSetingstencildata.pick_pose.append(pickPose[i].as<double>());
    }
  }
  else
  {
    // 处理错误或不合法的 YAML 数据
    std::cout<< "Error: Invalid pick_pose in YAML file."<<endl;
    return;
  }
  // qDebug() << "Qpick Pose: " << qPalletizingSetingstencildata.pick_pose;
  // std::cout << "Ppick Pose: " << pickPose << std::endl;

  // place_pose
  YAML::Node placePose = config["mobile_manipulation"]["place_pose"];
  if (placePose.size() == 3)
  {
    for (int i = 0; i < placePose.size(); i++)
    {
      qPalletizingSetingstencildata.place_pose.append(placePose[i].as<double>());
    }
  }
  else
  {
    // 处理错误或不合法的 YAML 数据
    qDebug() << "Error: Invalid place_pose in YAML file.";
    return;
  }
  // qDebug() << "QPlace Pose: " << qPalletizingSetingstencildata.place_pose;
  // std::cout << "Place Pose: " << placePose << std::endl;
  // 获取arm_detect_pose的值
  YAML::Node armDetectPose = config["mobile_manipulation"]["arm_detect_pose"];
  if (armDetectPose.size() == 6)
  {
    for (int i = 0; i < armDetectPose.size(); i++)
    {
      qPalletizingSetingstencildata.arm_detect_pose.append(armDetectPose[i].as<double>());
    }
  }
  else
  {
    // 处理错误或不合法的 YAML 数据
    qDebug() << "Error: Invalid arm_detect_pose in YAML file.";
    return;
  }
    ROS_INFO("22222222222");
  // qDebug() << "QArm Detect Pose: " << qPalletizingSetingstencildata.arm_detect_pose;
  // std::cout << "Arm Detect Pose: " << armDetectPose << std::endl;
  // 获取arm_motion_place的值
  YAML::Node armHomePose = config["mobile_manipulation"]["arm_home_pose"];
  if (armHomePose.size() == 6)
  {
    for (int i = 0; i < armHomePose.size(); i++)
    {
      qPalletizingSetingstencildata.arm_home_pose.append(armHomePose[i].as<double>());
    }
  }
  else
  {
    // 处理错误或不合法的 YAML 数据
    qDebug() << "Error: Invalid arm_home_pose in YAML file.";
    return;
  }
    YAML::Node armPlacePose = config["mobile_manipulation"]["arm_place_pose"];
  if (armPlacePose.size() == 6)
  {
    for (int i = 0; i < armPlacePose.size(); i++)
    {
      qPalletizingSetingstencildata.arm_place_pose.append(armPlacePose[i].as<double>());
    }
  }
  else
  {
    // 处理错误或不合法的 YAML 数据
    qDebug() << "Error: Invalid arm_place_pose in YAML file.";
    return;
  }
  // qDebug() << "QArm Motion Place: " << qPalletizingSetingstencildata.arm_motion_place;
  // std::cout << "Arm Motion Place: " << armMotionPlace << std::endl;
  // 获取place_marker_id的值
  // int placeMarkerId = config["mobile_manipulation"]["place_marker_id"].as<int>();
  // qPalletizingSetingstencildata.place_marker_id = placeMarkerId;
  // // qDebug() << "QPlace Marker ID: " << qPalletizingSetingstencildata.place_marker_id;
  // // std::cout << "Place Marker ID: " << placeMarkerId << std::endl;
  // // 获取placement的值
  // int placement = config["mobile_manipulation"]["placement"].as<int>();
  // qPalletizingSetingstencildata.placement = placement;
  // // qDebug() << "Qplacement: " << qPalletizingSetingstencildata.placement;
  // // std::cout << "placement: " << placement << std::endl;
  // int detect_method = config["sorting"]["detect_method"].as<int>();
  // qPalletizingSetingstencildata.detect_method = detect_method;

  // float move_forward_distance = config["mobile_manipulation"]["move_forward_distance"].as<float>();
  // qDebug() << move_forward_distance;
  // qPalletizingSetingstencildata.move_forward_distance = move_forward_distance;
  // 更新表格
  QString _temp;
  //qPalletizingSetingstencildata.pick_pose[0]
  _temp = QString("[%1, %2, %3]")
              .arg(1.0)
              .arg(2.0)
              .arg(3.0);
  // qDebug() << temp;
  ROS_INFO("123123");
  this->editCell(_temp, 0, 0);
  _temp = QString("[%1, %2, %3]")
              .arg(qPalletizingSetingstencildata.place_pose[0])
              .arg(qPalletizingSetingstencildata.place_pose[1])
              .arg(qPalletizingSetingstencildata.place_pose[2]);
  // qDebug() << _temp;
  this->editCell(_temp, 1, 0);
  _temp = QString("[%1, %2, %3, %4, %5,%6]")
              .arg(qPalletizingSetingstencildata.arm_detect_pose[0])
              .arg(qPalletizingSetingstencildata.arm_detect_pose[1])
              .arg(qPalletizingSetingstencildata.arm_detect_pose[2])
              .arg(qPalletizingSetingstencildata.arm_detect_pose[3])
              .arg(qPalletizingSetingstencildata.arm_detect_pose[4])
              .arg(qPalletizingSetingstencildata.arm_detect_pose[5]);
  // qDebug() << _temp;
  this->editCell(_temp, 2, 0);

    _temp = QString("[%1, %2, %3, %4, %5,%6]")
              .arg(qPalletizingSetingstencildata.arm_home_pose[0])
              .arg(qPalletizingSetingstencildata.arm_home_pose[1])
              .arg(qPalletizingSetingstencildata.arm_home_pose[2])
              .arg(qPalletizingSetingstencildata.arm_home_pose[3])
              .arg(qPalletizingSetingstencildata.arm_home_pose[4])
              .arg(qPalletizingSetingstencildata.arm_home_pose[5]);
  // qDebug() << _temp;
  this->editCell(_temp, 3, 0);

    _temp = QString("[%1, %2, %3, %4, %5,%6]")
              .arg(qPalletizingSetingstencildata.arm_place_pose[0])
              .arg(qPalletizingSetingstencildata.arm_place_pose[1])
              .arg(qPalletizingSetingstencildata.arm_place_pose[2])
              .arg(qPalletizingSetingstencildata.arm_place_pose[3])
              .arg(qPalletizingSetingstencildata.arm_place_pose[4])
              .arg(qPalletizingSetingstencildata.arm_place_pose[5]);
  // qDebug() << _temp;
  this->editCell(_temp, 4, 0);
  // _temp = QString::number(qPalletizingSetingstencildata.place_marker_id, 10);
  // this->editCell(_temp, 4, 0);
  // _temp = QString::number(qPalletizingSetingstencildata.placement, 10);
  // this->editCell(_temp, 5, 0);
  // _temp = QString::number(qPalletizingSetingstencildata.detect_method, 10);
  // this->editCell(_temp, 6, 0);
  // _temp = QString::number(qPalletizingSetingstencildata.move_forward_distance);
  // this->editCell(_temp + " m", 7, 0);
  // // writeYaml("6", 0);
#endif
}
void PalletizingSetingInit::editCell(QString Messager, int row, int col)
{
  // 编辑单元格
  // 居中
  //  1、获取单元格指针
  QTableWidgetItem* item = ui->palletizing_tableWidget->item(row, col);
  // 判断指针是否空
  if (!item)  // 如果单元格不存在，创建一个新的
  {
    item = new QTableWidgetItem();
    item->setTextAlignment(Qt::AlignCenter);
    ui->palletizing_tableWidget->setItem(row, col, item);
  }
  // if (item)
  // {
  // 2、设置新内容
  item->setText(Messager);
  //}
}
void PalletizingSetingInit::initConnect()
{
  connect(ui->palletizing_tableWidget, &QTableWidget::cellClicked, [&](int row, int column) {
    QTableWidgetItem* item = ui->palletizing_tableWidget->item(row, column);
    if (item)
    {

      std::cout<< "Clicked on Row:" << row << "Column:" << column;
      // std::cout << "Item Text:" << item->text();
      if ("get" == item->text())
      {
        // 建议触发一个信号
        //  emit myClicked(row,column);
        // 直接更新内容
//
#if ROSFLAG
        QString _temp;
        tf::Vector3 position;
        bool readFlag = true;
        // TODO:弃用
        if (0 == row && 2 == column)
        {
          geometry_msgs::Pose pose;
          this->getRobotPose(pose);
          
          double yaw = tf::getYaw(pose.orientation);
          yaw = this->NormalizeAngle(yaw);
          qDebug() << pose.position.x << pose.position.y << yaw;
          // qDebug()<<pose.theta();
          // read_pose(readFlag, position);
          if (readFlag)
          {
            // 清除数据
            this->qWritePalletizingSetingstencildataClear();
            this->qWritePalletizingSetingstencildata.pick_pose.append(QString::number(pose.position.x).toDouble());
            this->qWritePalletizingSetingstencildata.pick_pose.append(QString::number(pose.position.y).toDouble());
            this->qWritePalletizingSetingstencildata.pick_pose.append(QString::number(yaw).toDouble());

            _temp = QString("[%1, %2, %3]")
                        .arg(this->qWritePalletizingSetingstencildata.pick_pose[0])
                        .arg(this->qWritePalletizingSetingstencildata.pick_pose[1])
                        .arg(this->qWritePalletizingSetingstencildata.pick_pose[2]);
          }
          this->editCell(_temp, row, column - 1);
          this->editCell("未保存", row, column + 2);
        }
        else if (1 == row && 2 == column)
        {
          geometry_msgs::Pose pose;
          this->getRobotPose(pose);
          double yaw = tf::getYaw(pose.orientation);
          yaw = this->NormalizeAngle(yaw);
          qDebug() << pose.position.x << pose.position.y << yaw;
          //  read_pose(readFlag, position);
          if (readFlag)
          {
            if (!this->qWritePalletizingSetingstencildata.place_pose.isEmpty())
            {
              this->qWritePalletizingSetingstencildata.place_pose.clear();
            }
            // 添加当前位姿信息
            this->qWritePalletizingSetingstencildata.place_pose.append(QString::number(pose.position.x).toDouble());
            this->qWritePalletizingSetingstencildata.place_pose.append(QString::number(pose.position.y).toDouble());
            this->qWritePalletizingSetingstencildata.place_pose.append(QString::number(yaw).toDouble());
            _temp = QString("[%1, %2, %3]")
                        .arg(this->qWritePalletizingSetingstencildata.place_pose[0])
                        .arg(this->qWritePalletizingSetingstencildata.place_pose[1])
                        .arg(this->qWritePalletizingSetingstencildata.place_pose[2]);
            this->editCell(_temp, row, column - 1);
            this->editCell("未保存", row, column + 2);
          }
        }

#endif

        //
      }
      else if ("save" == item->text())
      {
        //直接触发写入动作
        ROS_INFO("+++++++++++++");

        this->writeYaml(this->yamlPath, row);
        this->editCell("已保存", row, column + 1);
        // //更新表格
        //  this->readYaml(this->yamlPath);
      }
    }
  });
}
#if ROSFLAG
bool topicExists(const std::string& topic_name) {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    for (const auto& info : topic_info) {
        if (info.name == topic_name) {
            return true;
        }
    }
    return false;
}

bool PalletizingSetingInit::getRobotPose(geometry_msgs::Pose& pose)
{
  if (topicExists("/map") ) {
      // ROS_INFO("topic is exits");
      
    } else {
      // ROS_INFO("topic is not exits");
      return false;
    }
  transform_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);

  pose.position.x = transform_.getOrigin().x();
  pose.position.y = transform_.getOrigin().y();
  pose.position.z = transform_.getOrigin().z();
  pose.orientation.x = transform_.getRotation().x();
  pose.orientation.y = transform_.getRotation().y();
  pose.orientation.z = transform_.getRotation().z();
  pose.orientation.w = transform_.getRotation().w();
  return true;
  
}

double PalletizingSetingInit::NormalizeAngle(const double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0)
  {
    a += M_PI;
  }
  else
  {
    a -= M_PI;
  }
  // TODO:与接口统一
  a = (a / M_PI) * 180.0 * (-1);
  return a;
}

#endif
