#include "robusterdemo.h"
#include "palletizingsetinginit.h"
#include "ui_palletizingsetinginit.h"
#include "ui_robusterdemo.h"
#include <QIcon>
#include <QLabel>
#include<unistd.h>
RobusterDemo::RobusterDemo(QWidget* parent) : QMainWindow(parent), ui(new Ui::RobusterDemo)
{
  // init ui
  ui->setupUi(this);
 palletizingSeting();
  setWindowIcon(QIcon(QStringLiteral(":/show/robusterDebuggingAssistant.png")));
  resize(592,230);
  setFixedSize(950, 250);
//检查导航是否打开
    // if (!isNavigationState())
    // {
    //   if (!promptState("请先打开导航功能"))
    //   {
    //   }
    //  exit(0);
    // }



  
  // QLabel* ver = new QLabel("v1.0.1", this);
  // this->statusBar()->addPermanentWidget(ver);
  // mProcess = new QProcess(this);
  // detachedProcess = new QProcess(this);

  // palletizingSetingInit(); //设置button
  // // test
  // // 启动任务，并返回 QFuture
  // //由于码垛，分拣，采摘功能需要启动launch文件，需要重启ros服务
  // //

  
  // this->grasp();
  // // this->goApple();
}



void RobusterDemo::grasp()
{
    QObject::connect(ui->palletizing_toolButton, &QToolButton::clicked, [&]() {
    if (!isNavigationState())
    {
      if (!promptState("请先打开导航功能"))
      {
      }
      return;
    }
    qDebug() << "执行采摘任务";
    // TODO:加入检测
    this->exec_bash_Memory("export MMX5_END_TOOL=robotiq85;echo $MMX5_END_TOOL;~/start_grasp.bash", true);
    sleep(1);
    this->exec_bash_Memory("export MMX5_END_TOOL=robotiq85;echo $MMX5_END_TOOL;~/start_yolo5.bash", true);
    sleep(1);
    this->exec_bash_Memory("export MMX5_END_TOOL=robotiq85;echo $MMX5_END_TOOL;~/start_control.bash", true);
    // this->exec_bash_Memory("roslaunch turtle_tf turtle_tf_demo.launch");
  });
}

void RobusterDemo::process(QString cmd)
{
  mProcess->close();
  mProcess->start("bash", QStringList() << "-c" << cmd);
  if (mProcess->waitForStarted() && mProcess->waitForFinished())
  {
    mProcess->close();
  }
}

QByteArray RobusterDemo ::exec_bash_return(QString bash)
{
  QProcess* executeProcess = new QProcess(this);
  QByteArray byteArray;
  executeProcess->start("/bin/bash", QStringList() << "-c" << bash);  //阻塞
  // 等待启动完成
  if (!executeProcess->waitForStarted())
  {
    qDebug() << "Failed to start the process.";
  }
  // 等待进程完成，并检查是否有错误
  if (!executeProcess->waitForFinished())
  {
    qDebug() << "Process execution timed out or an error occurred.";
  }
  byteArray = executeProcess->readAllStandardOutput();
  qDebug() << "done";
  return byteArray;
}
void RobusterDemo::exec_bash_Memory(QString bash)
{
  QProcess* executeProcess = new QProcess(this);
  executeProcess->start("/bin/bash", QStringList() << "-c" << bash);  //阻塞
  // 等待启动完成
  if (!executeProcess->waitForStarted())
  {
    qDebug() << "Failed to start the process.";
    return;
  }
  // 等待进程完成，并检查是否有错误
  if (!executeProcess->waitForFinished())
  {
    qDebug() << "Process execution timed out or an error occurred.";
    return;
  }
  QByteArray byteArray = executeProcess->readAllStandardOutput();
  if (!byteArray.isEmpty() && byteArray.endsWith('\n'))
    byteArray.chop(1);
  qDebug() << byteArray.data();
  qDebug() << "done";
}
void RobusterDemo::exec_bash_Memory(QString bash, bool detached)
{
  // 静态成员初始化标志
  static bool initFlag = false;
  // 初始化静态成员，只需连接一次
  if (!initFlag)
  {
    connect(detachedProcess, &QProcess::readyReadStandardOutput, this, &RobusterDemo::readStandardOutput);
    connect(detachedProcess, &QProcess::readyReadStandardError, this, &RobusterDemo::readStandardError);
    connect(detachedProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
            &RobusterDemo::processFinished);

    initFlag = true;
  }
  // 启动分离式 bash 进程
  if (!detached)
  {
    detachedProcess->startDetached("/bin/bash", QStringList() << "-c" << bash);
    // 等待进程启动
    if (!detachedProcess->waitForStarted())
    {
      qDebug() << "Failed to start the process.";
      return;
    }
    // 等待进程完成，并检查是否有错误
    if (!detachedProcess->waitForFinished())
    {
      qDebug() << "Process execution timed out or an error occurred.";
      return;
    }
  }
  else
  {
    bash += "; exec bash";
    detachedProcess->startDetached("/usr/bin/gnome-terminal", QStringList() << "--"
                                                                            << "bash"
                                                                            << "-c" << bash);
  }
}
// 处理标准输出的槽函数
void RobusterDemo::readStandardOutput()
{
  QByteArray byteArray = detachedProcess->readAllStandardOutput();
  // 去除末尾的换行符
  if (!byteArray.isEmpty() && byteArray.endsWith('\n'))
    byteArray.chop(1);
  qDebug() << byteArray.data();
}
// 处理错误输出的槽函数
void RobusterDemo::readStandardError()
{
  QByteArray byteArray = detachedProcess->readAllStandardError();
  // 去除末尾的换行符
  if (!byteArray.isEmpty() && byteArray.endsWith('\n'))
    byteArray.chop(1);
  qDebug() << byteArray.data();
}
void RobusterDemo::processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
  if (exitStatus == QProcess::NormalExit)
  {
    qDebug() << "Process exited with code: " << exitCode;
  }
  else
  {
    qDebug() << "Process exited with error: " << exitCode;
  }
}
void RobusterDemo::palletizingSeting()
{
  this->yamlPath = "/home/robuster/param.yaml";
  this->palletizingSetingWidget();

}
bool RobusterDemo::isNavigationState()
{
  qDebug() << "Check if navigation now...";
  m_running = false;
  //不返回内容，但是状态码为0为活跃状态，非活跃则返回inactive
  QByteArray isNavigationStateFlag = exec_bash_return("systemctl is-active navigation.service");
  if (isNavigationStateFlag.isNull())
  {
    qDebug() << "error!!";
    // return;
  }
  else
  {
    if (!isNavigationStateFlag.isEmpty() && isNavigationStateFlag.endsWith('\n'))
      isNavigationStateFlag.chop(1);
  }
  qDebug() << isNavigationStateFlag.data();
  QString flag_ = isNavigationStateFlag.data();
  if ("active" == flag_)
  {
    m_running = true;
  }
  else if ("inactive" == flag_)
  {
    m_running = false;
  }

  return m_running;
}
//主窗口的点击设置
void RobusterDemo::palletizingSetingInit()
{
  QObject::connect(ui->seting_toolButton, &QToolButton::clicked, this, &RobusterDemo::palletizingSeting);
}
#if ROSFLAG

void RobusterDemo::getRobotPose(geometry_msgs::Pose& pose)
{
  transform_listener_.lookupTransform("/map", "/base_link", ros::Time(1), transform_);

  pose.position.x = transform_.getOrigin().x();
  pose.position.y = transform_.getOrigin().y();
  pose.position.z = transform_.getOrigin().z();
  pose.orientation.x = transform_.getRotation().x();
  pose.orientation.y = transform_.getRotation().y();
  pose.orientation.z = transform_.getRotation().z();
  pose.orientation.w = transform_.getRotation().w();
}
#endif
RobusterDemo::~RobusterDemo()
{
  delete ui;
  // exec_bash_Memory("echo szsh1234 | sudo -S systemctl restart roscore.service", false);
  //   terminate() 会发送一个终止信号给进程
  // kill() 则会强制结束进程
  detachedProcess->terminate();  // 或者 detachedProcess->kill();
  
}


void RobusterDemo::palletizingSetingWidget()
{
  std::ifstream file(getenv("HOME") + string("/.robuster/maps/control_tasks.yaml"));
  
  PalletizingSetingInit* palletizingSetingInit = new PalletizingSetingInit(this);
  // 设置窗口模态
  palletizingSetingInit->setWindowModality(Qt::ApplicationModal);
  palletizingSetingInit->setWindowTitle("Seting");
  palletizingSetingInit->setFixedSize(950, 250);

  // 显示模态窗口
  palletizingSetingInit->show();
  // QObject::disconnect(palletizingSetingInit,&PalletizingSetingInit::myClicked,nullptr, nullptr);
}
bool RobusterDemo::promptState(QString str)
{
  QMessageBox errorDialog;
  errorDialog.setIcon(QMessageBox::Critical);  // 设置对话框图标为错误图标
  errorDialog.setWindowTitle("Error");         // 设置对话框标题
  errorDialog.setText(str);                    // 设置对话框显示的错误消息
  errorDialog.exec();                          // 显示对话框，阻塞程序执行直到对话框关闭
}
void RobusterDemo::checkNavigationState()
{
  if (!isNavigationState())
  {
    if (!promptState("请先打开导航功能"))
    {
      return;
    }
  }
}
void nodeKill()
{
// /mobile_manipulation
// /aruco_detector
// /xarm
// /aruco_sorting
} 
