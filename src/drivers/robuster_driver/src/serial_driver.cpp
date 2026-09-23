#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <sys/stat.h> 

#include "serial_node.h"
#include "utily.h"
#include "command.h"

#define TIMEOUT 2

double tick_th_multiplier = 0.0;

RobusterDriver::RobusterDriver() {

  ros::NodeHandle private_node("~");

  private_node.param<std::string>("port", port_name_, std::string("/dev/ttyS0"));
  private_node.param<int>("baud_rate", baud_rate_, 115200);
  private_node.param<int>("control_rate", control_rate_, 10);
  private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));
  private_node.param<std::string>("base_type", base_type_, "ackermann");
  private_node.param<std::string>("ackermann_control_type", ackermann_control_type_, "ackermann_drive");  //油门刹车：ackermann_drive 速度角度：cmd_vel 定速：constant_speed

  private_node.param<double>("wheel_radius", wheel_radius_, 0.125); 
  private_node.param<double>("wheel_bias", wheel_bias_, 0.68);
  private_node.param<double>("wheel_base", wheel_base_, 0.80);
  private_node.param<int>("reduction_ratio", reduction_ratio_, 32);
  // The motor parameters are 65536 for MR2000, but the hardware driver has processed it, and actually only 10000
  private_node.param<int>("ticks_per_revo", ticks_per_revolution_, 10000); 

  private_node.param<bool>("publish_odom", publish_odom_, false);
  private_node.param<bool>("publish_odom_trans", publish_odom_trans_, false);

  private_node.param<bool>("use_new_proto", use_new_proto_, false);
  private_node.param<bool>("use_diff_twist", use_diff_twist_, true);

  /**
   * 为兼容 MR1000 2WD 及 4WD 版本，以及 MR500, 额外提供 motion_model 参数.
   * 另外，针对 4WD 设备，也可通过将该参数设置为 "2wd" 强制只使用前面两轮的数据进行里程解算.
   */
  private_node.param<std::string>("motion_model", motion_model_, "4wd");

  ROS_INFO("Robuster base type: %s, motion model: %s", base_type_.c_str(), motion_model_.c_str());
  
  pthread_mutex_init(&mutex_,NULL);
  pthread_condattr_setclock(&attr_, CLOCK_MONOTONIC);
  pthread_cond_init(&cond_, &attr_);

  last_send_time_ = getSysTime(); // initialize 

  serial_ptr_ = nullptr;
  disable_permission_ = false;
  cur_dev_state_.control_mode = 0x0;
  time_count_ = 5;

  tick_to_rad_ = 2 * PI / ticks_per_revolution_ / reduction_ratio_;
  rpm_to_vel_ = 2 * PI * wheel_radius_ / (reduction_ratio_ * 60); 

  tick_th_multiplier = (reduction_ratio_ * ticks_per_revolution_) / (PI * wheel_radius_ * 2 * 5.0); // 30.0: Times per second
  /// TODO: change with realtime vel. 1.6: max vel. 
  tick_threshold_ = (int)(1.6 * tick_th_multiplier); 
  ROS_INFO("tick to rad: [%lf], rpm to vel: [%lf], tick max: [%d]", 
            tick_to_rad_, rpm_to_vel_, tick_threshold_);

  initSerialRingBuffer(ring_buffer_);
  init();
}

RobusterDriver::~RobusterDriver() {
  updateMileage();

  /// TODO: close port
  closeDriver();
}

void RobusterDriver::closeDriver() {
  ROS_INFO("Close robuster driver.");
  int  retry_cnt = 0;

  // 清除灯光状态
  if (cur_dev_state_.front_light != 0 || cur_dev_state_.rear_light != 0) {
    msg_que_.push(Command::setLightStatus(0, 0));
  }

  // 退出时解除控制，取消报警
  disable_permission_ = true;
  while (cur_dev_state_.control_mode != 0x00 && retry_cnt < 10) { // 不可用 0x01 进行判断,避免遥控开启时退出程序报警
    ROS_INFO("Stop control when exiting, cancel alarm");
    msg_que_.push(Command::setDeviceStatus(0x00, 0x07));
    
    while (!msg_que_.empty()) { // 循环等待发送完成
      usleep(100*1000); 
    }
    retry_cnt++;
    usleep(500*1000);
  }

  loop_running_ = false;
  usleep(10*1000);
  if (serial_ptr_ && serial_ptr_->isOpen()) {
    serial_ptr_->close();
    serial_ptr_ = nullptr;
  }
}

int RobusterDriver::waitForResponse(int millisecond) {
  struct timespec outtime;
  clock_gettime(CLOCK_MONOTONIC, &outtime);

  outtime.tv_sec += millisecond / 1000;
  outtime.tv_nsec += (millisecond % 1000) * 1000000;
  pthread_mutex_lock(&mutex_);
  int rtn = pthread_cond_timedwait(&cond_, &mutex_, &outtime);
  if (rtn == ETIMEDOUT) {
    ROS_INFO("Timeout.");
  } 
  pthread_mutex_unlock(&mutex_);

  return rtn;
}

void RobusterDriver::emitCondSignal() {
  pthread_mutex_lock(&mutex_);
  pthread_cond_signal(&cond_);
  pthread_mutex_unlock(&mutex_);
}

bool RobusterDriver::init() {

  dev_data_dir_ = getenv("HOME") + string("/.robuster/");
  if (access(dev_data_dir_.c_str(), F_OK) != 0) {
    if(mkdir(dev_data_dir_.c_str(), S_IRWXU|S_IRWXG|S_IRWXO)) {
      ROS_ERROR("Create dir[%s] failed.", dev_data_dir_.c_str());
      return false;
    }
  }
  dev_data_file_ = dev_data_dir_ + "dev_data.yaml";

  cur_mileage_state_.total_mileage = getCurMileage();
  last_mileage_ = cur_mileage_state_.total_mileage;

  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &RobusterDriver::cmdCallback, this);
  acker_sub_ = nh_.subscribe<robuster_mr_msgs::AckermannDriveStamped>("ackermann_cmd", 5, &RobusterDriver::ackerCmdCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

  mileage_state_pub_ = nh_.advertise<robuster_mr_msgs::MileageState>("mileage_state", 10);

  dev_state_pub_ = nh_.advertise<robuster_mr_msgs::DeviceState>("device_state", 10);
  battery_state_pub_ = nh_.advertise<robuster_mr_msgs::BatteryState>("battery_state", 10);
  // aberrant_pub_ = nh_.advertise<>("aberrant", 10);
  light_state_pub_ = nh_.advertise<robuster_mr_msgs::LightState>("light_state", 10);
  motor_state_pub_ = nh_.advertise<robuster_mr_msgs::MotorState>("motor_state", 10);

  wheel_vel_pub_ = nh_.advertise<robuster_mr_msgs::WheelVel>("wheel_info", 10);
  get_devinfo_srv_ = nh_.advertiseService("/robuster/get_devInfo", &RobusterDriver::getDevInfoCallback, this);
  light_control_srv_ = nh_.advertiseService("/robuster/light_control", &RobusterDriver::lightControlCallback, this);
  err_dis_srv_ = nh_.advertiseService("/robuster/disable_error", &RobusterDriver::setErrorMask, this);
  new_light_control_ = false;

  loop_running_ = true;

  boost::thread recv_thread(boost::bind(&RobusterDriver::spin, this));
  boost::thread handle_thread(boost::bind(&RobusterDriver::handleData, this));
  boost::thread send_thread(boost::bind(&RobusterDriver::sendControlCmd, this));

  publish_data_timer_ = nh_.createTimer(ros::Duration(1/30.0), &RobusterDriver::publishData, this);
}

void RobusterDriver::showData(const char *data, int size, const char *prefix) {
  if (NULL == data || !size) {
    return ;
  }

  if (prefix) {
    printf("%s ", prefix);
  }

  printf("Data(%dBytes): ", size);
  for (int i = 0; i < size; i++) {
    printf("%#x ", (uint8_t)data[i]);
  }
  printf("\n");
}

void RobusterDriver::paraseData(char *data, int size) {
  // showData(data, size);

  if (size < 6) {
    ROS_WARN_THROTTLE(5, "Incomplete data segment. Drop.");
    return ;
  } 

  if (data[2] != 0x7f) { // 地址域当前固定为 0x7f
    ROS_INFO("paraseData serial id. 0x%x", data[2]);
    return ;
  }
	  
  uint16_t data_len = (uint16_t)((uint8_t)data[5]<<8 | (uint8_t)data[4]);
  if (data_len + 8 > size) {
    ROS_WARN("Incomplete data length. Drop");
    return ;
  }

  switch (static_cast<uint8_t>(data[3]))
  {
    case DEVICE_STATUS:
      /* handle device status */
      handleDevStatusData(data, size);
      break;
    case ERROR_STATUS:
      /* handle error status */
      handleErrStatusData(data, size);
      break;
    case LIGHT_STATUS:
      handleLightStatusData(data, size);
      break;
    case ODOMETRY_DATA:
      /* handle odometry message */
      handleOdomData(data, size);
      break;
    case VEL_STATUS:
      handleVelData(data, size);
      break;
    case MOTOR_TICK_STATUS:
      handleMotorTickStatus(data, size);
      break;
    case MOTOR_RPM_STATUS:
      handleMotorRPMStatus(data, size);
      break;
    case MOTOR_V_STATUS:
      handleMotorVoltageStatus(data, size);
      break;
    case MOTOR_A_STATUS:
      handleMotorCurrentStatus(data, size);
      break;
    case MOTOR_T_STATUS:
      handleMotorTemperatureStatus(data, size);
      break;
    case MOTOR_E_STATUS:
      handleMotorErrorStatus(data, size);
      break;
    /* battery data */
    case BATTERY_STATUS: 
      handleBatteryStatusData(data, size);
      break;
    case BATTERY_CAP_STATUS:
      handleBatteryCapStatusData(data, size);
      break;
    case BATTERY_VA_STATUS:
      handleBatteryVAStatusData(data, size);
    case CMD_STATUS:
      handleCmdStatusData(data, size);
      break;
    default:
      break;
  }
}

void RobusterDriver::spin() {
  while(ros::ok() && loop_running_) {
    /******************************************
     * Checking Connection
     * If connection break,than reconnect
     *******************************************/
    if (!serial_ptr_ || !serial_ptr_->isOpen()) {
      try {
        serial_ptr_ = SerialPtr(new serial::Serial(port_name_, baud_rate_, serial::Timeout::simpleTimeout(TIMEOUT)));
        ROS_INFO("Open port(%s) success.", port_name_.c_str());

        msg_que_.push(Command::setDeviceStatus(0x01, 0x0f)); 

      } catch (serial::IOException& e) {
        ROS_ERROR("Open port(%s) failed.Retry", port_name_.c_str());
        ros::Duration(3).sleep();
        continue;
      }
    } else {
      try {
        if (serial_ptr_ && serial_ptr_->waitReadable()) {
          int size = serial_ptr_->read(buffer, sizeof(buffer));
          bool ret = writeSerialRingBuffer(ring_buffer_, buffer[0]);
          if (size <= 0) {
            ROS_INFO("read data from serial failed.");
          } else {
            //ROS_INFO("Recveived %dBytes datas", size);
            //paraseData((char *)buffer, size);
          }
              
        }
      } catch (serial::IOException& e) {
        ROS_INFO("Serial read failed. %s", e.what());
        serial_ptr_->close();
      }
    }
  }
}

void RobusterDriver::handleData() {
  while(ros::ok() && loop_running_) {
    dataStream(); 
    ros::Duration(1.0/11520).sleep();  
  }
}

void RobusterDriver::dataStream(void) { 
  static int recv_flag = 0;
  static int offset = 6;
  uint8_t data = 0;

  if(readSerialRingBuffer(ring_buffer_, &data)) {      /* 串口数据接收后处理 */
    switch(recv_flag) {
      case 0: {
        if(data == 0xFF) {
          handler_buf[0] = data;
          recv_flag = 1;
        }
        else {
          recv_flag = 0;
        }
        break;
      }
      case 1: {
        if(data == 0xAA) {
          handler_buf[1] = data;
          recv_flag = 2;
        }
        else {
          recv_flag = 0;
        }
        break;
      }    
      case 2: {
        handler_buf[2] = data;       //从机地址
        recv_flag = 3;
        break;
      }
      case 3: {
        handler_buf[3] = data;       //功能码
        recv_flag = 4;
        break;
      }         
      case 4: {
        handler_buf[4] = data;       //数据段字节数L
        recv_flag = 5;
        break;
      }       
      case 5: {
        handler_buf[5] = data;       //数据段字节数H
        recv_flag = 6;
        break;
      }     
      case 6: {
        handler_buf[offset] = data;
        uint16_t data_len = (uint16_t)(handler_buf[5]<<8 | handler_buf[4]);
        if(data_len < 1 || data_len > DATA_LEN_MAX || offset > sizeof(handler_buf)) {
		  ROS_INFO_THROTTLE(10, "error data_len: %d, offset: %d", data_len,offset);
          recv_flag = 0;
          offset = 6;
          break;
        }
        else if(offset >= (6 + data_len-1)) {
          recv_flag = 7;
        }
        offset++;
        break;
      }
      case 7: {
        handler_buf[offset] = data;       //校验值L
        offset ++;
        recv_flag = 8;
        break;
      }
      case 8: {
        handler_buf[offset] = data;       //校验值H
        // for(int i=0; i<= offset; i++) {
        //   printf(" 0x%x ,",handler_buf[i]);          
        // }
        // printf("\n");
        uint16_t checksum =(uint16_t) (handler_buf[offset]<<8 | handler_buf[offset-1]);  
        std::vector<uint8_t> tmp(handler_buf, handler_buf + (offset - 1));
        //std::cout<<"<dataStream> CRC: "<<getChecksum(tmp)<<", "<< checksum <<std::endl;
        if(getChecksum(tmp) == checksum){
          paraseData((char *)handler_buf, offset+1);         //数据处理
         // memset(handler_buf, 0, sizeof(handler_buf));   
        }
        else {
          ROS_WARN("Incomplete data cheksum. Drop");
         // memset(handler_buf, 0, sizeof(handler_buf)); 
        }
        recv_flag = 0;
        offset = 6;
        break;
      }
      default: {
        assert(false);
      }
    }    
  }

}

vector<uint8_t> RobusterDriver::sendSpeedSteeringCmd(geometry_msgs::Twist cur_twist) {
  vector<uint8_t> msg;
  float x = cur_twist.linear.x;
  float z = cur_twist.angular.z;
  float L = wheel_base_;     //轴距
  float T = wheel_bias_;     //轮间距
  float angle = 0.0;

  if(x != 0 && z != 0) {
    float R = fabs(x/z);
    angle = atan2(L, R);
  }
  msg = Command::setThrottleSteering(0x06, 0.0, 0.0, cur_twist.linear.x, 31.4, angle);     // 31.4: 10*pi rad/s

  return msg;
}

vector<uint8_t> RobusterDriver::setAckermannSpeed(robuster_mr_msgs::AckermannDriveStamped cur_twist) {
  vector<uint8_t> msg;
  float max_speed = 3.063;      //m/s
  float max_acc = 0.5105;        //m/s2
  float max_break_acc = 1.021;     //m/s2
  uint8_t gear_position = 0x1A;     //前进D档
  uint16_t throttle = 0;
  uint16_t brake_strength = 0;
  float speed = 0.0;
  float steering_angular = cur_twist.drive.steering_angle;
  float steering_angle_velocity = 31.4;
  float L = wheel_base_;     //轴距
  float T = wheel_bias_;     //轮间距

  if(cur_twist.drive.speed > 0) {
    gear_position = 0x1A;   //前进D档
  }
  else if(cur_twist.drive.speed < 0) {
    gear_position = 0x15;  //倒车R档
  }
  else {
    gear_position = 0x10;  //空档N
  }

  if(cur_twist.drive.speed>max_speed || cur_twist.drive.speed< -max_speed) {    //设置范围
    if(cur_twist.drive.speed>max_speed) {
      cur_twist.drive.speed= max_speed;
    }
    else if(cur_twist.drive.speed< -max_speed) {
      cur_twist.drive.speed = -max_speed;
    }
  }
  else {  //速度计算油门千分比
    if(cur_twist.drive.speed > 0) {
      throttle = (cur_twist.drive.speed / max_speed) * 1000;
    }
    else if(cur_twist.drive.speed < 0) {
      throttle = (cur_twist.drive.speed / (-max_speed)) * 1000;
    }
    else {
      throttle = 0;
    }
  }

  if(cur_twist.drive.acceleration != 0) {   //加速度计算油门千分比 
    throttle = (cur_twist.drive.acceleration / max_acc) *1000;
  }

  if(cur_twist.drive.jerk != 0) {           //加速度计算刹车千分比 
    brake_strength = (cur_twist.drive.jerk / max_break_acc) * 1000;
  }


  if(cur_twist.drive.steering_angle_velocity != 0) {
    steering_angle_velocity = cur_twist.drive.steering_angle_velocity;
  }
  else {
    steering_angle_velocity = 31.4;
  }

  msg = Command::setThrottleSteering(gear_position , throttle, brake_strength, speed, steering_angle_velocity, steering_angular);     // 31.4: 10*pi rad/s

  return msg;
}

void RobusterDriver::sendControlCmd() {
  while(loop_running_) {
      vector<uint8_t> msg;

    if (serial_ptr_ && serial_ptr_->isOpen()) {
      /// 确保已获取权限. TODO: 解决串口通信问题后,可去除,防止遥控控制时循环发送获取权限指令
      if (cur_dev_state_.control_mode != 0x01 && !disable_permission_) { // 0x01: 即驱动控制权限; 0x02: 表示遥控权限
        msg = Command::setDeviceStatus(0x01, 0x0f); 
        std::queue<vector<uint8_t>> empty;
        std::swap(msg_que_, empty); // 防止指令堆积
      } else {
        if (!msg_que_.empty()) { // 控制命令
          msg = msg_que_.front();
          msg_que_.pop();
        } else {
          // ROS_INFO("Time duration %lf", (ros::Time::now() - last_twist_time_).toSec());
          if ((ros::Time::now() - last_twist_time_).toSec() <= 0.5) {
            // 非麦克纳姆轮模型时清除 y 轴速度
            if(strstr(base_type_.c_str(), "mecanum") == NULL) {
              current_twist_.linear.y = 0.0; 
            }
          } else { // 控制超时，停止运动
            current_twist_.linear.x = 0.0;
            current_twist_.linear.y = 0.0;
            current_twist_.angular.z = 0.0;
          }
          if(strstr(base_type_.c_str(), "ackermann") != NULL) {
            if(strstr(ackermann_control_type_.c_str(), "ackermann_drive") != NULL) {    // 油门刹车：ackermann_drive 速度角度：cmd_vel 定速：constant_speed
              msg = setAckermannSpeed(current_acker_);
              //msg = Command::setThrottleSteering(current_acker_.gear_position, current_acker_.acceleration, current_acker_.jerk, current_acker_.speed, current_acker_.steering_angle_velocity, current_acker_.steering_angle);
            }
            else if (strstr(ackermann_control_type_.c_str(), "constant_speed") != NULL) {
              msg = sendSpeedSteeringCmd(current_twist_);
            }
            else {
	      if(time_count_ >= 1) {
                time_count_--;
                msg = Command::setThrottleSteering(0x00, 0, 0, 0, 0, 0);
              }
              else {
                msg = Command::setVelocityControl(current_twist_.linear.x, current_twist_.angular.z);
	      }
            }
          }
          else {
            if (use_new_proto_) {
              msg = Command::setVelocityControl(current_twist_.linear.x, current_twist_.linear.y, current_twist_.angular.z);
            } else {
              msg = Command::setVelocityControl(current_twist_.linear.x, current_twist_.angular.z);
            }
          }
        }
      }

      if (!sendCommand(msg)) {
        ROS_WARN_THROTTLE(10, "Write serial failed.");
      }
    }

    ros::Duration(1.0/control_rate_).sleep();
  }
}

void RobusterDriver::cmdCallback(const geometry_msgs::Twist::ConstPtr& vel) {
  twist_mutex_.lock();
  last_twist_time_ = ros::Time::now();
  current_twist_ = *(vel.get());
  twist_mutex_.unlock();
}

void RobusterDriver::ackerCmdCallback(const robuster_mr_msgs::AckermannDriveStamped::ConstPtr& vel) {
  acker_mutex_.lock();
  last_acker_time_ = ros::Time::now();
  current_acker_ = *(vel.get());
  acker_mutex_.unlock();
}

void RobusterDriver::publishWheelInfo(double left_front, double left_rear, double right_front, double right_rear) {
  robuster_mr_msgs::WheelVel msg;
  msg.left_front_wheel = left_front;
  msg.left_rear_wheel = left_rear;
  msg.right_front_wheel = right_front;
  msg.right_rear_wheel = right_rear;

  wheel_vel_pub_.publish(msg);
}

void RobusterDriver::publishOdometry(double x, double y, double thelta, double vel_x, double vel_thelta) {
  ros::Time cur_time = ros::Time::now();

  cur_dev_state_.linear_velocity = vel_x;
  cur_dev_state_.angular_velocity = vel_thelta;

  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(thelta);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = cur_time;
  odom_trans.header.frame_id = odom_frame_;
  odom_trans.child_frame_id = base_frame_;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = orientation;

  /* Don't need odom transform for 3D SLAM */
  if (publish_odom_trans_) {
    odom_broadcaster_.sendTransform(odom_trans);
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = cur_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = orientation;

  //set the velocity
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vel_thelta;

  odom.pose.covariance[0]  = 0.1;   
  odom.pose.covariance[7]  = 0.1;	
  odom.pose.covariance[35] = 0.2;   

  odom.pose.covariance[14] = 1e10; 	
  odom.pose.covariance[21] = 1e10; 	
  odom.pose.covariance[28] = 1e10; 
  
  if (publish_odom_) {
    odom_pub_.publish(odom);
  }
}

double RobusterDriver::getCurMileage() {
  // check if file exist or not
  if (access(dev_data_file_.c_str(), F_OK ) != 0) {
    ROS_WARN_THROTTLE(10, "Device data file [%s] doesn't exist.Then enforce update this file.", dev_data_file_.c_str());
    updateMileage(true);
    return 0.0;
  }

  YAML::Node dev_data = YAML::LoadFile(dev_data_file_);
  if (dev_data["total_mileage"]) {
    return dev_data["total_mileage"].as<double>();
  }
  else {
    return 0.0;
  }
}

void RobusterDriver::updateMileage(bool enforce) {
  
  if (!enforce && fabs(cur_mileage_state_.total_mileage - last_mileage_) < 0.05) {
    return ;
  }
  last_mileage_ = cur_mileage_state_.total_mileage;

  std::ofstream file(dev_data_file_);
  if (!file.is_open()) {
    ROS_ERROR("Open %s failed.", dev_data_file_.c_str());
    return ;
  }

  YAML::Node dev_data;
  dev_data["total_mileage"] = cur_mileage_state_.total_mileage;

  file << dev_data;
  file.close();
}

void RobusterDriver::publishMileage(double ds) {
  cur_mileage_state_.total_mileage += fabs(ds);
  cur_mileage_state_.relative_mileage += fabs(ds);
  mileage_state_pub_.publish(cur_mileage_state_);
}

void RobusterDriver::updateOdometry(void) {
  // cur_motor_state_
  static bool init = false;
  static ros::Time last_time;

  if (!init) {
    /// TODO: wait for motor state from mobile base.
    last_motor_state_ = cur_motor_state_;
    init = true;

    last_time = ros::Time::now();
  }

  double dt = ros::Time::now().toSec() - last_time.toSec();
  if (dt < 0.00001) {
    return ;
  }
  last_time = ros::Time::now();
  
  int32_t lf_diff_ticks = cur_motor_state_.motor_states[LF_MOTOR_ID].tick 
                              - last_motor_state_.motor_states[LF_MOTOR_ID].tick;
  int32_t lb_diff_ticks = cur_motor_state_.motor_states[LB_MOTOR_ID].tick 
                              - last_motor_state_.motor_states[LB_MOTOR_ID].tick;
  int32_t rf_diff_ticks = cur_motor_state_.motor_states[RF_MOTOR_ID].tick 
                              - last_motor_state_.motor_states[RF_MOTOR_ID].tick;
  int32_t rb_diff_ticks = cur_motor_state_.motor_states[RB_MOTOR_ID].tick 
                              - last_motor_state_.motor_states[RB_MOTOR_ID].tick;
 // ROS_INFO("[valied] last ticks: [%d, %d, %d, %d], cur ticks:[%d, %d, %d, %d]",
 //     last_motor_state_.motor_states[LF_MOTOR_ID].tick, last_motor_state_.motor_states[RF_MOTOR_ID].tick,
 //     last_motor_state_.motor_states[LB_MOTOR_ID].tick, last_motor_state_.motor_states[RB_MOTOR_ID].tick,
 //     cur_motor_state_.motor_states[LF_MOTOR_ID].tick, cur_motor_state_.motor_states[RF_MOTOR_ID].tick,
 //     cur_motor_state_.motor_states[LB_MOTOR_ID].tick, cur_motor_state_.motor_states[RB_MOTOR_ID].tick);
 // ROS_INFO("[Invalied] diff ticks: [%d, %d, %d, %d]", lf_diff_ticks,lb_diff_ticks,rf_diff_ticks,rb_diff_ticks);  
  if (abs(lf_diff_ticks) > tick_threshold_ || abs(lb_diff_ticks) > tick_threshold_
      || abs(rf_diff_ticks) > tick_threshold_ || abs(rb_diff_ticks) > tick_threshold_) {
    ROS_WARN("Encoder value invalied.Drop.");
    ROS_INFO("[Invalied] last ticks: [%d, %d, %d, %d], cur ticks:[%d, %d, %d, %d]",
      last_motor_state_.motor_states[LF_MOTOR_ID].tick, last_motor_state_.motor_states[RF_MOTOR_ID].tick,
      last_motor_state_.motor_states[LB_MOTOR_ID].tick, last_motor_state_.motor_states[RB_MOTOR_ID].tick,
      cur_motor_state_.motor_states[LF_MOTOR_ID].tick, cur_motor_state_.motor_states[RF_MOTOR_ID].tick,
      cur_motor_state_.motor_states[LB_MOTOR_ID].tick, cur_motor_state_.motor_states[RB_MOTOR_ID].tick);
      last_motor_state_ = cur_motor_state_;
    return ;
  }
  last_motor_state_ = cur_motor_state_;

  double dist_lf = 0.0; 
  double dist_lb = 0.0;
  double dist_rf = 0.0;
  double dist_rb = 0.0;
  double ds = 0.0;
  double domega = 0.0;
  if (motion_model_ == "2wd") {
    if(strstr(base_type_.c_str(), "ackermann") != NULL) {
      dist_lb = wheel_radius_ * tick_to_rad_ * lb_diff_ticks; 
      dist_rb = wheel_radius_ * tick_to_rad_ * rb_diff_ticks;
      ds = (dist_lb + dist_rb) * 0.5;
      //TODO :ackermann不是这样计算的
      // 前轮没有运动，直接使用前轮的差异计算角速度
      domega = (dist_rb - dist_lb) / wheel_bias_;  // 根据左右前轮的距离差来计算角速度
    }
    else {
      dist_lf = wheel_radius_ * tick_to_rad_ * lf_diff_ticks;
      dist_rf = wheel_radius_ * tick_to_rad_ * rf_diff_ticks;
      ds = (dist_lf + dist_rf) * 0.5;
      // 后轮没有运动，直接使用前轮的差异计算角速度
      domega = (dist_rf - dist_lf) / wheel_bias_;  // 根据左右前轮的距离差来计算角速度
    }

    
  } else { // default 4WD
    dist_lf = wheel_radius_ * tick_to_rad_ * lf_diff_ticks;
    dist_lb = wheel_radius_ * tick_to_rad_ * lb_diff_ticks;
    dist_rf = wheel_radius_ * tick_to_rad_ * rf_diff_ticks;
    dist_rb = wheel_radius_ * tick_to_rad_ * rb_diff_ticks;
    ds = ((dist_lf + dist_lb) + (dist_rf + dist_rb)) * 0.25;
    domega = ((dist_rf + dist_rb) - (dist_lf + dist_lb)) * 0.5 / wheel_bias_;
  }


  if (fabs(ds) > 1.6) {
    ROS_WARN("Invalid ds value[%lf].", ds);
    return ;
  }

  pose_x_ += ds * cos(pose_angle_);
  pose_y_ += ds * sin(pose_angle_);

  // if (abs(domega))
  pose_angle_ += domega;

  double vel_x = 0.0;
  double vel_thelta = 0.0;
  if (use_diff_twist_) {
    vel_x = ds / dt;
    vel_thelta = 1.0*domega / dt;
  } else {
    double vel_lf, vel_lb, vel_rf, vel_rb;
    vel_lf = cur_motor_state_.motor_states[LF_MOTOR_ID].rpm * rpm_to_vel_;
    vel_lb = cur_motor_state_.motor_states[LB_MOTOR_ID].rpm * rpm_to_vel_;
    vel_rf = cur_motor_state_.motor_states[RF_MOTOR_ID].rpm * rpm_to_vel_;
    vel_rb = cur_motor_state_.motor_states[RB_MOTOR_ID].rpm * rpm_to_vel_;
    
    if (motion_model_ == "2wd") {
      vel_x = (vel_lf + vel_rf) * 0.5;
      vel_thelta = (vel_lf - vel_rf) / wheel_bias_;
    } else {
      vel_x = (vel_lf + vel_lb + vel_rf + vel_rb) * 0.25;
      vel_thelta = ((vel_lf + vel_lb) - (vel_rf + vel_rb)) * 0.5 / wheel_bias_;
    }
  }

  publishMileage(ds);
  publishWheelInfo(dist_lf / dt, dist_lb / dt, dist_rf / dt, dist_rb / dt);
  publishOdometry(pose_x_, pose_y_, pose_angle_, vel_x, vel_thelta);
}

bool RobusterDriver::sendCommand(const vector<uint8_t> command, bool verb) {
  if (verb) {
    showData((const char*)&command[0], command.size());
  }

  if (serial_ptr_ && serial_ptr_->isOpen()) {
    boost::lock_guard<boost::mutex> lock(write_mutex_);
    try {
      size_t size = serial_ptr_->write(command);
      if (size < command.size()) {
        ROS_ERROR("Send cmd failed.Cmd size: %ldBytes, but only send %ldBytes", command.size(), size);
        return false;
      } 
      // ROS_INFO("Write cmd success. %ld - %ld", size, command.size());
      // serial_ptr_->flush();
    } catch (serial::IOException& e) {
      ROS_ERROR("Serial write data failed. %s", e.what());
      return false;
    } 
    return true;
  } else {
    return false;
  }
}

// must, or serial segment fault
void RobusterDriver::run() {
  ros::spin();
}

// no use. Compatible with older versions
bool RobusterDriver::getDevInfoCallback(robuster_mr_msgs::DevPermission::Request &req, 
     robuster_mr_msgs::DevPermission::Response &res) {
  return true;
}

bool RobusterDriver::lightControlCallback(robuster_mr_msgs::LightControl::Request &req, 
    robuster_mr_msgs::LightControl::Response &res) {
  ROS_INFO("Set light status: front(%d), rear(%d)", req.front, req.rear);
  res.status = 1;

  msg_que_.push(Command::setLightStatus(req.front, req.rear));
  if (1) {
    usleep(20 * 1000);
    new_light_control_ = true;

    if (waitForResponse(1000) < 0) {
      res.status = 1;
      res.status_msg = "Set light value timeouted.";
    } else {
      res.status = 0;
      res.status_msg = "success.";
    }
  }

  res.front = cur_light_state_.front_light.value;
  res.rear = cur_light_state_.rear_light.value;

  return true;
}

bool RobusterDriver::setErrorMask(robuster_mr_msgs::ErrorMask::Request &req, 
    robuster_mr_msgs::ErrorMask::Response &res)
{
  res.status = 1;
  uint8_t mask[8] = {0xff}; // 默认全使能
  mask[0] = req.motor;
  mask[1] = req.abnormal;
  mask[2] = req.bumper;
  mask[3] = req.communicate;
  ROS_INFO("ErrorMask: motor(%d), abnormal(%d), bumper(%d), comm(%d)", mask[0], mask[1], mask[2], mask[3]);

  msg_que_.push(Command::disableError(mask, 8));
  if (1) {
    usleep(2500);
    new_error_mask_ = true;

    if (waitForResponse(1000)) {
      res.status = 1;
      res.status_msg = "Set error mask timeouted.";
    } else {
      res.status = 0;
      res.status_msg = "success.";
    }
  }
  return true;
}

void RobusterDriver::handleOdomData(const char *data, const int len) {
  // showData(data, len);
  int32_t front_left_odom, front_right_odom, rear_left_odom, rear_right_odom;
  memcpy(&front_left_odom, (const void *)&data[6], sizeof(int32_t));  
  memcpy(&front_right_odom, (const void *)&data[10], sizeof(int32_t));
  memcpy(&rear_left_odom, (const void *)&data[14], sizeof(int32_t));
  memcpy(&rear_right_odom, (const void *)&data[18], sizeof(int32_t));         //单位0.1mm
  // publishOdometry(x/1000.0, y/1000.0, th/1000.0, 
  //               cur_dev_state_.linear_velocity, cur_dev_state_.angular_velocity);
}

void RobusterDriver::handleOdomOrientationData(const char *data, const int len) {

}

void RobusterDriver::handleVelData(const char *data, const int len) {

}

void RobusterDriver::handleMotorStatus(const char *data, const int len) {
  // Deprecated
  int32_t rpm = (int)(*((int *)&data[4]));
  int32_t position = (int)(*((int *)&data[8]));
}

void RobusterDriver::handleMotorVAStatus(const char *data, const int len) {
  // Deprecated
}

void RobusterDriver::handleMotorTickStatus(const char *data, const int len) {
  // showData(data, len);
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&cur_motor_state_.motor_states[i].tick, (const void *)&data[4 * (i + 1)+2], sizeof(int32_t));

    // 不在此处过滤异常值，方便查询原始脉冲数据状态
  }

  updateOdometry();
}

void RobusterDriver::handleMotorRPMStatus(const char *data, const int len) {
  //showData(data, len);
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&cur_motor_state_.motor_states[i].rpm, (const void *)&data[4 * (i + 1) + 2], sizeof(int32_t));
  }
}

void RobusterDriver::handleMotorVoltageStatus(const char *data, const int len) {
  uint32_t voltage = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&voltage, (const void *)&data[6 + 2 * i], sizeof(uint32_t));
    cur_motor_state_.motor_states[i].voltage = voltage / 100.0;
  }
}

void RobusterDriver::handleMotorCurrentStatus(const char *data, const int len) {
  int32_t current = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    memcpy(&current, (const void *)&data[4 * i + 2], sizeof(int32_t));
    cur_motor_state_.motor_states[i].voltage = current / 100.0;
  }
}

void RobusterDriver::handleMotorTemperatureStatus(const char *data, const int len) {
  /// TODO:
}

void RobusterDriver::handleMotorErrorStatus(const char *data, const int len) {
  int fault_value = 0;
  for (int i = 0; i < MOTOR_COUNTS; i++) {
    
    memcpy(&fault_value, (const void *)&data[4 * i + 2], sizeof(int16_t));
    // if motor fault happened, clean the state of this motor
    if (fault_value > 0) {
      memset(&cur_motor_state_, 0, sizeof(cur_motor_state_));
    } 
    
    cur_motor_state_.motor_states[i].fault = fault_value;
  }
}

void RobusterDriver::handleDevStatusData(const char *data, const int len) {
  boost::lock_guard<boost::mutex> lock(dev_state_mutex_);

  int8_t control_mode;
  memcpy(&control_mode, (const void *)&data[6], sizeof(control_mode));
  cur_dev_state_.control_mode = control_mode;
}

void RobusterDriver::handleErrStatusData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(dev_state_mutex_);
  if (len < 14) {
    ROS_ERROR("Unvailed serial data.");
    return;
  }

  if (((data[6] & 0x0f) == 0x0f) || ((data[6] & 0x0f) == 0x03)) {                 // emergency stop 2wd or 4wd
    cur_dev_state_.e_stop = true;
  }
  else {
    cur_dev_state_.e_stop = false;
  }

  if ((data[8] & 0x01) || (data[8] & 0x04)) {                 // front bumper
    cur_dev_state_.front_bumper = true;
  }
  else {
    cur_dev_state_.front_bumper = false;
  }

  if ((data[8] & 0x02) || (data[8] & 0x08)) {                 // rear bumper
    cur_dev_state_.rear_bumper = true;
  }
  else {
    cur_dev_state_.rear_bumper = false;
  }
}

void RobusterDriver::handleErrorMaskData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(light_mutex_);

  if (len < 14) {
    ROS_ERROR("Unvailed error mask msg.");
    return;
  }

  if (new_error_mask_) {
    ROS_INFO("Emit cond signal for setErrorMask");
    emitCondSignal();
    new_light_control_ = false;
  }
}

void RobusterDriver::handleLightStatusData(const char *data, const int len) {
  // showData(data, len);
  boost::lock_guard<boost::mutex> lock(light_mutex_);

  if (len < 14) {
    ROS_ERROR("Unvailed light state msg.");
    return;
  }

  int front_light_value, rear_light_value;
  memcpy(&front_light_value, (const void *)&data[6], sizeof(front_light_value));
  memcpy(&rear_light_value, (const void *)&data[10], sizeof(rear_light_value));

  cur_light_state_.front_light.value = static_cast<char>(front_light_value);
  cur_light_state_.rear_light.value = static_cast<char>(rear_light_value);

  cur_dev_state_.front_light = static_cast<int16_t>(front_light_value);
  cur_dev_state_.rear_light = static_cast<int16_t>(rear_light_value);

  if (new_light_control_) {
    ROS_INFO("Emit cond signal for light control.(%d, %d)", front_light_value, rear_light_value);
    emitCondSignal();
    new_light_control_ = false;
  }
}

void RobusterDriver::handleBatteryStatusData(const char *data, const int len) {
  // showData(data, len);
  unsigned short battery_percentage = 0;
  short battery_temeprature = 0;
  short voltage = 0;
  short current = 0;    // 负数表示放电
  memcpy(&battery_percentage, (const void *)&data[6], sizeof(battery_percentage));
  memcpy(&battery_temeprature, (const void *)&data[8], sizeof(battery_temeprature));
  memcpy(&voltage, (const void *)&data[10], sizeof(voltage));
  memcpy(&current, (const void *)&data[12], sizeof(current));

  if (battery_percentage <= 100) {
    battery_mutex_.lock();

    cur_battery_state_.battery_percentage = battery_percentage;
    cur_battery_state_.battery_temperature = (float)battery_temeprature / 10.0;

    cur_dev_state_.battery_percentage = battery_percentage;
    battery_mutex_.unlock();
  } else {
    ROS_WARN("Invalid power percentage value(%d)", battery_percentage);
  }

  battery_mutex_.lock();
  if (voltage <= VOLTAGE_MAX) {
    cur_battery_state_.voltage = (float)voltage / 100.0;
  } else {
    ROS_WARN("Invalid voltage value(%d)", voltage);
  }

  if (current <= CURRENT_MAX) {
    cur_battery_state_.current = (float)current / 100.0;
    cur_dev_state_.battery_current = (float)current / 100.0;
  } else {
    ROS_WARN("Invalid current value(%d)", current);
  }
  battery_mutex_.unlock();

}

void RobusterDriver::handleBatteryCapStatusData(const char *data, const int len) {
  /// unuse 
}

void RobusterDriver::handleBatteryVAStatusData(const char *data, const int len) {
  // showData(data, len);
  unsigned int voltage = 0;
  int current = 0;  // 负数表示放电
  memcpy(&voltage, (const void *)&data[6], sizeof(voltage));
  memcpy(&current, (const void *)&data[10], sizeof(current));
  battery_mutex_.lock();
  if (voltage <= VOLTAGE_MAX) {
    cur_battery_state_.voltage = voltage / 100.0;
  } else {
    ROS_WARN("Invalid voltage value(%d)", voltage);
  }

  if (current <= CURRENT_MAX) {
    cur_battery_state_.current = current / 100.0;
    cur_dev_state_.battery_current = current / 100.0;
  } else {
    ROS_WARN("Invalid current value(%d)", current);
  }
  
  battery_mutex_.unlock();
}

void RobusterDriver::handleCmdStatusData(const char *data, const int len) {
  //showData(data, len);
  boost::lock_guard<boost::mutex> lock(cmd_state_mutex_);

  uint8_t gear_position = 0xFF; 
  uint16_t throttle = 0; 
  uint16_t brake_strength = 0; 
  int constant_speed = 0;
  int steering_speed = 0;
  int angle= 0;
  memcpy(&gear_position, (const void *)&data[0 + 6], sizeof(uint8_t));
  memcpy(&throttle, (const void *)&data[1 + 6], sizeof(uint16_t));
  memcpy(&brake_strength, (const void *)&data[3 + 6], sizeof(uint16_t));
  memcpy(&constant_speed, (const void *)&data[5 + 6], sizeof(int));
  memcpy(&steering_speed, (const void *)&data[9 + 6], sizeof(int));
  memcpy(&angle, (const void *)&data[13 + 6], sizeof(int));
  
  printf("gear_position: 0x%02x, throttle: %d, brake_strength: %d, constant_speed: %d, steering_speed: %d, angle: %f\n",gear_position,throttle,brake_strength,constant_speed, steering_speed,(float)(angle)/1000);
}

void RobusterDriver::publishData(const ros::TimerEvent&) {
  static ros::Time last_time;

  /// TODO: publish while value changed
  if (battery_state_pub_.getNumSubscribers() != 0) {
    battery_state_pub_.publish(cur_battery_state_);
  }

  if (light_state_pub_.getNumSubscribers() != 0) {
    light_state_pub_.publish(cur_light_state_);
  }

  if (dev_state_pub_.getNumSubscribers() != 0) {
    dev_state_pub_.publish(cur_dev_state_);
  }

  if (motor_state_pub_.getNumSubscribers() != 0){
    motor_state_pub_.publish(cur_motor_state_);
  }

  if (ros::Time::now().toSec() - last_time.toSec() > 10.0) {
    updateMileage(); // 定时更新行驶里程
    last_time = ros::Time::now();
  }
}

void RobusterDriver::initSerialRingBuffer(SerialRingBuffers &ring_buffer) {
  boost::lock_guard<boost::mutex> lock(ring_buffer_mutex_);
  ring_buffer.head = 0;
  ring_buffer.tail = 0;
  ring_buffer.uart_length = 0;
  memset(ring_buffer.recv_buf, 0, sizeof(ring_buffer.recv_buf));
}


/* 读队列 */
bool RobusterDriver::readSerialRingBuffer(SerialRingBuffers &ring_buffer, uint8_t *data) {
  boost::lock_guard<boost::mutex> lock(ring_buffer_mutex_);
  if(ring_buffer.uart_length == 0) {
    return false;
  }
  *data = ring_buffer.recv_buf[ring_buffer.head];
  ring_buffer.recv_buf[ring_buffer.head] = 0;
  ring_buffer.head++;
  ring_buffer.head %= UART_DATA_MAX;  /* 防止数组溢出 */
  ring_buffer.uart_length--;  
  return true;
}
 
/* 往队列里面写 */
bool RobusterDriver::writeSerialRingBuffer(SerialRingBuffers &ring_buffer, uint8_t data) {
  boost::lock_guard<boost::mutex> lock(ring_buffer_mutex_);
  if(ring_buffer.uart_length >= UART_DATA_MAX) {
    //std::cout<<"<writeSerialRingBuffer> ring_buffer.uart_length: "<<ring_buffer.uart_length<<std::endl;
    return false;
  }
  ring_buffer.recv_buf[ring_buffer.tail] = data;
  ring_buffer.tail ++;
  ring_buffer.tail %= UART_DATA_MAX; /* 防止数组溢出 */
  ring_buffer.uart_length++;
  return true;
}

/* 获取当前队列有效数据（未处理数据）长度 */
uint32_t RobusterDriver::getSerialRingBuffer(SerialRingBuffers ring_buffer) {
  uint32_t length;
  boost::lock_guard<boost::mutex> lock(ring_buffer_mutex_);
  length = ring_buffer.uart_length;
  return length;
}
