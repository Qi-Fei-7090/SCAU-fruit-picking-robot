# robuster_driver
robuster 系列产品底盘 ROS 驱动

## 目录说明

* script：功能脚本
  * robuster_teleop_key.py：键盘控制脚本
* launch：ros launch 文件
  * driver.launch：底盘驱动.启动底盘驱动节点
* src：源码
* include：头文件

## 安装依赖  

```
pip install rospkg # 或 sudo apt install python-rospkg
sudo apt install ros-melodic-yocs-velocity-smoother 
```

## 编译方法

```
cd ~
mkdir robuster_ws/src -p
cd robuster_ws/src 
# 将源码拷贝到该目录下

cd ../
catkin_make

source devel/setup.bash
rospack profile

```

## 启动底盘驱动

```
roslaunch robuster_driver driver.launch
```

`robuster_base.launch` 参数说明：
* **port**：串口端口号，默认为 `/dev/ttyS0` 
* **baud_rate**：串口波特率，默认为 `115200`
* **control_rate**：串口发送控制命令的速率,建议维持不变
* **motion_model**：运动模型，默认为 `4wd`, 即: 4驱. 当前支持 `4wd/2wd`. MR2000 为 `4wd`, MR500 为 `2wd`, MR1000 旧版为 `2wd`, 新版为 `4wd`  
* **base_type**：底盘驱动方式，默认为 `skid_steer`. 支持类型包括：
  * skid_steer：skid steer 
  * mecanum：麦克纳姆轮驱动 
* **odom_frame**：里程坐标系，默认为 `odom`
* **base_frame**：base 坐标系，默认为 `base_link`
* **publish_odom**：是否发布 `odom` 数据. 默认为 `false`, 即:不发布.
* **publish_odom_trans**：是否发布 `odom->base_link` transform. 默认为 `false`，即:不发布.
* **wheel_radius**：车轮半径, 注：此处使用修正值，可能与实际轮子半径有差别
* **wheel_bias**：轴间距
* **rpm_factor**：RPM 因子. 因电机方案选择引入的参数,用户无需修改.

MR 系列产品车轮半径及轴间距参数：
|  | Wheel Radius | Bias |
| --- | ------ | ------ |
| MR500  | 0.102 | 0.375 |   
| MR1000 | 0.154 | 0.572 |
| MR2000 | 0.125 | 0.673 |


## 键盘控制  

要使用键盘控制功能，首先确保已启动底盘驱动，并确保 `script/robuster_teleop_key.py` 文件有可执行权限

```
cd robuster_driver/script/
chmod +x robuster_teleop_key.py

```

运行键盘控制节点：  
```
rosrun robuster_driver robuster_teleop_key.py
```

按键说明：
```
Control Your robuster!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (-0.22 ~ 0.22)
a/d : increase/decrease angular velocity (-2.84 ~ 2.84)

space key, s : force stop

```

* w/x：增加/减小线速度
* a/d：增加/减小角速度
* 空格/s：强制停止

## Topics 及 Services

### Topics

#### 1. 发布主题
* /battery_state：电池状态. `robuster_mr_msgs/BatteryState` 类型. field 信息：  
```
battery_percentage: 电量百分比
voltage：电压值
current：电流值
battery_temperature：电池温度
remaining_capacity：剩余容量
rated_capacity：额定容量
```

* /device_state：设备状态. `robuster_mr_msgs/DeviceState` 类型. field 信息：  
```
base_state：保留,无实际意义
control_mode：控制模式. 0 - 空闲; 1 - 上位机; 2 - 遥控器
temperature：车体内部温度值. 当前未使用
linear_velocity：线速度值
angular_velocity：角速度值
front_bumper：前防撞杆状态. true - 触发; false - 未触发
rear_bumper：后防撞杆状态
front_light：前车灯亮度值. [0-100], 0 - 关闭 
rear_light：前车灯亮度值. [0-100]
battery_percentage：电量百分比
battery_current：电流值
e_stop：急停状态
fault：异常值
uptime：保留,暂未使用

```

* /light_state：预留, 灯光状态. `robuster_mr_msgs/LightState` 类型. 预留灯光模式控制功能, 当前模式无效, 请使用 `/device_state` 主题获取灯光信息. field 信息：  
```
front_light # 前灯状态
  mode：灯光模式
  value：亮度值
rear_light # 后灯状态
  mode
  value
left_light # 左灯状态
  mode
  value
right_light # 右灯状态
  mode
  value
```

* /mileage_state：总里程信息. `robuster_mr_msgs/MileageState` 类型. field 信息：  
```
total_mileage：历史总里程
relative_mileage：此次启动总里程. 从启动驱动节点开始统计
```

* /motor_state：点击状态. `robuster_mr_msgs/MotorState` 类型. field 信息：  
```
voltage：电机电压
current：电机电流
rpm：电机转速
tick：脉冲统计
temperature：电机温度
fault：电机异常状态
```

* /odom：odometry 数据. `nav_msgs/Odometry` 类型

#### 2. 订阅主题

* /cmd_vel：控制指令. 例如：控制小车以 0.2m/s 速度向前移动
```
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Services

* /robuster/light_control：设置灯光亮度. 例如：设置前后灯亮度均为 50%   
```
$ rosservice call /robuster/light_control "front: 50
rear: 50"
```