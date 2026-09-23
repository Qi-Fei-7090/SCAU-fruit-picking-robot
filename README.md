# 华南农业大学工程训练中心自主采摘机器人
# 采摘机器人（MMC1 移动机械臂）

华南农业大学工程训练中心采摘机器人项目 —— 基于 **ROS1 (Melodic) + MoveIt** 的移动机械臂采摘系统。

系统由 MMC1 轮式底盘、JAKA MiniCobo 六轴机械臂、RealSense D435i 深度相机与 YOLOv5/TensorRT 视觉检测组成，
通过行为树（py_trees）编排「导航 → 视觉定位 → 抓取 → 放置」的完整采摘作业流程。

> **平台说明**：本仓库是一个 ROS1环境下工程，构建与运行均在 **Ubuntu 18.04 + ROS Melodic** 上完成。



## 系统架构

```
                    ┌──────────────────────────────────────────┐
                    │   bt_task_manager（行为树任务编排）        │
                    │   grasp_node / controller_node           │
                    └───────┬───────────────┬──────────────────┘
                            │               │
              ┌─────────────▼──────┐   ┌────▼─────────────────────┐
              │  navigation_2d     │   │  MoveIt (jaka_planner +  │
              │  move_base + AMCL  │   │  mmc1_moveit_config)     │
              │  /  slam_toolbox   │   │  → JAKA 机械臂 + 夹爪     │
              └─────────┬──────────┘   └────▲─────────────────────┘
                        │                   │ 抓取位姿 (TF/黑board)
                        │          ┌────────┴─────────────────────┐
                        │          │  yolo5_detector              │
                        │          │  YOLOv5 + TensorRT → 3D 坐标 │
                        │          └────────▲─────────────────────┘
                        │                   │ /camera/color/image_raw
    ┌───────────────────▼───────────────────┴──────────────────────┐
    │  robot_bringup（整机启动汇聚层）                                │
    │  robot_driver.launch / arm_bringup.launch / camera.launch     │
    └───────────────────────────┬──────────────────────────────────┘
                                │
    ┌───────────────────────────▼──────────────────────────────────┐
    │  drivers: robuster_driver(底盘) · wit_node(IMU)               │
    │           lakibeam1(雷达) · realsense-ros(相机)               │
    └──────────────────────────────────────────────────────────────┘
```

**数据流（一次完整采摘）**

1. `controller_node` 读取任务 YAML，通过 `move_base` 导航到目标工位 `[x, y, yaw]`。
2. `yolo5_detector` 订阅相机彩色/深度图，用 TensorRT 推理后在 `yolo5/<camera>/detected_objects`（`PointCloud`）发布目标 3D 坐标与类别。
3. 行为树的 `GetGraspPoseByTopic` 叶节点订阅该话题，做 **连续 10 帧稳定判据** 去抖，再通过 TF 把相机系坐标变换到 `base_link`，结合 `tcp_offset_z` / `feed_depth` 生成抓取位姿，写入黑板并广播 `grasp_pose` TF。
4. `moveit_arm` 调用 MoveIt（规划组 `jaka_minicobo`）执行进刀、闭合夹爪、退刀。
5. 重复上述过程完成 `place` 动作，或按 `repeat: true` 循环作业。

---

## 目录结构

仓库按功能分层，**每个包都是独立的 git 仓库**（shallow clone），改动前请先确认所在包的仓库与分支。

| 目录 | 职责 | 关键包 |
|---|---|---|
| `application/` | 集成层：整机启动、任务编排、导航与 MoveIt 配置 | `robot_bringup`、`bt_task_manager`、`navigation_2d`、`mmc1_moveit_config` |
| `drivers/` | 硬件驱动 | `robuster_driver`（底盘）、`wit_node`（IMU）、`lakibeam1`（雷达）、`realsense-ros`（相机） |
| `manipulator/` | JAKA 机械臂全家族 | `jaka_robot/*`（`jaka_driver`、`jaka_planner`、`jaka_msgs`、各型号 `moveit_config`） |
| `navigation/` | ROS1 navigation 栈（第三方源码） | `navigation`、`navigation_msgs`、`teb_local_planner` |
| `robot/` | 机器人模型、消息、GUI、视觉 | `mmc1_description`、`robuster_mr_msgs`、`robuster_demo_gui`、`yolo5_detector` |
| `slam/` | 建图与雷达处理 | `slam_toolbox/*`、`lidar_undistortion_2d` |
| `vision/` | Gazebo 仿真视觉 | `realsense_gazebo_description`、`realsense_gazebo_plugin` |



---

## 硬件配置

| 部件 | 型号 / 参数 | 关键配置位置 |
|---|---|---|
| 移动底盘 | Robuster MMC1，2wd / `skid_steer`，轮径 0.098 m，减速比 20 | `drivers/robuster_driver/launch/driver.launch` |
| 机械臂 | JAKA MiniCobo，IP `192.168.11.60` | `application/robot_bringup/launch/arm_bringup.launch` |
| 深度相机 | RealSense D435i，序列号 `238722072768`（eye-on-hand） | `application/robot_bringup/launch/camera.launch` |
| 激光雷达 | LakiBeam1，sensor `192.168.11.20`，host `192.168.11.11`，端口 2368 | `drivers/lakibeam1/launch/lakibeam1_scan_front.launch` |
| IMU | wit_node，`/dev/ttyUSB1` | `drivers/wit_node/launch/wit.launch` |
| 底盘串口 | `/dev/ttyUSB0`，115200 bps，控制频率 25 Hz | `drivers/robuster_driver/launch/driver.launch` |

> **串口与 IP 为硬编码默认值**，更换硬件后请全局 `grep` 确认所有引用点后再修改。

---

## 环境搭建

**目标环境：Ubuntu 18.04 + ROS Melodic + Python 3.6 + CUDA / TensorRT**

### 1. 创建 catkin 工作区

```bash
mkdir -p ~/mmc1_ws/src
# 将本仓库所有包放入 src/
cd ~/mmc1_ws
```

### 2. 安装依赖

```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt install ros-melodic-moveit ros-melodic-moveit-commander
sudo apt install ros-melodic-py-trees* ros-melodic-rqt-py-trees
sudo apt install ros-melodic-teb-local-planner ros-melodic-slam-toolbox
sudo apt install ros-melodic-realsense2-camera ros-melodic-joy

# Python 依赖
pip3 install numpy opencv-python pycuda
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

> ⚠️ 编译 `yolo5_detector` 时必须显式指定 Python 路径；同时注意 `empy` 版本必须为 `3.3.4`，
> 且需卸载与其同名的 `em` 包。

```bash
# 确认 libpython3.6m 实际路径
whereis libpython3.6m.so.1.0

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6 \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so.1.0

source devel/setup.bash
```

### 4. 串口权限

```bash
sudo usermod -aG dialout $USER
# 重新登录生效，或临时赋权：
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

---

## 模型转换（YOLOv5 → TensorRT）

视觉模块使用 TensorRT 加速推理，需要把 YOLOv5 权重（`.wts`）转换为 `.engine`。

### 内置模型

| 模型 | 识别类别 | 标签 |
|---|---|---|
| `asamu_degrees90.wts` | 农夫山泉、橙汁、阿萨姆奶茶 | 0 / 1 / 2 |
| `best_appasamu.wts` | 上者 + 橘子、苹果 | 0 / 1 / 2 / 3 / 4 |

> 出厂内置模型为 `best_appasamu.wts`。**本项目默认使用苹果模型 `apple.engine`（单类别）**，
> 见 `robot/yolo5_detector/scripts/yolo5rt_detector.py`。

### 转换步骤

```bash
# 1. 按类别数修改 CLASS_NUM（asamu 模型 = 3，best_appasamu 模型 = 5）
gedit ./notROS-tools/tensorrtx-yolov5-v6.0/yolov5/yololayer.h

# 2. 编译转换工具
cd ./notROS-tools/tensorrtx-yolov5-v6.0/yolov5/ && mkdir -p build && cd build
cmake .. && make -j$(($(nproc) - 1))

# 3. 生成为 engine（必须 sudo）
sudo ./yolov5 -s best_appasamu.wts best_appasamu_s.engine s
```

### 部署与配置

将生成的 `*_s.engine` 与 `libmyplugins.so` 放入 `yolo5_detector/scripts/tensorRT/`，
然后按实际模型修改 `robot/yolo5_detector/scripts/yolo5rt_detector.py`：

```python
pluginlibname = "libmyplugins.so"
wfname = "apple.engine"        # TensorRT 模型文件
categories = ["apple"]         # 识别类别，必须与网络标签顺序一致
```

### 设置抓取目标类别

每次进入检测叶节点前，通过参数服务器指定要抓取的目标标签：

```bash
rosparam set /pick_target_type 4    # 4 = apple（best_appasamu 模型）
```

也可在 launch 中固定默认值：

```xml
<param name="/pick_target_type" type="int" value="0"/>
```

---

## 运行流程

### 步骤 1：建图（首次部署或环境变化时）

```bash
roslaunch navigation_2d slam_online_async.launch    # 在线建图
rosrun map_server map_saver -f ~/map/map            # 保存地图
```

其他可选：`slam_offline.launch`（离线建图）、`slam_localization.launch`（纯定位）、`slam_lifelong.launch`（终生建图）。

### 步骤 2：启动整机驱动

```bash
# 底盘 + IMU + 雷达 + TF 树
roslaunch robot_bringup robot_driver.launch
```

### 步骤 3：启动导航

```bash
roslaunch navigation_2d navigation.launch
```

> 注意该 launch 内部会再次 include `robot_driver.launch`，若步骤 2 已单独启动会重复拉起节点。

### 步骤 4：启动机械臂与相机

```bash
# 机械臂（含 MoveIt）—— robot_ip / robot_model 可按需覆盖
roslaunch robot_bringup arm_bringup.launch robot_ip:=192.168.11.60 robot_model:=minicobo

# 或直接启动「机械臂 + 相机 + RViz」组合
roslaunch robot_bringup grasp_bringup.launch add_realsense_d435i_mode:=eye_on_hand
```

### 步骤 5：启动视觉检测

```bash
roslaunch yolo5_detector yolo5_detector.launch
```

### 步骤 6：启动任务节点

```bash
# 原地抓取（底盘不动）
roslaunch bt_task_manager grasp_node.launch

# 移动抓取（按任务列表导航 + 抓取）
roslaunch bt_task_manager controller_node.launch
```

### 步骤 7：下发指令

```bash
# grasp_node：单次抓取
rostopic pub /grasp_node/cmd std_msgs/String "data: 'pick'" -1
# 可选指令：pick / place / loop / reset / gripper_open / gripper_close / stop

# controller_node：开始 / 取消任务
rostopic pub /controller_node/dashboard/start std_msgs/Empty "{}" -1
rostopic pub /controller_node/dashboard/cancel std_msgs/Empty "{}" -1
```

---

## 任务配置

`controller_node` 从 YAML 读取任务列表，默认文件为 `application/bt_task_manager/data/task_grasp.yaml`：

```yaml
name: "采摘任务"
repeat: true          # true：循环执行，直到收到 cancel
tasks:
  - position: [-11.5, 12, 130]   # [x, y, yaw]，单位为 m / 度
    action: pick
  - position: [-8, 10, -30]
    action: place
```

- `position`：工作地点坐标 `[x, y, yaw]`
- `action`：`pick`（抓取）、`place`（放置）、`empty`（仅导航不作业）

关键抓取参数（在 `grasp_node.launch` / `controller_node.launch` 中配置）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `move_group_name` | `jaka_minicobo` | MoveIt 规划组名，须与 `mmc1.srdf` 一致 |
| `gripper_name` | `jaka_gripper` | 夹爪类型，见 `gripper/gripper.py` |
| `target_frame` | `yolo5_obj_camera_1` | 目标 TF frame |
| `camera_frame` | `camera_color_optical_frame` | 相机光学坐标系 |
| `tcp_offset_z` | `-0.15` | 规划 eef 与夹爪 TCP 之间的 z 向偏移 |
| `arm_feed_depth` | `0.12` | 进刀 / 退刀深度 |
| `grasp_angles_{x,y,z}` | `0 / 91 / 0` | 末端抓取姿态欧拉角（注意旋转轴与旋转顺序） |

---

## 关键话题与服务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/yolo5/<camera>/detected_objects` | `sensor_msgs/PointCloud` | 检测目标 3D 坐标；`channels` 依次为 类别 / id / 置信度 |
| `/yolo5/<camera>/resultimg` | `sensor_msgs/Image` | 渲染后的检测结果图 |
| `/yolo5/controlservice` | `yolo5_detector/ControlCmd` | 检测服务的启停控制 |
| `/grasp_node/cmd` | `std_msgs/String` | 原地抓取任务指令 |
| `/controller_node/dashboard/start` \| `/cancel` | `std_msgs/Empty` | 移动抓取任务的开始 / 取消 |
| `/pick_target_type` | `rosparam` (int) | 目标类别标签 |
| `/front_scan` | `sensor_msgs/LaserScan` | 前向雷达点云（经 `lidar_undistortion` 去畸变后供导航使用） |
| `/imu/data` | `sensor_msgs/Imu` | IMU 数据 |

---

## 常见问题与注意事项

### 构建与依赖

- **`empy` 版本冲突**：编译报错时确认 `empy==3.3.4`，并卸载同名的 `em` 包。
- **Python 路径**：`catkin_make` 必须带 `-DPYTHON_LIBRARY`，否则 `yolo5_detector` 会链接失败。
- **消息包改动**：修改 `robuster_mr_msgs` 或 `jaka_msgs` 的 `.msg` 后，**必须重编所有依赖方**，而不只是消息包本身。

### 模型与 TF 链路

- **URDF 改动影响面大**：`mmc1_description` 的 URDF/xacro 被 `robot_bringup`、`mmc1_moveit_config` 与 Gazebo 共同消费；
  改动关节名 / 连杆名会同时打断 TF 树、MoveIt 规划与仿真。
- **SRDF 与 URDF 必须一致**：`mmc1_moveit_config/config/mmc1.srdf` 中的 group 名（`jaka_minicobo`）与末端执行器命名需与 URDF 对应。
- **`arm_bringup.launch` 的包名拼接写法**（`$(eval find('mmc1' + '_moveit_config'))`）是刻意为之：
  切换 `robot_model` 时 MoveIt 配置包名需跟着匹配（`jaka_<model>_moveit_config` 与 `mmc1_moveit_config`），请勿按字面改成单一包名。
- **`.wts` 与 `categories` 必须同步**：更换模型后类别数与标签顺序都要改，否则抓取目标会错位。



