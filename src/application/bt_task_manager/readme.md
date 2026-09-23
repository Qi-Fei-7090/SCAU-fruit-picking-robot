# Behavior Tree Task Manager
基于行为树的任务管理框架和原地抓取和移动抓取的demo实现

# 依赖
```
sudo apt install ros-melodic-py-trees*
sudo apt install ros-melodic-rqt-py-trees ros-melodic-moveit-commander
```

# 功能脚本说明

## grasp_node

底盘不移动，根据控制指令进行原地的抓取、放置和其他任务

### 调用接口

- sub : /grasp_node/cmd 
- type : String
- values : 
    - pick : 执行单次抓取任务
    - place : 执行单次放置任务
    - loop : 循环执行pick and place任务
    - reset : 执行单次复位任务
    - gripper_open : 执行单次打开夹爪任务
    - gripper_close : 执行单次关闭夹爪任务
    - stop : 停止任务

### launch启动文件和参数说明

 - move_group_name: 机械臂moveit配置的move_group name
 - gripper_name: gripper的name，参考gripper.py里面的类型定义
 - target_frame: 目标的tf frame
 - tcp_offset_z: 机械臂move group规划的eef和夹爪之间的偏移长度
 - arm_feed_depth: 机械臂先规划到目标的z轴前方距离，再按照这个值进行进刀、close gripper、退刀
 - grasp_angles: 默认的末端抓取姿态，欧拉角XYZ,注意实际使用时，需要确认好旋转轴和旋转顺序和动定轴

## controller_node

根据任务列表配置文件，执行移动抓取/放置任务

### 调用接口

- 开始任务
    - sub : /controller_node/dashboard/start
    - type : Empty
    - value:None
- 取消任务
    - sub : /controller_node/dashboard/cancel
    - type : Empty
    - value: 

### launch启动文件和参数说明

 - task_cfg: 任务列表路径
 - move_group_name: 机械臂moveit配置的move_group name
 - gripper_name: gripper的name，参考gripper.py里面的类型定义
 - target_frame: 目标的tf frame
 - tcp_offset_z: 机械臂move group规划的eef和夹爪之间的偏移长度
 - arm_feed_depth: 机械臂先规划到目标的z轴前方距离，再按照这个值进行进刀、close gripper、退刀
 - grasp_angles: 默认的末端抓取姿态，欧拉角XYZ,注意实际使用时，需要确认好旋转轴和旋转顺序和动定轴
 - arm_feed_depth: 底盘移动到抓取或放置工位前的距离

### 任务列表yaml配置格式说明

```
name: ""
repeat: true
tasks:
  -
    position: [-7,14 ,90]
    action: pick
  -
    position: [-7,10 ,-90]
    action: place
```

- name: 任务名称
- repeat: 是否重复执行，true:重复执行，直到取消
- tasks: 任务对象数组

#### 任务列表数组说明

列表中的每一个元素都含有position和action

- position: 工作地点坐标[x,y,yaw]
- action: 具体工作名；pick:抓取，place:放置，empty:只添加导航

# 使用

- grasp_node : 原地抓取节点

- controller_node : 移动抓取节点

使用步骤：

1. 启动导航、机械臂、视觉检测软件包
2. 启动grasp_node或者controller_node
3. 发送开始命令或者其他指定命令

# TODO

## controller_node
- 导航和底盘移动前增加checkpose，不在goal才进行移动
- 后续可以加载所有任务，根据调用的任务名，执行不同任务
- 取消后，取消导航和其他任务
## grasp_node
