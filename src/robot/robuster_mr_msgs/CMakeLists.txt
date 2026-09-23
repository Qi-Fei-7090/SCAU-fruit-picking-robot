cmake_minimum_required(VERSION 3.0.2)
project(robuster_mr_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  pcl_msgs
  sensor_msgs
  std_msgs
  tf
)


add_message_files(
  FILES
  DeviceState.msg
  BatteryState.msg
  LightState.msg
  SingleLightState.msg
  MotorState.msg
  SingleMotorState.msg
  imuData.msg 
  sonarData.msg  
  encoderData.msg
  MileageState.msg
  WheelVel.msg
  AckermannDrive.msg
  AckermannDriveStamped.msg
)


 add_service_files(
   FILES
   controlCmdData.srv
   DevPermission.srv
   setPermission.srv
   resetAlarm.srv
   LightControl.srv
   enableUltrasound.srv
   resetImu.srv
   getDevInfo.srv
   NavigationControl.srv
   MappingControl.srv
   MappingSave.srv
   MappingSave3D.srv
   MappingSelect.srv
   MappingList.srv
   RobusterBaseControl.srv
   DockingControl.srv
   ErrorMask.srv
   #lightControl.srv
   TaskStart.srv
   TaskStop.srv
   TaskRevert.srv
   TaskPause.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   move_base_msgs   nav_msgs   pcl_msgs   sensor_msgs   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robuster_mr_msgs
#  CATKIN_DEPENDS geometry_msgs message_generation move_base_msgs nav_msgs pcl_msgs sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

