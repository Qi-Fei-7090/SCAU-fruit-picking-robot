#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <vector>
#include <map>
#include <geometry_msgs/Pose.h>

using  namespace std;

class MetaTask
{
public:
  explicit MetaTask(/* args */);
  ~MetaTask();

  geometry_msgs::Pose getTaskPose() { return goal_; }
  void setTaskPose(geometry_msgs::Pose &pose) { goal_ = pose; }

  string getTaskAction() { return action_type_; }
  void setTaskAction(string action) { action_type_ = action; }

  string getTaskType() { return task_type_; }
  void setTaskType(string type) { task_type_ = type; }

  void showTask() {
    std::cout << "      curent task type: " << task_type_ << std::endl;
    if (task_type_ == "nav") {
      std::cout << "      nav pose: (" << goal_.position.x << ", " << goal_.position.y << ", "
                << goal_.position.z << "), (" << goal_.orientation.x << ", " << goal_.orientation.y << ", " 
                << goal_.orientation.z << ", " << goal_.orientation.w << ")" << std::endl;
    } else if (task_type_ == "action") {
      std::cout << "      Action type: " << action_type_.c_str() << std::endl;
    }
  }

private:
  int task_num_;
  string task_type_; // nav/action

  geometry_msgs::Pose goal_;
  string action_type_;
};

class Task
{
public:
  explicit Task();
  ~Task();

  bool getRepeatFlag() {return repeat_;}
  void setRepeatFlag(bool repeat) {repeat_ = repeat;}

  string getTaskName() {return task_name_;}
  void setTaskName(string name) {task_name_ = name;}

  void showTask() {
    std::cout << "Task list info: \n  task name: " << task_name_ << ", task count(" << meta_tasks_.size() << ")" << std::endl;
    for (int i = 0; i < meta_tasks_.size(); i++) {
      std::cout << "    " << i << "th task have " <<  meta_tasks_[i].size() << " sub_task" << std::endl;
      for (int j = 0; j < meta_tasks_[i].size(); j++) {
        meta_tasks_[i][j].showTask();
      }
    }
  }

public:
  vector<vector<MetaTask>> meta_tasks_;

private:
  string task_name_;
  bool repeat_;
};


#endif