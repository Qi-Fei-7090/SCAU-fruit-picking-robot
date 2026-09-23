/**************************************************************************
 * Copyright (C), 2020-2025, Robot++ Co., Ltd.
 * Author: Damon
 * version: 0.1
 * Date: 2020-12-03
 * 
 * Description: Serial Driver
 **************************************************************************/ 

#include "serial_node.h"
#include <signal.h>

RobusterDriver *driver = nullptr;

void sigintHandler(int sig)
{
	ROS_INFO("shutting down!");
  if (driver) {
    delete driver;
    driver = nullptr;
  }
	ros::shutdown();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "robuster_base_node", ros::init_options::NoSigintHandler);

  signal(SIGINT, sigintHandler);

  try {
    driver = new RobusterDriver;
    driver->run();
  } catch (exception &e) {
    ROS_ERROR("Failed to construct RobusterDriver object.");
  }

  // driver->closeDriver();

  return 0;
}