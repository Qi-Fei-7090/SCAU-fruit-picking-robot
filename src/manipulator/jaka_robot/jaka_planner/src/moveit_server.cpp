#include "ros/ros.h"
#include "JAKAZuRobot.h"
#include "jkerr.h"
#include "jktypes.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <map>
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Empty.h"
#include <thread>

#include "jaka_msgs/SetIO.h"
#include "jaka_msgs/GetIO.h"

using namespace std;
JAKAZuRobot robot;
const  double PI = 3.1415926;
BOOL in_pos;
int ret_preempt;
int ret_inPos;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};

// Determine if the robot has reached the target position.
bool jointStates(JointValue joint_pose)
{
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    bool joint_state = true;
   
    for (int i = 0; i < 6; i++)
    {
        bool ret = joint_pose.jVal[i] * 180 / PI - 0.2 < robotstatus.joint_position[i] * 180 / PI
        && robotstatus.joint_position[i] * 180 / PI < joint_pose.jVal[i] * 180 / PI + 0.2;
        joint_state = joint_state && ret;
    }
    cout << "Whether the robot has reached the target position: " << joint_state << endl;       //1到达；0未到达
    return joint_state;
}

//Moveit server
void goalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as)
{
    BOOL in_pos;
    robot.servo_move_enable(true);
    int point_num=torso_goal->trajectory.points.size();
    ROS_INFO("number of points: %d",point_num);
    JointValue joint_pose;
    float lastDuration=0.0;
    OptionalCond* p = nullptr;
    for (int i=1; i<point_num; i++) {        
        joint_pose.jVal[0] = torso_goal->trajectory.points[i].positions[0];
        joint_pose.jVal[1] = torso_goal->trajectory.points[i].positions[1];
        joint_pose.jVal[2] = torso_goal->trajectory.points[i].positions[2];
        joint_pose.jVal[3] = torso_goal->trajectory.points[i].positions[3];
        joint_pose.jVal[4] = torso_goal->trajectory.points[i].positions[4];
        joint_pose.jVal[5] = torso_goal->trajectory.points[i].positions[5];      
        float Duration=torso_goal->trajectory.points[i].time_from_start.toSec();

        float dt=Duration-lastDuration;
        lastDuration=Duration;

        int step_num=int (dt/0.008);
        int sdk_res=robot.servo_j(&joint_pose, MoveMode::ABS, step_num);

        if (sdk_res !=0)
        {
            ROS_INFO("Servo_j Motion Failed");
        } 
        ROS_INFO("The return status of servo_j:%d",sdk_res);
        ROS_INFO("Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
        }

    while(true)
    {
        if(jointStates(joint_pose))
        {
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }

        if ( ret_preempt = as->isPreemptRequested())      
        {
            robot.motion_abort();
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }
        ros::Duration(0.5).sleep();
    }
as->setSucceeded();    
ros::Duration(0.5).sleep();
}

//Send the joint value of the physical robot to move_group
void joint_states_callback(ros::Publisher joint_states_pub)
{
    sensor_msgs::JointState joint_position;
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(robotstatus.joint_position[i]);
        int j = i + 1;
        joint_position.name.push_back("joint_" + to_string(j));
    }
    joint_position.header.stamp = ros::Time::now();
    joint_states_pub.publish(joint_position);
}

bool stop_move_callback(std_srvs::Empty::Request &request,
                        std_srvs::Empty::Response &response)
{
    //Initialize jog related parameters
    int ret = robot.motion_abort();
    switch(ret)
    {
        case 0:
            ROS_INFO("stop_move has been executed");
            break;
        default:
            ROS_INFO("error occurred: %s", mapErr[ret].c_str());
            return false;
    }
    return true;
}

bool set_io_callback(jaka_msgs::SetIO::Request &request,
                     jaka_msgs::SetIO::Response &response)
{   
    IOType type;
    int ret;
    switch(request.type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    float value = request.value;
    string signal = request.signal;
    int index = request.index;
    if(signal == "digital")
    {      
        BOOL digital_value;
        if(value)
        {
            digital_value = TRUE;
        }
        else
        {
            digital_value = FALSE;
        }
        ret = robot.set_digital_output(type, index, digital_value);
    }
    else if(signal == "analog")
    {
        ret = robot.set_analog_output(type, index, value);
    }
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "set IO has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}



bool get_io_callback(jaka_msgs::GetIO::Request &request,
                     jaka_msgs::GetIO::Response &response)
{   
    IOType type;
    int ret;
    BOOL digital_result;
    float analog_result;
    switch(request.type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    string signal = request.signal;
    int index = request.index;
    int path = request.path;
    if(signal == "digital")
    {       
        if(path == 0)
        {
            ret = robot.get_digital_input(type, index, &digital_result);
        }
        else if(path == 1)
        {
            ret = robot.get_digital_output(type, index, &digital_result);
        }
        switch(ret)
        {
            case 0:
                response.value = float(digital_result);
                response.message = "get IO has been executed";
                break;
            default:
                response.value = -999999;
                response.message = "error occurred:" + mapErr[ret];
        }
        return true;
    }
    else if(signal == "analog")
    {
        if(path == 0)
        {
            ret = robot.get_analog_input(type, index, &analog_result);
        }
        else if(path == 1)
        {
            ret = robot.get_analog_output(type, index, &analog_result);
        }
        switch(ret)
        {
            case 0:
                response.value = analog_result;
                response.message = "get IO has been executed";
                break;
            default:
                response.value = -999999;
                response.message = "error occurred:" + mapErr[ret];

        }
    return true;
    }
    
}

bool clear_err_callback(std_srvs::Empty::Request &request,
                        std_srvs::Empty::Response &response)
{
    BOOL in_collision = false;
    int ret = robot.is_in_collision(&in_collision);
    if(ret) {
        ROS_INFO("error occurred: %s", mapErr[ret].c_str());
        return false;
    }    
    if (in_collision){
        ret = robot.collision_recover();
        switch(ret)
        {
            case 0:
                ROS_INFO("recovery from collision has been executed");
                break;
            default:
                ROS_INFO("error occurred: %s", mapErr[ret].c_str());
                return false;
        }
    } else {
        ROS_INFO("robot is not collision and don't need to recovery!");
    }
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moveit_server");
    ros::NodeHandle nh;
    string default_ip = "10.5.5.100";
    string default_model = "zu3";
    string robot_ip = nh.param("ip", default_ip);
    string robot_model = nh.param("model", default_model);
    robot.login_in(robot_ip.c_str());
    // robot.set_status_data_update_time_interval(100);
    ros::Rate rate(125);
    robot.servo_move_enable(false);
    ros::Duration(0.5).sleep();
    //Set filter parameter
    robot.servo_move_use_joint_LPF(0.5);
    robot.power_on();
    robot.enable_robot();
    //Create topic "/joint_states"
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    //Create service to stop move
    ros::ServiceServer stop_move_service = nh.advertiseService("/jaka_driver/stop_move",stop_move_callback);
    //Create service to Set IO
    ros::ServiceServer set_io_service = nh.advertiseService("/jaka_driver/set_io",set_io_callback);
    //Create service to Get IO
    ros::ServiceServer get_io_service = nh.advertiseService("/jaka_driver/get_io",get_io_callback);
    //Create service to recovery from collision
    ros::ServiceServer clear_err_service = nh.advertiseService("/jaka_driver/clear_err",clear_err_callback);


    //Create action server object
    Server moveit_server(nh, "/jaka_"+robot_model+"_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
	moveit_server.start();
    cout << "==================Moveit Start==================" << endl;

    while(ros::ok())
    {
        //Report robot joint information to RVIZ
        joint_states_callback(joint_states_pub);
        rate.sleep();
        ros::spinOnce();
    }
    //ros::spin();
}