/*
 * RobotisController.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_ROBOTIS_CONTROLLER_ROBOTISCONTROLLER_H_
#define ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_ROBOTIS_CONTROLLER_ROBOTISCONTROLLER_H_


#include <ros/ros.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include "robotis_device/Robot.h"
#include "robotis_framework_common/MotionModule.h"

#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"

// TODO: TEMPORARY CODE !!
#include "dynamixel_sdk/GroupBulkRead.h"
#include "dynamixel_sdk/GroupSyncWrite.h"

namespace ROBOTIS
{

enum CONTROLLER_MODE
{
    MOTION_MODULE_MODE,
    DIRECT_CONTROL_MODE
};

class RobotisController
{
private:
    static RobotisController   *unique_instance_;

    boost::thread               queue_thread_;
    boost::thread               gazebo_thread_;
    boost::mutex                queue_mutex_;

    bool                        init_pose_loaded_;
    bool                        is_timer_running_;
    bool                        stop_timer_;
    pthread_t                   timer_thread_;
    CONTROLLER_MODE             controller_mode_;

    std::list<MotionModule *>       modules_;
    std::vector<GroupSyncWrite *>   direct_sync_write_;

    RobotisController();

    void QueueThread();
    void GazeboThread();

    bool CheckTimerStop();
    void InitSyncWrite();

public:
    static const int            CONTROL_CYCLE_MSEC  = 8;    // 8 msec

    bool                        DEBUG_PRINT;
    Robot                      *robot;

    bool                        gazebo_mode;
    std::string                 gazebo_robot_name;

    // TODO: TEMPORARY CODE !!
    std::map<std::string, GroupBulkRead *>  port_to_bulk_read;
    std::map<std::string, GroupSyncWrite *> port_to_sync_write;

    /* publisher */
    ros::Publisher  goal_joint_state_pub;
    ros::Publisher  present_joint_state_pub;
    ros::Publisher  current_module_pub;

    std::map<std::string, ros::Publisher> gazebo_joint_pub;

    static void *ThreadProc(void *param);
    static RobotisController *GetInstance() { return unique_instance_; }

    bool    Initialize(const std::string robot_file_path, const std::string init_file_path);
    void    Process();
    void    AddModule(MotionModule *module);
    void    RemoveModule(MotionModule *module);

    void    StartTimer();
    void    StopTimer();
    bool    IsTimerRunning();

    void    LoadOffset(const std::string path);

    /* ROS Topic Callback Functions */
    void    SyncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr &msg);
    void    SetControllerModeCallback(const std_msgs::String::ConstPtr &msg);
    void    SetJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void    SetCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);
    bool    GetCtrlModuleCallback(robotis_controller_msgs::GetJointModule::Request &req, robotis_controller_msgs::GetJointModule::Response &res);

    void    GazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

    int     Ping        (const std::string joint_name, UINT8_T *error = 0);
    int     Ping        (const std::string joint_name, UINT16_T* model_number, UINT8_T *error = 0);

    int     Action      (const std::string joint_name);
    int     Reboot      (const std::string joint_name, UINT8_T *error = 0);
    int     FactoryReset(const std::string joint_name, UINT8_T option = 0, UINT8_T *error = 0);

    int     Read        (const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);
    int     ReadCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T *data, UINT8_T *error = 0);

    int     Read1Byte   (const std::string joint_name, UINT16_T address, UINT8_T *data, UINT8_T *error = 0);
    int     Read2Byte   (const std::string joint_name, UINT16_T address, UINT16_T *data, UINT8_T *error = 0);
    int     Read4Byte   (const std::string joint_name, UINT16_T address, UINT32_T *data, UINT8_T *error = 0);

    int     Write       (const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);
    int     WriteCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T data, UINT8_T *error = 0);

    int     Write1Byte  (const std::string joint_name, UINT16_T address, UINT8_T data, UINT8_T *error = 0);
    int     Write2Byte  (const std::string joint_name, UINT16_T address, UINT16_T data, UINT8_T *error = 0);
    int     Write4Byte  (const std::string joint_name, UINT16_T address, UINT32_T data, UINT8_T *error = 0);

    int     RegWrite    (const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error = 0);
};

}


#endif /* ROBOTIS_FRAMEWORK_ROBOTIS_CONTROLLER_INCLUDE_ROBOTIS_CONTROLLER_ROBOTISCONTROLLER_H_ */
