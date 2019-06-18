#ifndef MAIN_ACTION_DRIVER_H
#define MAIN_ACTION_DRIVER_H

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <functional>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "wsg_50_common/Cmd.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/GraspAction.h"
#include "wsg_50_common/HommingAction.h"
#include "wsg_50_common/MoveAction.h"
#include "wsg_50_common/StopAction.h"
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

namespace iwtros{

    typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> Gripper;

    class wsg50{
    private:
        float increment;
        bool objectGrasping;
        int g_timer_cnt = 0;
        bool g_ismoving = false, g_mode_script = false, g_mode_periodic = false, g_mode_polling = false;
        float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
        std::string ip, protocol, com_mode;
        int port, local_port;
        double rate, speed, grasping_force;
        bool use_udp = false;
        wsg_50_common::Status status_msg;
        status_t status;

        ros::NodeHandle _nh;
        ros::Subscriber _sub_position, _sub_speed;
        ros::Publisher _pub_state, _pub_joint, _pub_moving;
        void poseCallback(const wsg_50_common::Cmd::ConstPtr& msg);
        void speedCallback(const std_msgs::Float32::ConstPtr& msg);

        /**
         * Timer Callback
         */
        void timerCallback(const ros::TimerEvent& ev);
        /**
         * Read Thread which is alternative to the timer callback
         */
        void read_thread(int interval_ms);
    public:
        wsg50(ros::NodeHandle& nh);
        ~wsg50();

        /**! Move the gripper
         * @return: Currently incompatible with error handle function
         */
        int moveAction(const wsg_50_common::MoveGoalConstPtr& goal);
        /**! Homming the gripper
         * @return: Currently incompatible with error handle function
         */
        int hommingAction(const wsg_50_common::HommingGoalConstPtr& goal);
        /**! Grasp the gripper
         * @return: Currently incompatible with error handle function
         */
        int graspAction(const wsg_50_common::GraspGoalConstPtr& goal);
        /**! Stop the gripper
         * @return: Currently incompatible with error handle function
         */
        int stopAction(const wsg_50_common::StopGoalConstPtr& goal);
        /**Handles the action server and execute the action goal
         * @breif currently this function is not used because action server
         * exepected to come up are not working because of the which not executed 
         * under the class!!
         */
        template<typename T_action, typename T_goal, typename T_result>
        void handleError(actionlib::SimpleActionServer<T_action>* server,
                            std::function<bool(const T_goal&)> handler,
                            const T_goal& goal);
        /**MoveIt controller action callback
         * @breif execute the control managere action server
         */
        void gripperCommandExecution(const control_msgs::GripperCommandGoalConstPtr& goal);

        protected:
            Gripper gs_;
    };
    
}

#endif  //MAIN_ACTION_DRIVER_H
