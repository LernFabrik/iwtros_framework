/*
* This ws50 node is created for ROS framework in the LernFabrik project under 
* IWT Wirtschaft und Technik GmbH
* This file is the modification of the old wsg50 node
* author: Vishnuprasad Prachandanhanu
* Date: 11.06.2019
*/

#include "wsg_50/main_action_diver.h"

namespace iwtros{

    template<typename T_action, typename T_goal, typename T_result>
    void wsg50::handleError(actionlib::SimpleActionServer<T_action>* server,
                        std::function<bool(const T_goal&)> handler,
                        const T_goal& goal){
        T_result result;
        try{
            result.success = handler(goal);
            server->setSucceeded(result);
        }catch(int e){
            ROS_ERROR_STREAM("Error handling error" << e);
            result.success = false;
            result.error = e;
            server->setAborted(result);
        }
    }

    void wsg50::gripperCommandExecution(const control_msgs::GripperCommandGoalConstPtr& goal){
        auto gripper_command_handler = [goal, this](auto default_speed){
            /*HACK: Gripper motion given by the MoveIt is for one 
            finger and other should mimic respectively.
            For the single gripper position is from the midway*/
            double target_width = std::abs(2000*goal->command.position);
            if(target_width > GRIPPER_MAX_OPEN || target_width < 0.0){
                ROS_ERROR_STREAM("GripperServer: Commanding out of range with max_dith= " << GRIPPER_MAX_OPEN << "command = " << target_width);
                return false;
            }
            constexpr double kSamplePositionThreshold = 1e-4;
            if(std::abs(target_width - this->status_msg.width/1000) < kSamplePositionThreshold){
                return true;
            }
            if(target_width >= this->status_msg.width/1000){
                ROS_WARN_STREAM("Executing move command current = " <<  this->status_msg.width/1000 << " width");
                return move(target_width, default_speed, false, false);
            }
            return grasp(target_width, default_speed);

        };

        try{
            if(gripper_command_handler(this->speed)){
                if(this->status == E_SUCCESS){
                    ROS_WARN("Successful gripper action ");
                    control_msgs::GripperCommandResult result;
                    result.effort = 0.0;
                    result.position = this->status_msg.width/1000;
                    result.reached_goal = static_cast<decltype(result.reached_goal)>(true);
                    result.stalled = static_cast<decltype(result.stalled)>(false);
                    gs_.setSucceeded(result);
                    return;
                }
            }
        }catch(const std::exception& e){
            std::cerr << e.what() << '\n';
            ROS_ERROR("Failed to move the gripper");
        }
        gs_.setAborted();
    }
        
    wsg50::wsg50(ros::NodeHandle& nh):gs_(nh, "gripper_action", boost::bind(&wsg50::gripperCommandExecution, this, _1), false), _nh(nh){
        _nh.param("ip", ip, std::string("172.31.1.160"));
        _nh.param("port", port, 1000);
        _nh.param("local_port", local_port, 1501);
        _nh.param("protocol", protocol, std::string("tcp"));
        _nh.param("com_mode", com_mode, std::string("auto_update"));
        _nh.param("rate", rate, 50.0); // With custom script, up to 30Hz are possible
        _nh.param("grasping_force", grasping_force, 40.0);
        _nh.param("speed", speed, 40.0);

        if (protocol == "udp")
            use_udp = true;
        else
            protocol = "tcp";
        if (com_mode == "script")
            g_mode_script = true;
        else if (com_mode == "auto_update")
            g_mode_periodic = true;
        else {
            com_mode = "polling";
            g_mode_polling = true;
        }

        ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());

        int res_con;
        if (!use_udp)
            res_con = cmd_connect_tcp( ip.c_str(), port );
        else
            res_con = cmd_connect_udp(local_port, ip.c_str(), port );

        if (res_con == 0 ) {
            ROS_INFO("Gripper connection stablished");

            /*Initialize the action handlers */
            
            auto homming_handler = [this](auto&& goal){return this->hommingAction(goal);};
            auto stop_handler = [this](auto&& goal){return this->stopAction(goal);};
            auto move_handler = [this](auto&& goal){return this->moveAction(goal);};
            auto grasp_handler = [this](auto&& goal){return this->graspAction(goal);};
            /*Start the action servers */
            if(g_mode_script || g_mode_polling || g_mode_periodic){
                /*Bellow action server is not coming up!*/
                actionlib::SimpleActionServer<wsg_50_common::HommingAction> homming_action_server(
                    _nh, "homming", 
                    [=, &homming_action_server](auto&& goal){
                        return this->handleError<wsg_50_common::HommingAction, wsg_50_common::HommingGoalConstPtr,
                                            wsg_50_common::HommingResult>(&homming_action_server, homming_handler, goal);
                    }, false);
                
                actionlib::SimpleActionServer<wsg_50_common::StopAction> stop_action_server(
                    _nh, "stop",
                    [=, &stop_action_server](auto&& goal){
                        return this->handleError<wsg_50_common::StopAction, wsg_50_common::StopGoalConstPtr,
                                                wsg_50_common::StopResult>(&stop_action_server, stop_handler, goal);
                    }, false);

                actionlib::SimpleActionServer<wsg_50_common::MoveAction> move_action_server(
                    _nh, "move",
                    [=, &move_action_server](auto&& goal){
                        return this->handleError<wsg_50_common::MoveAction, wsg_50_common::MoveGoalConstPtr,
                                                wsg_50_common::MoveResult>(&move_action_server, move_handler, goal);
                    }, false);
                
                actionlib::SimpleActionServer<wsg_50_common::GraspAction> grasp_action_server(
                    _nh, "grasp",
                    [=, &grasp_action_server](auto&& goal){
                        return this->handleError<wsg_50_common::GraspAction, wsg_50_common::GraspGoalConstPtr,
                                                wsg_50_common::GraspResult>(&grasp_action_server, grasp_handler, goal);
                    }, false);
                
                /*homming_action_server.start();
                stop_action_server.start();
                move_action_server.start();
                grasp_action_server.start();*/

                /*actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_command_action_server(
                    _nh, "wsg50_gripper",
                    [=, &gripper_command_action_server](auto&& goal){
                        ROS_ERROR("Gripper commander server ----");
                        return this->gripperCommandExecution<control_msgs::GripperCommandAction, control_msgs::GripperCommandGoalConstPtr,
                                                                control_msgs::GripperCommandResult>(speed, &gripper_command_action_server, goal);
                    }, false);
                gripper_command_action_server.start();*/
                ROS_WARN("Gripper Command is Started"); 
                /*The proper way to initialize the action server in class*/ 
                gs_.start();
            }

            // Subscribers
            if(g_mode_script || g_mode_periodic)_sub_position = _nh.subscribe<wsg_50_common::Cmd>("goal_position", 5, boost::bind(&wsg50::poseCallback, this, _1));
            if(g_mode_script)_sub_speed = _nh.subscribe<std_msgs::Float32>("goal_speed", 5, boost::bind(&wsg50::speedCallback, this, _1));
            //Publisher
            _pub_state = _nh.advertise<wsg_50_common::Status>("status", 1000);
            _pub_joint = _nh.advertise<sensor_msgs::JointState>("joint_states", 10);
            if(g_mode_script || g_mode_periodic)
                _pub_moving = _nh.advertise<std_msgs::Bool>("moving", 10);
            
            ROS_INFO("Ready to use. Homing now....");
            ack_fault();
            homing();
            grasp(0.0, 50);
            ros::Duration(0.5).sleep();
            if(grasping_force > 0){
                ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
                setGraspingForceLimit(grasping_force);
            }

            ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
            std::thread th;
            ros::Timer tmr;
            //if(g_mode_polling || g_mode_script) tmr = _nh.createTimer(ros::Duration(1.0/rate), boost::bind(&wsg50::timerCallback, this, _1));
            if(g_mode_periodic){
                ROS_INFO("Initializing threading");
                th = std::thread(boost::bind(&wsg50::read_thread, this, _1), (int)(1000.0/rate));
            }

            ros::spin();

        }else{
            ROS_ERROR("Unable to connect. please check the port and ip address used");
        }
        ROS_INFO("Exiting...");
        g_mode_periodic = false;
        g_mode_script = false;
        g_mode_polling = false;
        sleep(1);
        cmd_disconnect();
    }

    wsg50::~wsg50(){
        g_mode_periodic = false;
        g_mode_script = false;
        g_mode_polling = false;
        sleep(1);
        cmd_disconnect();
        ros::shutdown();
    }

    void wsg50::poseCallback(const wsg_50_common::Cmd::ConstPtr& msg){
        g_speed = msg->speed;
        g_goal_position = msg->pos;
        if(g_mode_periodic){
            stop(true);
            if(move(g_goal_position, g_speed, false, true) != false) ROS_INFO("Failed to send Move command");
        }
    }

    void wsg50::speedCallback(const std_msgs::Float32::ConstPtr& msg){
        g_goal_speed = msg->data;
        g_speed = msg->data;
    }

    void wsg50::timerCallback(const ros::TimerEvent& ev){
        gripper_response info;
        float acc = 0.0;
        info.speed = 0.0;

        if (g_mode_polling) {
            const char * state = systemState();
            if (!state)
                return;
            info.state_text = std::string(state);
            info.position = getOpening();
            acc = getAcceleration();
            info.f_motor = getForce();//getGraspingForce();

        } else if (g_mode_script) {
            // ==== Call custom measure-and-move command ====
            int res = 0;
            if (!std::isnan(g_goal_position)) {
                ROS_INFO("Position command: pos=%5.1f, speed=%5.1f", g_goal_position, g_speed);
                res = script_measure_move(1, g_goal_position, g_speed, info);
            } else if (!std::isnan(g_goal_speed)) {
                ROS_INFO("Velocity command: speed=%5.1f", g_goal_speed);
                res = script_measure_move(2, 0, g_goal_speed, info);
            } else
                res = script_measure_move(0, 0, 0, info);
            if (!std::isnan(g_goal_position))
                g_goal_position = NAN;
            if (!std::isnan(g_goal_speed))
                g_goal_speed = NAN;

            if (!res) {
                ROS_ERROR("Measure-and-move command failed");
                return;
            }

            // ==== Moving msg ====
            if (g_ismoving != info.ismoving) {
                std_msgs::Bool moving_msg;
                moving_msg.data = info.ismoving;
                _pub_moving.publish(moving_msg);
                g_ismoving = info.ismoving;
            }
        } else
            return;

        // ==== Status msg ====
        wsg_50_common::Status status_msg;
        status_msg.status = info.state_text;
        status_msg.width = info.position;
        status_msg.speed = info.speed;
        status_msg.acc = acc;
        status_msg.force = info.f_motor;
        status_msg.force_finger0 = info.f_finger0;
        status_msg.force_finger1 = info.f_finger1;

        _pub_state.publish(status_msg);


        // ==== Joint state msg ====
        // \todo Use name of node for joint names
        sensor_msgs::JointState joint_states;
        joint_states.header.stamp = ros::Time::now();;
        joint_states.header.frame_id = "wsg50_base_link";
        joint_states.name.push_back("wsg50_finger_left_joint");
        joint_states.name.push_back("wsg50_finger_right_joint");
        joint_states.position.resize(2);

        joint_states.position[0] = -info.position/2000.0;
        joint_states.position[1] = info.position/2000.0;
        joint_states.velocity.resize(2);
        joint_states.velocity[0] = info.speed/1000.0;
        joint_states.velocity[1] = info.speed/1000.0;
        joint_states.effort.resize(2);
        joint_states.effort[0] = info.f_motor;
        joint_states.effort[1] = info.f_motor;

        _pub_joint.publish(joint_states);
    }

    void wsg50::read_thread(int interval_ms){
        ROS_INFO("Thread started");
        int res;
        bool pub_state = false;

        double rate_exp = 1000.0 / (double)interval_ms;
        std::string names[3] = { "opening", "speed", "force" };

        // Prepare messages
        status_msg.status = "UNKNOWN";

        sensor_msgs::JointState joint_states;
        joint_states.header.frame_id = "";
        joint_states.name.push_back("wsg50_finger_left_joint");
        joint_states.name.push_back("wsg50_finger_right_joint");
        joint_states.position.resize(2);
        joint_states.velocity.resize(2);
        joint_states.effort.resize(2);

        // Request automatic updates (error checking is done below)
        getOpening(interval_ms);
        getSpeed(interval_ms);
        getForce(interval_ms);


        msg_t msg; msg.id = 0; msg.data = 0; msg.len = 0;
        int cnt[3] = {0,0,0};
        auto time_start = std::chrono::system_clock::now();


        while (g_mode_periodic) {
            // Receive gripper response
            msg_free(&msg);
            res = msg_receive( &msg );
            if (res < 0 || msg.len < 2) {
                ROS_ERROR("Gripper response failure");
                continue;
            }

            float val = 0.0;
            status = cmd_get_response_status(msg.data);

            // Decode float for opening/speed/force
            if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
                if (status != E_SUCCESS) {
                    ROS_ERROR("Gripper response failure for opening/speed/force\n");
                    continue;
                }
                val = convert(&msg.data[2]);
            }

            // Handle response types
            int motion = -1;
            switch (msg.id) {
            /*** Opening ***/
            case 0x43:
                status_msg.width = val;
                pub_state = true;
                cnt[0]++;
                break;

            /*** Speed ***/
            case 0x44:
                status_msg.speed = val;
                cnt[1]++;
                break;

            /*** Force ***/
            case 0x45:
                status_msg.force = val;
                cnt[2]++;
                break;

            /*** Move ***/
            // Move commands are sent from outside this thread
            case 0x21:
                if (status == E_SUCCESS) {
                    ROS_INFO("Position reached");
                    motion = 0;
                } else if (status == E_AXIS_BLOCKED) {
                    ROS_INFO("Axis blocked");
                    motion = 0;
                } else if (status == E_CMD_PENDING) {
                    ROS_INFO("Movement started");
                    motion = 1;
                } else if (status == E_ALREADY_RUNNING) {
                    ROS_INFO("Movement error: already running");
                } else if (status == E_CMD_ABORTED) {
                    ROS_INFO("Movement aborted");
                    motion = 0;
                } else {
                    ROS_INFO("Movement error");
                    motion = 0;
                }
                break;

            /*** Stop ***/
            // Stop commands are sent from outside this thread
            case 0x22:
                // Stop command; nothing to do
                break;
            default:
                ROS_INFO("Received unknown respone 0x%02x (%2dB)\n", msg.id, msg.len);
            }

            // ***** PUBLISH motion message
            if (motion == 0 || motion == 1) {
                std_msgs::Bool moving_msg;
                moving_msg.data = motion;
                _pub_moving.publish(moving_msg);
                g_ismoving = motion;
            }

            // ***** PUBLISH state message & joint message
            if (pub_state) {
                pub_state = false;
                _pub_state.publish(status_msg);

                joint_states.header.stamp = ros::Time::now();;
                joint_states.position[0] = -status_msg.width/2000.0;
                joint_states.position[1] = status_msg.width/2000.0;
                joint_states.velocity[0] = status_msg.speed/1000.0;
                joint_states.velocity[1] = status_msg.speed/1000.0;
                joint_states.effort[0] = status_msg.force;
                joint_states.effort[1] = status_msg.force;
                _pub_joint.publish(joint_states);
            }

            // Check # of received messages regularly
            std::chrono::duration<float> t = std::chrono::system_clock::now() - time_start;
            double t_ = t.count();
            if (t_ > 5.0) {
                time_start = std::chrono::system_clock::now();
                //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

                std::string info = "Rates for ";
                for (int i=0; i<3; i++) {
                    double rate_is = (double)cnt[i]/t_;
                    info += names[i] + ": " + std::to_string((int)rate_is) + "Hz, ";
                    if (rate_is == 0.0)
                        ROS_ERROR("Did not receive data for %s", names[i].c_str());
                }
                ROS_DEBUG_STREAM((info + " expected: " + std::to_string((int)rate_exp) + "Hz").c_str());
                cnt[0] = 0; cnt[1] = 0; cnt[2] = 0;
            }
        }

        // Disable automatic updates
        // TODO: The functions will receive an unexpected response
        getOpening(0);
        getSpeed(0);
        getForce(0);

        ROS_INFO("Thread ended");
    }

    int wsg50::moveAction(const wsg_50_common::MoveGoalConstPtr& goal){
        return move(goal->width, goal->speed, false);
    }

    int wsg50::hommingAction(const wsg_50_common::HommingGoalConstPtr& goal){
        return homing();
    }

    int wsg50::graspAction(const wsg_50_common::GraspGoalConstPtr& goal){
        return grasp(goal->width, goal->speed);
    }

    int wsg50::stopAction(const wsg_50_common::StopGoalConstPtr& goal){
        return stop();
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "wsg50_gripper");
    ros::NodeHandle nh("~");

    iwtros::wsg50 wsg(nh);

    ros::spin();
}