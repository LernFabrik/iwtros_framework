/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <vector>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"
#include "wsg_50_common/Cmd.h"
#include "wsg_50_common/OpticalCMD.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

#define OPT_SET_REG 0
#define OPT_ADD_REG 1
#define OPT_DEL_REG 2
#define OPT_READ_REG 3
#define OPT_WRITE_REG 4
#define OPT_CALIB_RESULT 5
#define OPT_CALIB_SEE_THROUGH 6
#define OPT_CALIB_SEE_DARK 7
#define OPT_CALIB_OFFSET 8
#define OPT_CALIB_CROSS_TALK 9

#define MAX_REGISTERS 10
#define DEVICE_COUNT 8

#define FMF_READ_FINGER_1 0x63
#define FMF_READ_FINGER_2 0x73

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


float increment;
bool objectGraspped;

int g_timer_cnt = 0;
ros::Publisher g_pub_state, g_pub_joint, g_pub_moving, g_pub_optical;
bool g_ismoving = false, g_mode_script = false, g_mode_periodic = false, g_mode_polling = false;
float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
std::vector<uint16_t> g_reg_addrs;
std::vector<uint8_t> g_reg_lengths;
//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------


bool moveSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
  		ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
		res.error = move(req.width, req.speed, false);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move(req.width, req.speed, false);
	}

	ROS_INFO("Target position reached.");
  	return true;
}

bool graspSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
        ROS_INFO("Grasping object at %f with %f mm/s.", req.width, req.speed);
		res.error = grasp(req.width, req.speed);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = grasp(req.width, req.speed);
	}

	ROS_INFO("Object grasped correctly.");
	objectGraspped=true;
  	return true;
}

bool incrementSrv(wsg_50_common::Incr::Request &req, wsg_50_common::Incr::Response &res)
{
	if (req.direction == "open"){

		if (!objectGraspped){
      ROS_INFO("Opening...");
			float currentWidth = getOpening();
			float nextWidth = currentWidth + req.increment;
			if ( (currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth >= GRIPPER_MAX_OPEN){
				//grasp(GRIPPER_MAX_OPEN, 1);
				move(GRIPPER_MAX_OPEN,1, true);
				currentWidth = GRIPPER_MAX_OPEN;
			}
		}else{
			ROS_INFO("Releasing object...");
			release(GRIPPER_MAX_OPEN, 20);
			objectGraspped = false;
		}
	}else if (req.direction == "close"){
    ROS_INFO("Got close request");
		if (!objectGraspped){
      ROS_INFO("Beginning to close..");
			float currentWidth = getOpening();
			float nextWidth = currentWidth - req.increment;

			if ( (currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth <= GRIPPER_MIN_OPEN){
				//grasp(GRIPPER_MIN_OPEN, 1);
				move(GRIPPER_MIN_OPEN,1, true);
				currentWidth = GRIPPER_MIN_OPEN;
			}
			ROS_INFO("Current width: %f", currentWidth);
		}
	}
	return true;
}

bool releaseSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
  		ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		res.error = release(req.width, req.speed);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = release(req.width, req.speed);
	}
	ROS_INFO("Object released correctly.");
  	return true;
}

bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Homing...");
	homing();
	ROS_INFO("Home position reached.");
	return true;
}

bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_WARN("Stop!");
	stop();
	ROS_WARN("Stopped.");
	return true;
}

bool setAccSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setAcceleration(req.val);
	return true;
}

bool setForceSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setGraspingForceLimit(req.val);
	return true;
}

bool opticalCMD(uint8_t finger_idx, uint8_t cmd, const std::vector<uint8_t>& tx_data, std::vector<uint8_t>& rx_data) {
  if(finger_idx > 1) {
    ROS_ERROR("Finger idx must be less than 2, got %d", finger_idx);
    return false;
  }

  unsigned char payload[2+tx_data.size()];
  payload[0] = finger_idx;
  payload[1] = cmd;
  for(unsigned int i = 0; i < tx_data.size(); i++) {
    payload[2+i] = tx_data[i];
  }
  unsigned char* resp;
  unsigned int resp_len;

  switch(cmd) {
  case OPT_SET_REG:
  {
    if(tx_data.size() != 3) {
      ROS_ERROR("Expected payload of reg address (2 bytes) and reg length (1 byte), aborting...");
      return false;
    }
    if(tx_data[2] > 4) {
      ROS_WARN("Using reg lengths greater than 4 could cause overflow");
    }
    g_reg_addrs.clear();
    g_reg_lengths.clear();
    g_reg_addrs.push_back((tx_data[0] << 8) | tx_data[1]);
    g_reg_lengths.push_back(tx_data[2]);
    cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    break;
  }
  case OPT_ADD_REG:
  {
    if(tx_data.size() != 3) {
      ROS_ERROR("Expected payload of reg address (2 bytes) and reg length (1 byte), aborting...");
      return false;
    }
    if(tx_data[2] > 4) {
      ROS_WARN("Using reg lengths greater than 4 could cause overflow");
    }
    g_reg_addrs.push_back((tx_data[0] << 8) | tx_data[1]);
    g_reg_lengths.push_back(tx_data[2]);
    cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    break;
  }
  case OPT_DEL_REG:
  {
    if(tx_data.size() != 2) {
      ROS_ERROR("Expected payload of reg address (2 bytes), aborting...");
      return false;
    }
    uint16_t del_address = tx_data[0] << 8 | tx_data[1];
    bool deleted = false;
    for(unsigned int i = g_reg_addrs.size()-1; i >= 0; i--) {
      if(g_reg_addrs[i] == del_address) {
        g_reg_addrs.erase(g_reg_addrs.begin()+i);
        g_reg_lengths.erase(g_reg_lengths.begin()+i);
        deleted = true;
      }
    }
    if(deleted) {
      cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    } else {
      ROS_ERROR("Could not delete register, aborting...");
      return false;
    }
    break;
  }
  case OPT_READ_REG:
  {
    if(g_reg_addrs.size() <= 0) {
      ROS_ERROR("Cannot read, there are no registers");
      return false;
    }
    int read_length = 0;
    for(unsigned int i = 0; i < g_reg_lengths.size(); i++) {
      read_length += g_reg_lengths[i];
    }
    if(tx_data.size() != DEVICE_COUNT*read_length) {
      unsigned char new_payload[2+DEVICE_COUNT*read_length];
      new_payload[0] = finger_idx;
      new_payload[1] = cmd;
      cmd_submit(0xB3, new_payload, 2+tx_data.size(), true, &resp, &resp_len);
    } else {
      cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    }
    break;
  }
  case OPT_WRITE_REG:
  {

    int write_length = 0;
    for(unsigned int i = 0; i < g_reg_lengths.size(); i++) {
      write_length += g_reg_lengths[i];
    }

    if(tx_data.size() != 1+write_length) {
      ROS_ERROR("Expected payload of device idx (1 byte) and %d bytes of data, aborting...", write_length);
      return false;
    }
    cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    break;
  }
  case OPT_CALIB_RESULT:
  {
    if(tx_data.size() != 2*DEVICE_COUNT) {
      unsigned char new_payload[2+2*DEVICE_COUNT];
      new_payload[0] = finger_idx;
      new_payload[1] = cmd;
      cmd_submit(0xB3, new_payload, 2+tx_data.size(), true, &resp, &resp_len) ;
    } else {
      cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    }
    break;
  }
  case OPT_CALIB_SEE_THROUGH:
  case OPT_CALIB_SEE_DARK:
  case OPT_CALIB_OFFSET:
  case OPT_CALIB_CROSS_TALK:
    if(tx_data.size() != 2) {
      ROS_ERROR("Expected payload of # of samples (1 byte) and target (1 byte), aborting...");
      return false;
    }
    cmd_submit(0xB3, payload, 2+tx_data.size(), true, &resp, &resp_len);
    break;
  default:
    ROS_ERROR("Received unknown cmd: %d", cmd);
    return false;
  }

  rx_data.clear();
  for(unsigned int i = 0; i < resp_len; i++) {
    rx_data.push_back(resp[i]);
  }
  return true;

}

bool opticalCMDSrv(wsg_50_common::OpticalCMD::Request &req, wsg_50_common::OpticalCMD::Response &res)
{
  res.success = opticalCMD(req.finger_idx, req.cmd, req.tx_data, res.rx_data);
  return true;
}

bool fmfCMD(uint8_t finger_idx, uint8_t cmd, const std::vector<uint8_t>& tx_data, std::vector<uint8_t>& rx_data) {
  ROS_INFO("Received fmf request");
  if(finger_idx > 1) {
    ROS_ERROR("Finger idx must be less than 2, got %d", finger_idx);
    return false;
  }

  unsigned char payload[tx_data.size()];
  for(unsigned int i = 0; i < tx_data.size(); i++) {
    payload[i] = tx_data[i];
  }
  unsigned char* resp;
  unsigned int resp_len;


  switch(cmd) {
  case FMF_READ_FINGER_1:
  {
    if(finger_idx != 0) {
      ROS_ERROR("CMD %d expects finger idx to be 0", cmd);
      return false;
    }
    cmd_submit(cmd, payload, tx_data.size(), true, &resp, &resp_len);
    break;
  }

  case FMF_READ_FINGER_2:
  {
    if(finger_idx != 1) {
      ROS_ERROR("CMD %d expects finger idx to be 1", cmd);
      return false;
    }
    cmd_submit(cmd, payload, tx_data.size(), true, &resp, &resp_len);
    break;
  }
  default:
    ROS_ERROR("Received unknown cmd: %d", cmd);
    return false;
  }

  rx_data.clear();
  for(unsigned int i = 0; i < resp_len; i++) {
    rx_data.push_back(resp[i]);
  }
  return true;
}

bool fmfCMDSrv(wsg_50_common::OpticalCMD::Request& req, wsg_50_common::OpticalCMD::Response& res)
{
  res.success = fmfCMD(req.finger_idx, req.cmd, req.tx_data, res.rx_data);
  return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ack_fault();
	return true;
}

/** \brief Callback for goal_position topic (in appropriate modes) */
void position_cb(const wsg_50_common::Cmd::ConstPtr& msg)
{
    g_speed = msg->speed; g_goal_position = msg->pos;
    // timer_cb() will send command to gripper

    if (g_mode_periodic) {
        // Send command to gripper without waiting for a response
        // read_thread() handles responses
        // read/write may be simultaneous, therefore no mutex
        stop(true);
        if (move(g_goal_position, g_speed, false, true) != 0)
            ROS_ERROR("Failed to send MOVE command");
    }
}

/** \brief Callback for goal_speed topic (in appropriate modes) */
void speed_cb(const std_msgs::Float32::ConstPtr& msg)
{
    g_goal_speed = msg->data; g_speed = msg->data;
    // timer_cb() will send command to gripper
}

/** \brief Loop for state polling in modes script and polling. Also sends command in script mode. */
void timer_cb(const ros::TimerEvent& ev)
{

  std::vector<uint8_t> tx_data;
  std::vector<uint8_t> rx_data;
  opticalCMD(0, OPT_READ_REG, tx_data, rx_data);

  std_msgs::UInt8MultiArray optical_msg;
  optical_msg.data = rx_data;
  g_pub_optical.publish(optical_msg);

}


/** \brief Reads gripper responses in auto_update mode. The gripper pushes state messages in regular intervals. */
void read_thread(int interval_ms)
{
    ROS_INFO("Thread started");

    status_t status;
    int res;
    bool pub_state = false;

    double rate_exp = 1000.0 / (double)interval_ms;
    std::string names[3] = { "opening", "speed", "force" };

    // Prepare messages
    wsg_50_common::Status status_msg;
    status_msg.status = "UNKNOWN";

    sensor_msgs::JointState joint_states;
    joint_states.header.frame_id = "wsg50_base_link";
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
            g_pub_moving.publish(moving_msg);
            g_ismoving = motion;
        }

        // ***** PUBLISH state message & joint message
        if (pub_state) {
            pub_state = false;
            g_pub_state.publish(status_msg);

            joint_states.header.stamp = ros::Time::now();;
            joint_states.position[0] = -status_msg.width/2000.0;
            joint_states.position[1] = status_msg.width/2000.0;
            joint_states.velocity[0] = status_msg.speed/1000.0;
            joint_states.velocity[1] = status_msg.speed/1000.0;
            joint_states.effort[0] = status_msg.force;
            joint_states.effort[1] = status_msg.force;
            g_pub_joint.publish(joint_states);
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

void sigint_handler(int sig) {
    ROS_INFO("Exiting...");
    g_mode_periodic = false;
    g_mode_script = false;
    g_mode_polling = false;
    ros::shutdown();
}

/**
 * The main function
 */

int main( int argc, char **argv )
{
   ros::init(argc, argv, "wsg_50");
   ros::NodeHandle nh("~");
   signal(SIGINT, sigint_handler);

   std::string ip, protocol, com_mode;
   int port, local_port;
   double rate, grasping_force;
   bool use_udp = false;

   nh.param("ip", ip, std::string("192.168.1.20"));
   nh.param("port", port, 1000);
   nh.param("local_port", local_port, 1501);
   nh.param("protocol", protocol, std::string(""));
   nh.param("com_mode", com_mode, std::string(""));
   nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
   nh.param("grasping_force", grasping_force, 0.0);

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

   // Connect to device using TCP/USP
   int res_con;
   if (!use_udp)
       res_con = cmd_connect_tcp( ip.c_str(), port );
   else
       res_con = cmd_connect_udp(local_port, ip.c_str(), port );

   if (res_con == 0 ) {
        ROS_INFO("Gripper connection stablished");

		// Services
        ros::ServiceServer moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS, opticalCMDSS, fmfCMDSS;

        if (g_mode_script || g_mode_polling) {
            moveSS = nh.advertiseService("move", moveSrv);
            graspSS = nh.advertiseService("grasp", graspSrv);
            releaseSS = nh.advertiseService("release", releaseSrv);
            homingSS = nh.advertiseService("homing", homingSrv);
            stopSS = nh.advertiseService("stop", stopSrv);
            ackSS = nh.advertiseService("ack", ackSrv);
            incrementSS = nh.advertiseService("move_incrementally", incrementSrv);

            setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
            setForceSS = nh.advertiseService("set_force", setForceSrv);

            opticalCMDSS = nh.advertiseService("optical_cmd", opticalCMDSrv);
            fmfCMDSS = nh.advertiseService("fmf_cmd", fmfCMDSrv);
        }

		// Subscriber
        ros::Subscriber sub_position, sub_speed;
        if (g_mode_script || g_mode_periodic)
            sub_position = nh.subscribe("goal_position", 5, position_cb);
        if (g_mode_script)
            sub_speed = nh.subscribe("goal_speed", 5, speed_cb);

		// Publisher
		g_pub_state = nh.advertise<wsg_50_common::Status>("status", 1000);
		g_pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        if (g_mode_script || g_mode_periodic)
            g_pub_moving = nh.advertise<std_msgs::Bool>("moving", 10);
            g_pub_optical = nh.advertise<std_msgs::UInt8MultiArray>("optical_data", 1);

		ROS_INFO("Ready to use. Homing and taring now...");
		homing();
        ros::Duration(0.5).sleep();
        //doTare();

		if (grasping_force > 0.0) {
			ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
			setGraspingForceLimit(grasping_force);
		}

        ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
        std::thread th;
        ros::Timer tmr;
        //if (g_mode_polling || g_mode_script)
           // tmr = nh.createTimer(ros::Duration(1.0/rate), timer_cb);
        if (g_mode_periodic)
             th = std::thread(read_thread, (int)(1000.0/rate));

        ros::spin();

	} else {
        ROS_ERROR("Unable to connect, please check the port and address used.");
	}

   ROS_INFO("Exiting...");
   g_mode_periodic = false;
   g_mode_script = false;
   g_mode_polling = false;
   sleep(1);
   cmd_disconnect();

	return 0;

}


//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
