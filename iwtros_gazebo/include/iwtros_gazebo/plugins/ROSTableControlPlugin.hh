#ifndef _ROS_TABLE_CONTROL_PLUGINS_HH_
#define _ROS_TABLE_CONTROL_PLUGINS_HH_

#include<functional>
#include<gazebo/gazebo.hh>
#include<gazebo/util/system.hh>
#include<gazebo/physics/physics.hh>
#include<gazebo/physics/PhysicsTypes.hh>
#include<gazebo/common/common.hh>
#include<gazebo/common/Plugin.hh>
#include<gazebo/common/UpdateInfo.hh>
#include<ignition/math2/ignition/math/Vector3.hh>
#include<thread>
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/subscribe_options.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/Twist.h>
#include<gazebo/transport/transport.hh>
#include<gazebo/transport/Node.hh>
#include<gazebo/transport/Publisher.hh>
#include<gazebo/msgs/msgs.hh>

// for transformation
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/TransformStamped.h>


namespace gazebo{
    class ROSTableControlPlugin : public ModelPlugin{

        public: ROSTableControlPlugin();
        public: virtual ~ROSTableControlPlugin();

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        public: void OnRosMsg_Pos(const geometry_msgs::TwistConstPtr &msg);

        void MoveModel(float lin_x, float lin_y, float lin_z, float ang_x, float ang_y, float ang_z);

        void tfBroadCater(geometry_msgs::Transform crnt_pose);

        public: void OnUpdate();

        public: void QueueTHread();

        //Pointer to the model
        private: physics::ModelPtr model;
        //Pointer to the updated event connection
        private: event::ConnectionPtr updateConnection;
        //Time memory
        double old_sec;
        //direction value
        int direction = 1;
        //Frequency of earthquakes
        double x_axis_pose = 1.0;
        //Magnitude of oscillation
        double y_axis_pose = 1.0;
        //table movement topic name
        private: std::string table_cmd_vel;
        private: std::string robotNamespace_;
        /*These offset are for the URDF reference
        howver it is not visible in the gazebo
        therefore this offset*/
        private: std::float_t delta_theta, crnt_theta, old_theta;
        private: std::float_t lin_Y; //, crnt_x, old_x;
        private: std::float_t lin_X; //, crnt_y, old_y;
        private: std::float_t yawValue; //, current_time, prev_time, dt;
        private: std::int64_t loop_counter = 0;
        private: std::float_t off_yaw;
        private: std::string table_base_link;

        private: geometry_msgs::Transform offsets;
        private: geometry_msgs::Transform prev_offsets;

        //brief a node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;
        // ROS subscriber
        private: ros::Subscriber rosSub;
        // ROS Callbackqueues
        private: ros::CallbackQueue rosQueue;
        // thread the keeps running rosQueue
        private: std::thread rosQueueThread;
        // ROS subscriber
        private: ros::Subscriber rosSub2;
        // ROS Callbackqueues
        private: ros::CallbackQueue rosQueue2;
        // thread the keeps running rosQueue
        private: std::thread rosQueueThread2;
    };
}
#endif
