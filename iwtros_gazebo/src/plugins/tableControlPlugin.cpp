#include<functional>
#include<gazebo-7/gazebo/gazebo.hh>
#include<gazebo-7/gazebo/physics/physics.hh>
#include<gazebo-7/gazebo/common/common.hh>
#include<ignition/math2/ignition/math/Vector3.hh>
#include<thread>
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/subscribe_options.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/Twist.h>
#include<gazebo-7/gazebo/transport/transport.hh>
#include<gazebo-7/gazebo/msgs/msgs.hh>

namespace gazebo{
    class TableMover : public ModelPlugin{

    };
}