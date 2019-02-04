#include "iwtros_gazebo/conveyor_spawner.h"
//hello for github
int main(int argc, char **argv){
    ros::init(argc, argv, "conveyor_object_spawner");
    ros::NodeHandle pnh("~"), nh;

    XmlRpc::XmlRpcValue spawner_params;
    if(!pnh.getParam("spawner", spawner_params)){
        ROS_ERROR("Failed to get spawner parameters");
        return -1;
    }

    iwtros::simulation::ConveyorSpawner spawner (nh);
    if(!spawner.init(spawner_params)){
        ROS_ERROR("Failed to initialize spawner");
        return -1;
    }
    spawner.run();
    return 0;
}
