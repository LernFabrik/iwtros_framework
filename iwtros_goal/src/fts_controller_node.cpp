#include <iwtros_goal/fts_controller.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "fts_controller_node");
    iwtros::ftsControl controller;
    ros::spin();
    return 0;
}