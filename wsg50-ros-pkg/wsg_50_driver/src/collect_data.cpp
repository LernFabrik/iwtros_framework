#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "wsg_50/hand.h"
#include "wsg_50/optical_weiss_finger.h"
#include "wsg_50_common/WeissFingerData.h"

std::string prepend_zeros(int val, int places) {
    std::stringstream ss;
    ss << val;
    std::string result(ss.str());
    int zeros = places - (((int)(std::floor(std::log10(val)))+1));

    assert(zeros >= 0);

    for(unsigned int i = 0; i < zeros; i++) {
        result = "0"+result;
    }
    return result;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "collect_data_node");
    ros::NodeHandle nh;

    std::string data_path("");
    if(!nh.getParam("/collect_data/data_path", data_path)) {
        ROS_ERROR("Did not receive data path, abort");
        return 1;
    }

    int devices = 0;
    if(!nh.getParam("/collect_data/devices", devices) || devices <= 0) {
        ROS_ERROR("Did not receive valid number of devices, abort");
        return 1;
    }

    double start_width = -1.0;
    if(!nh.getParam("/collect_data/start_width", start_width) || start_width < 0.0) {
        ROS_ERROR("Did not receive valid start width, abort");
        return 1;
    }

    double end_width = -1.0;
    if(!nh.getParam("/collect_data/end_width", end_width) || end_width < 0.0) {
        ROS_ERROR("Did not receive valid end width, abort");
        return 1;
    }

    double delta_width = 0.0;
    if(!nh.getParam("/collect_data/delta_width", delta_width)) {
        ROS_ERROR("Did not receive valid delta width, abort");
        return 1;

    } else if((delta_width < 0.0 && start_width < end_width) || (delta_width > 0.0 && start_width > end_width)) {
        delta_width = -1*delta_width;
    }

    int samples_per_width = 0;
    if(!nh.getParam("/collect_data/samples_per_width", samples_per_width) || samples_per_width <= 0) {
        ROS_ERROR("Did not receive valid samples per width, abort");
        return 1;
    }

    bool load_finger0_calibration;
    if(!nh.getParam("/collect_data/load_finger0_calibration", load_finger0_calibration)) {
        ROS_ERROR("Did not receive load finger 0 calibration flag, abort");
        return 1;
    }

    bool load_finger1_calibration;
    if(!nh.getParam("/collect_data/load_finger1_calibration", load_finger1_calibration)) {
        ROS_ERROR("Did not receive load finger 1 calibration flag, abort");
        return 1;
    }

    Hand hand0(nh, "hand0");
    if(load_finger0_calibration) {
        hand0.load_calibration(0);
    }
    if(load_finger1_calibration) {
        //hand0.load_calibration(1);
        hand0.do_calibration(1);
    }
    hand0.start_reading();

    std::vector<std::ofstream> dist_files(devices);
    for(unsigned int i = 0; i < devices; i++) {
        std::string str_idx(prepend_zeros(i+1, 2));
        dist_files[i].open(data_path+"/dist"+str_idx+".csv");
    }


    double cur_width = start_width;
    do {
        std::vector<wsg_50_common::Status> hand_statuses;
        std::vector<wsg_50_common::WeissFingerData> finger0_samples;
        std::vector<wsg_50_common::WeissFingerData> finger1_samples;
        ROS_INFO("Going to move hand");
        // Tell hand to move
        hand0.move_hand(cur_width);
        ROS_INFO("Done moving hand");
        ros::Duration(0.5).sleep();
        ROS_INFO("Begining data collection");
        // Collect data
        ros::Duration(0.1).sleep();
        ros::Rate sample_rate(25);
        while(finger0_samples.size() < samples_per_width) {
            wsg_50_common::WeissFingerData wfd0 = hand0.get_finger_sample(0);
            wsg_50_common::WeissFingerData wfd1 = hand0.get_finger_sample(1);
            wsg_50_common::Status hs0 = hand0.get_hand_state();
            if(finger0_samples.size() == 0 || (wfd0.stamp - finger0_samples[finger0_samples.size()-1].stamp).toSec() > 0.0075) {
                std::cout << (int) wfd0.data[3*wfd0.data_shape[1]+0] << ' '
                          << (int) wfd0.data[3*wfd0.data_shape[1]+1] << ' '
                          << (int) wfd0.data[3*wfd0.data_shape[1]+2] << ' '
                          << (int) wfd0.data[3*wfd0.data_shape[1]+3] << ' '
                          << (int) wfd0.data[3*wfd0.data_shape[1]+4] << ' '
                          <<       hs0.width                         << ' '
                          <<       wfd1.data[0]                      <<'\n';
                finger0_samples.push_back(wfd0);
                finger1_samples.push_back(wfd1);
                hand_statuses.push_back(hs0);
            }
            sample_rate.sleep();
        }

        ROS_INFO("Data collection complete, writing");
        // Write data
        for(unsigned int i = 0; i < samples_per_width; i++) {
            for(unsigned int j = 0; j < finger0_samples[i].data_shape[0]; j++) {
                for(unsigned int k = 0; k < finger0_samples[i].data_shape[1]-1; k++) {
                    dist_files[j] << (double) finger0_samples[i].data[j*finger0_samples[i].data_shape[1]+k] << ',';
                }
                dist_files[j] << (double) finger0_samples[i].data[(j+1)*finger0_samples[i].data_shape[1]-1] << ',';
                dist_files[j] << (double) ((OpticalWeissFinger*)hand0.finger0_)->hand_width_to_finger_tip_separation(hand_statuses[i].width) << ',';
                dist_files[j] << (double) finger1_samples[i].data[0] << '\n';
            }
        }
        ROS_INFO("Finished writing");
        cur_width += delta_width;
    } while((cur_width < end_width && cur_width > start_width) ||
            (cur_width > end_width && cur_width < start_width));

    // Done, cleanup

    for(unsigned int i = 0; i < devices; i++) {
        dist_files[i].close();
    }
}
