#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "wsg_50/hand.h"
#include "wsg_50/optical_weiss_finger.h"
#include "wsg_50_common/WeissFingerData.h"

/************************************************************************

ASSUMES THAT A CUSTOM FINGER SENSOR IS ON FINGER 0 AND AN FMF FINGER IS ON FINGER 1

Produces a csv file for each device on custom finger sensor.  Each file is of the form:

  Finger 0 Data, True Distance, Finger 1 Data

************************************************************************/

// The amount to incrementally move the gripper in millimeters
const static double FORCE_DIST_DELTA = -0.1;

/**
  Convert the passed value to a string. Prepend zeros that the string has a lenght
  equal to places
  val: The value to convert to a string
  places: The number of characters in the resulting 
**/
std::string prepend_zeros(int val, int places) {
    std::stringstream ss;
    ss << val;
    std::string result(ss.str());
    int zeros = places - (((int)(std::floor(std::log10(val)))+1));

    assert(zeros >= 0); // Throw error if places was too smal

    for(unsigned int i = 0; i < zeros; i++) {
        result = "0"+result;
    }
    return result;

}

/** 
  Compute the mean and standard deviation of the applied force
  samples: The number of samples to collect
  Hand: Interface to collect measured data
  mean: The mean of the measured force
  std_dev: The standard deviation of the measured force
**/
void get_force_mean_and_std_dev(unsigned int samples, Hand& hand, double* mean, double* std_dev) {
    *mean=0.0;
    double square_sum = 0.0;
    unsigned int sample_count = 0;
    ros::Rate sample_rate(25);
    while(sample_count < samples) {

        wsg_50_common::WeissFingerData wfd1 = hand.get_finger_sample(1);
        *mean += wfd1.data[0];
        square_sum += wfd1.data[0]*wfd1.data[0];

        sample_count += 1;
        sample_rate.sleep();
    }

    *std_dev = std::sqrt((square_sum-(*mean)*(*mean)/sample_count)/(sample_count-1));
    *mean = (*mean)/sample_count;

}

/**
  Retrives data from both fingers on the hand.
  samples: The number of samples to collect
  Hand: Interface to collect data
  hand_statuses: Hand state data
  finger0_samples: Data from finger 0
  finger1_samples: Data from finger 1
**/
double get_data(unsigned int samples, Hand& hand,
                std::vector<wsg_50_common::Status>& hand_statuses,
                std::vector<wsg_50_common::WeissFingerData>& finger0_samples,
                std::vector<wsg_50_common::WeissFingerData>& finger1_samples) {

    double mean_force = 0.0;

    // Collect data
    ros::Duration(0.1).sleep();
    ros::Rate sample_rate(25);

    while(finger0_samples.size() < samples) {
        wsg_50_common::WeissFingerData wfd0 = hand.get_finger_sample(0);
        wsg_50_common::WeissFingerData wfd1 = hand.get_finger_sample(1);
        wsg_50_common::Status hs0 = hand.get_hand_state();
        
        // Make sure that this is either the beginning of finger 0 data or the time stamp
        // has changed since we last collected finger 0 data
        if(finger0_samples.size() == 0 || (wfd0.stamp - finger0_samples[finger0_samples.size()-1].stamp).toSec() > 0.0075) {
            // Print out the data from the last sensor
            std::cout << (int) wfd0.data[(wfd0.data_shape[0]-1)*wfd0.data_shape[1]+0] << ' '
                      << (int) wfd0.data[(wfd0.data_shape[0]-1)*wfd0.data_shape[1]+1] << ' '
                      << (int) wfd0.data[(wfd0.data_shape[0]-1)*wfd0.data_shape[1]+2] << ' '
                      << (int) wfd0.data[(wfd0.data_shape[0]-1)*wfd0.data_shape[1]+3] << ' '
                      << (int) wfd0.data[(wfd0.data_shape[0]-1)*wfd0.data_shape[1]+4] << ' '
                      <<       hs0.width                         << ' '
                      <<       wfd1.data[0]                      <<'\n';
            finger0_samples.push_back(wfd0);
            finger1_samples.push_back(wfd1);
            hand_statuses.push_back(hs0);
            mean_force += wfd1.data[0];
        }
        sample_rate.sleep();
    }

    return mean_force / finger1_samples.size();
}

/**
  Write collected data to disk
  files: File descriptors to write to. Length should be equal to the number of devices
         that data has been collected from
  hand_statuses: Hand data to write
  finger0_samples: Data from finger 0 to write
  finger1_samples: Data from finger 1 to write
**/
void write_data(std::vector<std::ofstream>& files,
                Hand& hand,
                const std::vector<wsg_50_common::Status>& hand_statuses,
                const std::vector<wsg_50_common::WeissFingerData>& finger0_samples,
                const std::vector<wsg_50_common::WeissFingerData>& finger1_samples) {
    // Write data
    // Loop through each sample
    for(unsigned int i = 0; i < finger0_samples.size(); i++) {
        // Loop through each device
        for(unsigned int j = 0; j < finger0_samples[i].data_shape[0]; j++) {
            // Loop through each data output
            for(unsigned int k = 0; k < finger0_samples[i].data_shape[1]-1; k++) {
                files[j] << (double) finger0_samples[i].data[j*finger0_samples[i].data_shape[1]+k] << ',';
            }
            files[j] << (double) finger0_samples[i].data[(j+1)*finger0_samples[i].data_shape[1]-1] << ',';
            files[j] << (double) ((OpticalWeissFinger*)hand.finger0_)->hand_width_to_finger_tip_separation(hand_statuses[i].width) << ',';
            files[j] << (double) finger1_samples[i].data[0] << '\n';
        }
    }
    ROS_INFO("Finished writing");
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "collect_dist_force_node");
    ros::NodeHandle nh;

    // Read in params
    std::string data_path("");
    if(!nh.getParam("/collect_dist_force/data_path", data_path)) {
        ROS_ERROR("Did not receive data path, abort");
        return 1;
    }

    int devices = 0;
    if(!nh.getParam("/collect_dist_force/devices", devices) || devices <= 0) {
        ROS_ERROR("Did not receive valid number of devices, abort");
        return 1;
    }

    double start_width = -1.0;
    if(!nh.getParam("/collect_dist_force/start_width", start_width) || start_width < 0.0) {
        ROS_ERROR("Did not receive valid start width, abort");
        return 1;
    }

    double dist_delta_width = 0.0;
    if(!nh.getParam("/collect_dist_force/dist_delta_width", dist_delta_width)) {
        ROS_ERROR("Did not receive valid dist delta width, abort");
        return 1;

    }
    if(dist_delta_width > 0.0) {
        ROS_INFO("Flipping dist delta width so that gripper decreases in width");
        dist_delta_width = -1*dist_delta_width;
    }

    double force_delta = 0.0;
    if(!nh.getParam("/collect_dist_force/force_delta", force_delta)) {
        ROS_ERROR("Did not receive force delta, abort");
        return 1;
    }
    if(force_delta < 0.0) {
        ROS_INFO("Flipping force delta so that gripper force increases with time");
    }

    double end_force = 0.0;
    if(!nh.getParam("/collect_dist_force/end_force", end_force) || end_force <= 0.0) {
        ROS_ERROR("Did not receive valid end force, abort");
        return 1;
    }

    int samples_per_step = 0;
    if(!nh.getParam("/collect_dist_force/samples_per_step", samples_per_step) || samples_per_step <= 0) {
        ROS_ERROR("Did not receive valid samples per step, abort");
        return 1;
    }

    bool load_finger0_calibration;
    if(!nh.getParam("/collect_dist_force/load_finger0_calibration", load_finger0_calibration)) {
        ROS_ERROR("Did not receive load finger 0 calibration flag, abort");
        return 1;
    }

    bool load_finger1_calibration;
    if(!nh.getParam("/collect_dist_force/load_finger1_calibration", load_finger1_calibration)) {
        ROS_ERROR("Did not receive load finger 1 calibration flag, abort");
        return 1;
    }

    // Load calibration for the fingers
    // Will calibrate force sensor on the fly below
    Hand hand0(nh, "hand0");
    if(load_finger0_calibration) {
        hand0.load_calibration(0);
    }
    
    /*
    if(load_finger1_calibration) {
        hand0.load_calibration(1);
    }
    */
    
    hand0.start_reading();

    // Check that the contact is not already being made
    if(((OpticalWeissFinger*)hand0.finger0_)->hand_width_to_finger_tip_separation(start_width) <= 0.0) {
        ROS_ERROR("Starting width would cause initial force (need larger starting width), abort");
        return 1;
    }

    std::vector<std::ofstream> dist_files(devices);
    for(unsigned int i = 0; i < devices; i++) {
        std::string str_idx(prepend_zeros(i+1, 2));
        dist_files[i].open(data_path+"/dist"+str_idx+".csv");
    }

    std::vector<std::ofstream> force_files(devices);
    for(unsigned int i = 0; i < devices; i++) {
        std::string str_idx(prepend_zeros(i+1,2));
        force_files[i].open(data_path+"/force"+str_idx+".csv");

    }

    // Calibrate the force sensor
    double no_force_mean;
    double no_force_std_dev;
    hand0.move_hand(start_width);
    ros::Duration(0.5).sleep();
    ROS_INFO("Calibrating fmf sensor");
    hand0.do_calibration(1);

    ros::Duration(1.0).sleep();

    ROS_INFO("Getting no force mean and std dev...");
    get_force_mean_and_std_dev(samples_per_step, hand0, &no_force_mean, &no_force_std_dev);
    ROS_INFO("No force mean: %f, std dev: %f", no_force_mean, no_force_std_dev);

    // Close the hand until contact is detected
    double cur_force_mean;
    double cur_force_std_dev;
    double cur_width = start_width;
    while(ros::ok()) {
        ROS_INFO("Going to move hand");
        // Tell hand to move
        hand0.move_hand(cur_width);
        ROS_INFO("Done moving hand");
        ros::Duration(0.5).sleep();

        std::vector<wsg_50_common::Status> hand_statuses;
        std::vector<wsg_50_common::WeissFingerData> finger0_samples;
        std::vector<wsg_50_common::WeissFingerData> finger1_samples;
        cur_force_mean = get_data(samples_per_step, hand0, hand_statuses, finger0_samples, finger1_samples);
        if(cur_force_mean > no_force_mean + 1.5*no_force_std_dev) {
            break;
        } else {
            ROS_INFO("Writing distance data...");
            write_data(dist_files, hand0, hand_statuses, finger0_samples, finger1_samples);
        }
        cur_width += dist_delta_width;
    }

    ROS_INFO("Made contact, beginning force servoing");
    ros::Duration(1.0).sleep();

    cur_width -= dist_delta_width;
    hand0.move_hand(cur_width);
    ros::Duration(0.5).sleep();
    get_force_mean_and_std_dev(samples_per_step, hand0, &cur_force_mean, &cur_force_std_dev);

    double force_target = 0.0;
    double prev_force_mean = 1000000.0;

    // Collect force data
    while(ros::ok() && force_target <= end_force) {
        // Clost the hand until no longer approaching force target
        while(std::abs(cur_force_mean-force_target) < std::abs(prev_force_mean-force_target) || (cur_force_mean < no_force_mean + 1.5*no_force_std_dev)) {
            cur_width += FORCE_DIST_DELTA;
            hand0.move_hand(cur_width);
            ros::Duration(0.5).sleep();

            prev_force_mean = cur_force_mean;
            get_force_mean_and_std_dev(samples_per_step, hand0, &cur_force_mean, &cur_force_std_dev);
            ROS_INFO("Dist: %f, prev_force: %f, cur_force: %f", cur_width, prev_force_mean, cur_force_mean);
        }

        // Move back to previous position
        cur_width -= FORCE_DIST_DELTA;
        hand0.move_hand(cur_width);
        ros::Duration(0.5).sleep();

        // Collect force data at this position
        std::vector<wsg_50_common::Status> hand_statuses;
        std::vector<wsg_50_common::WeissFingerData> finger0_samples;
        std::vector<wsg_50_common::WeissFingerData> finger1_samples;
        get_data(samples_per_step, hand0, hand_statuses, finger0_samples, finger1_samples);
        write_data(force_files, hand0, hand_statuses, finger0_samples, finger1_samples);

        // Move back to position where we left off
        cur_width += FORCE_DIST_DELTA;
        hand0.move_hand(cur_width);
        ros::Duration(0.5).sleep();

        // Update the force target
        ROS_INFO("About to change force target");
        while(std::abs(cur_force_mean-force_target) > std::abs(prev_force_mean-force_target)) {
            force_target += force_delta;
        }
        ROS_INFO("New force target: %f",force_target);

    }

    for(unsigned int i = 0; i < devices; i++) {
        dist_files[i].close();
    }

    for(unsigned int i = 0; i < devices; i++) {
        force_files[i].close();
    }

}
