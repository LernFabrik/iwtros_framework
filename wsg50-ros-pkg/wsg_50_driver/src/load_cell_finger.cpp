#include "wsg_50/load_cell_finger.h"
#include "wsg_50/uart_comms.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define ADC_RATIO 5.0/16777215 // (Volts/Counts)
#define LOAD_CELL_RATIO 10/(128*0.005) // (KG / Voltage Output)
#define COUNTS_TO_FORCE 9.81*ADC_RATIO*LOAD_CELL_RATIO //(Newtons)

LoadCellFinger::LoadCellFinger(std::string server_name, int port, unsigned int speed): uart_sock_(0),
                                                                                       tare_val_(0.0){
    uart_sock_ = client_connect(server_name.c_str(), port, speed);
}

LoadCellFinger::~LoadCellFinger() {
    client_close(uart_sock_);
}

void LoadCellFinger::tare_finger(unsigned int samples) {
    double new_tare_val = 0.0;
    for(unsigned int i = 0; i < samples; i++) {
        new_tare_val += get_measurement();
    }
    tare_val_ = new_tare_val / samples;
}

double LoadCellFinger::get_measurement() {
    unsigned char data[3];
    data[0] = 0;
    int resp_len = client_write(uart_sock_, data, 1);
    resp_len = client_read(uart_sock_, data, 3);

    return data_to_val(data, resp_len) - tare_val_;
}

double LoadCellFinger::data_to_val(unsigned char* data, int bytes) {
    if(bytes != 3) {
        ROS_ERROR("Expected 3 bytes, got %d - aborting", bytes);
        return 0.0;
    }

    int32_t val = 0;
    if(data[0] >= 128) {
        val = 0xFF000000;
    }
    val |= data[0] << 16;
    val |= data[1] << 8;
    val |= data[2];

    return val * COUNTS_TO_FORCE;

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_load_cell_finger");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("load_cell_data", 10);
    LoadCellFinger lcf("/dev/ttyUSB2", 8081, 9600);
    lcf.tare_finger(100);

    ros::Rate rate(10);
    while(ros::ok()) {
        double measurement = lcf.get_measurement();
        std_msgs::Float64 msg;
        msg.data = measurement;
        pub.publish(msg);
        ROS_INFO("%f", measurement);
        rate.sleep();
    }
}
