#include <iostream>
#include "wsg_50/uart_comms.h"
#include "serial/serial.h"

static serial::Serial* uart_;

int client_connect(const char *server_name, int port, unsigned int speed) {
    std::string name(server_name);
    uart_ = new serial::Serial(name, speed);
    uart_->setTimeout(1000, 1000, 4, 1000, 4);
    /*
    uart_->setFlowcontrol(serial::flowcontrol_software);
    uart_->setDTR(false);
    uart_->setRTS(false);
    uart_->set
    */
    if(uart_->getFlowcontrol() == serial::flowcontrol_hardware) {
        std::cout << "flow control is hw" << std::endl;
    } else if(uart_->getFlowcontrol() == serial::flowcontrol_none) {
        std::cout << "flow control is none" << std::endl;
    } else if(uart_->getFlowcontrol() == serial::flowcontrol_software) {
        std::cout << "flow control is sw" << std::endl;
    } else {
        std::cout << "Unknown flow control" << std::endl;
    }
    uart_->flush();
    return 1;
}

int client_read(int sockfd, unsigned char* buf, int bytes) {

    std::vector<uint8_t> data;
    uart_->read(data, bytes);
    uart_->flushInput();
    for(unsigned int i = 0; i < data.size(); i++) {
        buf[i] = data[i];
    }
    return data.size();
}

int client_write(int sockfd, unsigned char* buf, int bytes) {
    std::vector<uint8_t> data;
    for(unsigned int i = 0; i < bytes; i++) {
        data.push_back(buf[i]);
    }
    return uart_->write(data);
}

int client_close(int sockfd) {
    if(uart_) {
        delete uart_;
    }
    return 1;
}
