#ifndef LOAD_CELL_FINGER_H
#define LOAD_CELL_FINGER_H

#include <string>

class LoadCellFinger {

public:
    LoadCellFinger(std::string server_name, int port, unsigned int speed);
    ~LoadCellFinger();
    void tare_finger(unsigned int samples);
    double get_measurement();

private:
    double data_to_val(unsigned char* data, int bytes);

    int uart_sock_;
    double tare_val_;

};

#endif // LOAD_CELL_FINGER_H
