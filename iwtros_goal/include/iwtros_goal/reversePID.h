#ifndef _REVERSE_PID_H_
#define _REVERSE_PID_H_

#include <cmath>
#include <iostream>

namespace iwtros{
    class PID{
        public:
            PID(double max, double min, double kp, double ki, double kd);
            ~PID();
            double calculate(double dt, double setPoint, double currentPoint);
        private:
            double max, min, kp, ki, kd;
            double prevError = 0;
            double interError = 0;
    };
};

#endif
