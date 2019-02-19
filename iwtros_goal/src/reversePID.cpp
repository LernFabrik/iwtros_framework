#include <iwtros_goal/reversePID.h>
#include <ros/console.h>

namespace iwtros{
    PID::PID(double max, double min, double kp, double ki, double kd){
        this->max = max;
        this->min = min;
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->interError = 0;
        this->prevError = 0;
    }

    PID::~PID(){}

    double PID::calculate(double dt, double setPoint, double currentPoint){
        /*this will generate the command signal in negetive x axis
        because FTS has to move in revese*/
        double error = setPoint -currentPoint;
        double Pout = error * this->kp;
        this->interError += error * dt;
        double Iout = this->interError * this->ki;
        double diffError = error - this->prevError;
        double Dout = diffError * this->kd;
        double output = Pout + Iout + Dout;
        if(output > this->max) output = this->max;
        else if(output < this->min) output = this->min;
        this->prevError = error;
        return output;
    }
};