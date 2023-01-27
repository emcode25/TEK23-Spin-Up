#ifndef MISC_HPP
#define MISC_HPP

#include "subsystems/odometry.hpp"

struct Coordinate
{
    double x;
    double y;
};

struct PIDConstants
{
    double P;
    double I;
    double D;
};

enum TeamColor
{
    Red,
    Blue
};

class PIDController
{
    public:
        PIDController(PIDConstants pidc, int ms, double inches);
        void setPID(PIDConstants pidc);
        void setTimeout(int ms);
        void setTarget(double inches);
        void goToTarget(void (*action)(int mV), double (*getter)());
        void goToTarget(void (*action)(int mV), Odometry& odom);

    private:
        double target;
        int timeout;
        PIDConstants pid;
};

#endif