#ifndef FLYWHEEL_HPP
#define FLYWHEEL_HPP

#include "api.h"
#include "odometry.hpp"

class Flywheel
{
    public:
        const int STANDARD_MV = 12000;

        Flywheel(pros::Motor& m);
        void setSpeed(int mV);
        int findGoalSpeed(double distance);

    private:
        pros::Motor& motor;
        TeamColor color;
};

#endif