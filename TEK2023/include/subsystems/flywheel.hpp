#ifndef FLYWHEEL_HPP
#define FLYWHEEL_HPP

#include "api.h"
#include "odometry.hpp"

class Flywheel
{
    public:
        Flywheel(pros::Motor* m, int n, Odometry* odometry, pros::Vision* v, TeamColor tc);
        void setSpeed(int mV);
        double findGoalSpeed(bool useVision);

    private:
        pros::Motor* motors;
        int nMotors;
        TeamColor color;
        pros::Vision* vision;
        Odometry* odom;
};

#endif