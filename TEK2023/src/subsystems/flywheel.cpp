#include "subsystems/flywheel.hpp"

constexpr double A = 0;
constexpr double B = 0;
constexpr double C = 0;
constexpr double D = 0;
constexpr double E = 0;

Flywheel::Flywheel(pros::Motor* m, int n, Odometry* odometry, pros::Vision* v, TeamColor tc) : motors{m}, nMotors{n}, odom{odometry}, vision{v}, color{tc}
{
    
}

void Flywheel::setSpeed(int mV)
{
    for (int i = 0; i < nMotors; ++i)
    {
        motors[i].move_voltage(mV);
    }
}

double Flywheel::findGoalSpeed(bool useVision)
{
    return 0;
}