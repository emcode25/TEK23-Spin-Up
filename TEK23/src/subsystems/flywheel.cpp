#include "subsystems/flywheel.hpp"

constexpr double A = 0;
constexpr double B = 0;
constexpr double C = 0;
constexpr double D = 0;
constexpr double E = 0;

Flywheel::Flywheel(pros::Motor& m) : motor{m}
{
    
}

void Flywheel::setSpeed(int mV)
{
    motor.move_voltage(mV);
}

int Flywheel::findGoalSpeed(double distance)
{
    //Implement
    return 0;
}