#include "subsystems/drivetrain.hpp"

#include <cmath>
#include <cstdarg>

TankDrivetrain::TankDrivetrain(pros::Motor* l, pros::Motor* r)
{
    left = l;
    right = r;
}

void TankDrivetrain::drive(int mV)
{
    for(int i = 0; i < 3; ++i)
    {
        left[i].move_voltage(mV);
        right[i].move_voltage(mV);
    }
}

void TankDrivetrain::turnLeft(int mV)
{
    for(int i = 0; i < 3; ++i)
    {
        left[i].move_voltage(-mV);
        right[i].move_voltage(mV);
    }
}

pros::Motor* TankDrivetrain::getLeft()
{
    return left;
}

pros::Motor* TankDrivetrain::getRight()
{
    return right;
}

void TankDrivetrain::tankControl(pros::Controller& c)
{
    int l = (c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 120 / 127) * 100;
    int r = (c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 120 / 127) * 100;
    for(int i = 0; i < 3; ++i)
    {
        left[i].move_voltage(l);
        right[i].move_voltage(r);
    }
}