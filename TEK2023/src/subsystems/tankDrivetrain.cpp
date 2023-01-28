#include "subsystems/drivetrain.hpp"

#include <cmath>
#include <cstdarg>

template<size_t N>
TankDrivetrain<N>::TankDrivetrain(std::array<pros::Motor, N>& l, 
        std::array<pros::Motor, N>& r, PIDConstants drivePID, PIDConstants turnPID)
{
    left = r;
    right = r;
    this->drivePID = drivePID;
    this->turnPID  = turnPID;
}

template<size_t N>
void TankDrivetrain<N>::drive(int mV)
{
    for(int i = 0; i < N; ++i)
    {
        left.at(i).move_voltage(mV);
        right.at(i).move_voltage(mV);
    }
}

template<size_t N>
void TankDrivetrain<N>::turnLeft(int mV)
{
    for(int i = 0; i < N; ++i)
    {
        left.at(i).move_voltage(-mV);
        right.at(i).move_voltage(mV);
    }
}

template<size_t N>
std::array<pros::Motor, N> TankDrivetrain<N>::getLeft()
{
    return left;
}

template<size_t N>
std::array<pros::Motor, N> TankDrivetrain<N>::getRight()
{
    return right;
}