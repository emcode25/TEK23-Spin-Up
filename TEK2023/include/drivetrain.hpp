#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include <array>
#include "api.h"
#include "misc.hpp"

class Drivetrain
{
    public:
        virtual void drive(int mV);
        virtual void turnLeft(int mV);
};

template <size_t N>
class TankDrivetrain : Drivetrain
{
    public:
        TankDrivetrain();
        
        void drive(int mV);
        void turnLeft(int mV);
        void driveForward(double inches, int timeout);
        void turnTo(double inches, int timeout);
        std::array<pros::Motor, N> getLeft();
        std::array<pros::Motor, N> getRight();

    private:
        std::array<pros::Motor, N> left;
        std::array<pros::Motor, N> right;
        PIDConstants drivePID;
        PIDConstants turnPID;
        
};

#endif