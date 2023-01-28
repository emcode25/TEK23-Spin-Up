#include "subsystems/odometry.hpp"
#include "pros/rtos.hpp"
#include <cmath>

Odometry::Odometry(std::uint8_t xTop, std::uint8_t xBottom, std::uint8_t yTop, 
        std::uint8_t yBottom, std::uint8_t imu, double circumference) : 
        xEncoder{pros::ADIEncoder(xTop, xBottom)}, 
        yEncoder{pros::ADIEncoder(yTop, yBottom)}, inertial{pros::Imu(imu)}
{
    position = {0, 0};
    LINEAR_MULTIPLE = M_PI * circumference / 360.0;
}

Coordinate Odometry::getPosition()
{
    return {position.x * LINEAR_MULTIPLE, position.y * LINEAR_MULTIPLE};
}

void Odometry::setPosition(Coordinate c)
{
    position = c;
}

double Odometry::getAngle()
{
    return inertial.get_heading();
}

void Odometry::setAngle(double angle)
{
    inertial.set_heading(angle);
}

void Odometry::reset()
{
    xEncoder.reset();
    yEncoder.reset();
    inertial.reset();
}

void Odometry::update()
{
    double a = getAngle();
    position.x += xEncoder.get_value() * std::cos(a);
    position.y += yEncoder.get_value() * std::sin(a);
    xEncoder.reset();
    yEncoder.reset();
}




PIDController::PIDController(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::setPID(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::goToTarget(void (*action)(int mV), Coordinate target, 
        Odometry &odom, int timeout)
{
    Coordinate pos = odom.getPosition();
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;
    bool useX = true;
    double goal;

    if(std::abs(target.x - pos.x) > std::abs(target.y - pos.y))
    {
        goal = target.x;
    }
    else
    {
        goal = target.y;
        useX = false;
    }

    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        double actual = (useX) ? odom.getPosition().x : odom.getPosition().y;
        double error = std::abs(goal - actual);

        if(error < 0.1)
        {
            break;
        }

        double derivError = error - prevError;
        nextError += error;

        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        if(power < lowerBound)
        {
            power = lowerBound;
        }

        action(power * 110);

        prevError = error;
        pros::delay(10);
    }

    action(0);
}

void PIDController::goToAngle(void (*action)(int), double target, 
        Odometry &odom, int timeout)
{
    double prevError = 0;
    double nextError = 0;
    double lowerBound = 100;

    int timer = pros::millis();
    while(pros::millis() - timer < timeout)
    {
        double error = target - odom.getAngle();

        if(error < 0.1)
        {
            break;
        }

        double derivError = error - prevError;
        nextError += error;

        double power = pid.P * error + pid.I * nextError + pid.D * derivError;

        if(power < lowerBound)
        {
            power = lowerBound;
        }

        action(power * 110);

        prevError = error;
        pros::delay(10);
    }

    action(0);
}