#include "misc.hpp"

PIDController::PIDController(PIDConstants pidc, int ms, double inches)
{
    this->pid = pidc;
    this->timeout = ms;
    this->target = inches;
}

void PIDController::setPID(PIDConstants pidc)
{
    this->pid = pidc;
}

void PIDController::setTarget(double inches)
{
    this->target = inches;
}

void PIDController::setTimeout(int ms)
{
    this->timeout = ms;
}