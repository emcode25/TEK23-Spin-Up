#include "subsystems/indexer.hpp"

#include "pros/rtos.hpp"

Indexer::Indexer(pros::ADIDigitalOut& s) : solenoid{s}
{
    
}

void Indexer::indexDisc()
{
    solenoid.set_value(1);
    pros::delay(400);
    solenoid.set_value(0);
}