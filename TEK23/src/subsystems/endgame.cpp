#include "endgame.hpp"

Endgame::Endgame(pros::ADIDigitalOut& s) : solenoid{s}
{

}

void Endgame::doEndgame()
{
    solenoid.set_value(1);
}