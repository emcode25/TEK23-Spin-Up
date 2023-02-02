#ifndef ENDGAME_HPP
#define ENDGAME_HPP

#include "pros/adi.hpp"

class Endgame
{
    public:
        Endgame(pros::ADIDigitalOut& s);
        void doEndgame();
    private:
        pros::ADIDigitalOut& solenoid;
};

#endif