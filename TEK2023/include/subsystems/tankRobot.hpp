#ifndef TANKROBOT_H
#define TANKROBOT_H

#include "api.h"
#include "misc.hpp"
#include "odometry.hpp"
#include "drivetrain.hpp"

template <size_t N>
class TankRobot
{
    public:
        TankRobot(TankDrivetrain<N> d, Odometry* odom, TeamColor tc, pros::Vision* v, pros::Optical* o);
        void goTo(Coordinate c, double angle, int timeout);
        void driveTo(Coordinate c, int timeout);
        void turnTo(double angle, int timeout);
        void autoAim(bool useVision);
        void autoShoot(bool useVision);
        void pollController(bool dualDriver);

    private:
        TankDrivetrain<N> drivetrain;
        Odometry* odometry;
        pros::Controller driver;
        pros::Controller partner;
        //Flywheel
        //Intake
        pros::Vision* vision;
        pros::Optical* optical;
        //Indexer
        //Endgame*
        TeamColor color;
        PIDConstants drivePID;
        PIDConstants turnPID;
        PIDController PIDControl;
};

#endif