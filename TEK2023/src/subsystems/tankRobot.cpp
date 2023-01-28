#include "subsystems/tankRobot.hpp"

template<size_t N>
TankRobot<N>::TankRobot(TankDrivetrain<N> d, Odometry* odom, TeamColor tc, pros::Vision* v, pros::Optical* o) : drivetrain{d}, odometry{odom}, color{tc}, vision{v}, optical{o}
{
    driver = pros::Controller(pros::E_CONTROLLER_MASTER);
    partner = pros::Controller(pros::E_CONTROLLER_PARTNER);
}

template<size_t N>
void TankRobot<N>::goTo(Coordinate c, double angle, int timeout)
{
    //Implement
    return;
}

template<size_t N>
void TankRobot<N>::driveTo(Coordinate c, int timeout)
{
    PIDControl.goToTarget(drivetrain, c, odometry, timeout);
}

template<size_t N>
void TankRobot<N>::turnTo(double angle, int timeout)
{
    PIDControl.goToAngle(drivetrain, angle, odometry, timeout);
}

template<size_t N>
void TankRobot<N>::autoAim(bool useVision)
{
    //Implement
    return;
}

template<size_t N>
void TankRobot<N>::pollController(bool dualDriver)
{
    //Implement
    return;
}