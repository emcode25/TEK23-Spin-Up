#include "subsystems/tankRobot.hpp"

TankRobot::TankRobot(TankDrivetrain d, Odometry* odom, TeamColor tc, pros::Vision* v, pros::Optical* o, PIDConstants drive, PIDConstants turn) : drivetrain{d}, odometry{odom}, color{tc}, vision{v}, optical{o}, driver{pros::Controller(pros::E_CONTROLLER_MASTER)}, partner{pros::Controller(pros::E_CONTROLLER_PARTNER)}, PIDControl{PIDController(drivePID)}, drivePID{drive}, turnPID{turn}
{
    
}

void TankRobot::goTo(Coordinate c, double angle, int timeout)
{
    //Implement
    return;
}

void TankRobot::driveTo(Coordinate c, int timeout)
{
    PIDControl.goToTarget(drivetrain, c, *odometry, timeout);
}

void TankRobot::turnTo(double angle, int timeout)
{
    PIDControl.goToAngle(drivetrain, angle, *odometry, timeout);
}

void TankRobot::autoAim(bool useVision)
{
    //Implement
    return;
}

void TankRobot::pollController(bool dualDriver)
{
    drivetrain.tankControl(driver);

    if(!dualDriver)
    {
        //ENDGAME (X)
        //FLYWHEEL ON(LT)
        //SHOOT (LB)
        //INTAKE (RB)
        //INTAKE  BACK (RT)
    }
}