#include "main.h"
#include "subsystems/tankRobot.hpp"
#include <cstdlib>

constexpr int INERTIAL_PORT = 1;
constexpr int VISION_PORT = 2;
constexpr int OPTICAL_PORT = 3;

constexpr char XENCODER_TOP_PORT = 'A';
constexpr char XENCODER_BOTTOM_PORT = 'B';
constexpr char YENCODER_TOP_PORT = 'C';
constexpr char YENCODER_BOTTOM_PORT = 'D';

constexpr TeamColor TEAM_COLOR = Blue;

const PIDConstants drivePID = {0, 0, 0};
const PIDConstants turnPID = {0, 0, 0};
pros::Motor* leftSide;
pros::Motor* rightSide;
pros::Vision vision(VISION_PORT, pros::E_VISION_ZERO_CENTER);
pros::Optical optical(OPTICAL_PORT);
TankDrivetrain* drivetrain;
TankRobot* robot;
pros::Controller driver(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

TeamColor c = TEAM_COLOR;
Odometry odom(XENCODER_TOP_PORT, XENCODER_BOTTOM_PORT, YENCODER_TOP_PORT, YENCODER_BOTTOM_PORT, INERTIAL_PORT, 2);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	leftSide  = (pros::Motor*) malloc(sizeof(pros::Motor) * 3);
	rightSide = (pros::Motor*) malloc(sizeof(pros::Motor) * 3);
	drivetrain = new TankDrivetrain(leftSide, rightSide);
	robot = new TankRobot(*drivetrain, &odom, TEAM_COLOR, &vision, &optical, drivePID, turnPID);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	while(true)
	{
		robot->pollController(false);

		pros::delay(20);
	}

	delete robot;
	delete drivetrain;
}
