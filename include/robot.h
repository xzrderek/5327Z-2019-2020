#ifndef ROBOT_H
#define ROBOT_H
#include "api.h"
#include "util.h"
#include <string>
#include <vector>
#include <cerrno>
#include "lowLevel.h"

using pros::Motor, pros::ADIEncoder, std::string;
using namespace pros;

//defining motor ports:
#define RFront   12
#define RBack    13
#define LFront   4
#define LBack    2
#define LIFT     19
#define INTAKE1  9
#define INTAKE2  21
#define TRAY     6

#define BACK true

// extern ADIPotentiometer pot;
class Robot{
public:
	//CONSTRUCTOR:
	Robot() :
	//mechanisms	
	tray(
		{ Motor(TRAY) }, //motors
		{},//no sensors for indexer, thus use indexer motor
		PIDcontroller(2.0, 0.0, 10.0, 10,  10, true, true)//PID
	),
	intake(
		{ Motor(INTAKE1), Motor(INTAKE2, true) }, //motors
		{},//no sensors for intake, thus use indexer motor
		PIDcontroller(1.0, 0.0, 2.0, 10,  10, true, true)//PID
	),
	lift(
		{ Motor(LIFT)}, //motors
		{},//no sensors for lift, thus use indexer motor
		PIDcontroller(2.0, 0.0, 4.0, 10,  10, true, true)//PID
	),
	base(
		{ Motor(RFront, true), Motor(LFront, true), Motor(LBack), Motor(RBack) }, //motors
		{},//no sensors for intake, thus use indexer motor
		PIDcontroller(2.0, 0.0, 1.0, 10,  10, true, true)//PID
	),
	trayToggle(
		{ Motor(TRAY) }, //motors
		{},//no sensors for indexer, thus use indexer motor
		PIDcontroller(2.0, 0.0, 10.0, 10,  10, true, true)//PID
	)
	// base(//motors
	// 	{ Motor(RFront), Motor(LFront), Motor(LBack), Motor(RBack) },
	// 	//drive PID, then angle PID, then curve
	// 	{ PIDcontroller(12, 0.0, 0.05, 1.75, 10, true, false),
	// 	  PIDcontroller(3.5, 0.0, 0.5, 2.0,  10, false, false),
	// 	  PIDcontroller(2.5, 0.0, 0.0, 1.0,  10, false, false) },
	// 	//Odometry
	// 	Odometry(Position(0, 0, 0), Position(0, 0, 0)//,//actual position, tracker mech's position
	// 	//Odom Sensors:
	// 	)
	// )
	{
	// base.odom.pos.X = 0;
	// base.odom.pos.Y = 0;
	// base.odom.pos.heading = 90;
	}

	Mechanism tray;
	Mechanism intake;
	Mechanism lift;
	Mechanism trayToggle;
	Chassis base; //Chassis base;
	float FWVelGoal = 0;

	// void indexerAdvance(int amntTicks = 100){//bring indexer ball up once (given number of encoder ticks)
	// 	indexer.moveAmnt(amntTicks, 10);
	// }

	//tests

	// std::vector<string> debugString(){
		// std::vector<string> ret;
		// //ret.push_back(string("BATTERY percent:") + std::to_string( pros::battery::get_capacity()));
		// /*ret.push_back(string("Flywheel1 Vel:") + std::to_string(sprocket1.get_actual_velocity()));
		// ret.push_back(string("Flywheel1 Temp:") + std::to_string(sprocket1.get_temperature()));
		// ret.push_back(string("Flywheel2 Vel:") + std::to_string(sprocket2.get_actual_velocity()));
		// ret.push_back(string("Flywheel2 Temp:") + std::to_string(sprocket2.get_temperature()));
		// if(flyWheelVelPID.getRunningState()) ret.push_back(string("PID Running: YES"));
		// else ret.push_back(string("PID Running: NO"));
		// ret.pus h_back(string("PID Goal:") + std::to_string(flyWheelVelPID.getGoal()));
		// */

		// //ret.push_back(string("EncL: ") + std::to_string( encoderL.get_value())
		// //+string("; EncR: ") + std::to_string( encoderR.get_value())
		// //+string("; EncM: ") + std::to_string( encoderM.get_value()));
		// ret.push_back(string("Pos X: ") + std::to_string( base.odom.pos.X) + string(" Pos Y: ") + std::to_string( base.odom.pos.Y));
		// ret.push_back(string("Heading: ") + std::to_string( base.odom.pos.heading));
		// //ret.push_back(string("FlywheelPos: ") + std::to_string( flywheelEnc.get_value()));
		// //ret.push_back(string("FlywheelVel(rpm): ") + std::to_string( flywheel.velocity/2) + string(" Motors: ") + std::to_string( flywheel.getMotorVel() ));
		// ret.push_back(string("base Vel: ") + std::to_string( base.driveVel) + string("; rot Vel: ") + std::to_string( base.rotVel));
		// ret.push_back(string("Angle Err: ") + std::to_string( base.pid[ANGLE].error));

		// //ret.push_back(string("Error Val: ") + strerror(errno));
		// return ret;
	// }
};
#endif