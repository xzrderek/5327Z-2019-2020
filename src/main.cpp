#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"
// #include "odom.h"

#define LIFTTHRESHOLD_LOW (-800)
#define LIFTTHRESHOLD_HIGH (-1200)
#define LIFTLOW (-2400)
#define LIFTMED (-3000)
#define TRAYTARGET 1500
#define TRAYRESTING 0
#define TRAYNEUTRAL (-1)

using namespace pros;
using std::string;

Controller master (E_CONTROLLER_MASTER);
Robot rob = Robot();

ADIEncoder encoderL (1, 2, true), encoderR (3, 4, false), encoderM (5, 6, false);
float encM, encL, encR;
int baseLine;
float LeftBAvg, RightBAvg;
volatile int gSlow = 1;
volatile int gAdjustTray = TRAYNEUTRAL;

void updatePIDs(void* param) {
  Robot* r = (Robot*) param;
  const float delayAmnt = 2;
  while(true){
    r->tray.PID();
    r->lift.PID();
    r->intake.PID();
    r->base.PID();

    if(master.btnA) {
      rob.tray.setPIDState(OFF);
      rob.intake.setPIDState(OFF);
      rob.trayToggle.moveToPID(3800);
      rob.lift.moveToPID(700);
    //rob.intake.moveNew(20);
      // rob.lift.moveNew(20);
      // rob.tray.moveNew(20);

      // lcd::print(7, (string("traytoggle kd: ") + std::to_string(r->trayToggle.pid.kD)).c_str());
    }

    if(gAdjustTray != TRAYNEUTRAL) {
      rob.tray.setPIDState(OFF);
      rob.trayToggle.moveToPID(gAdjustTray);
      gAdjustTray = TRAYNEUTRAL;
      // rob.tray.setPIDState(ON);
    }
    //debug
    lcd::print(0, (string("Tray: ") + std::to_string(r->tray.getSensorVal())).c_str());
    lcd::print(1, (string("Lift: ") + std::to_string(r->lift.getSensorVal())).c_str());
    lcd::print(2, (string("Intake: ") + std::to_string(r->intake.getSensorVal())).c_str());
    lcd::print(3, (string("Base: ") + std::to_string(r->base.getSensorVal())).c_str());
    lcd::print(4, (string("Tray Goal: ") + std::to_string(r->tray.getPIDGoal())).c_str());
    lcd::print(5, (string("Lift Goal: ") + std::to_string(r->lift.getPIDGoal())).c_str());
    lcd::print(6, (string("Intake Goal: ") + std::to_string(r->intake.getPIDGoal())).c_str());
    lcd::print(7, (string("gAdjustTray: ") + std::to_string(gAdjustTray)).c_str());
    //lcd::print(7, (string("Base Goal: ") + std::to_string(r->base.getPIDGoal())).c_str());
    //lcd::print(8, (string("toggle goal: ") + std::to_string(r->trayToggle.getPIDGoal())).c_str());


    // delay
    delay(delayAmnt);
  }
}

// void trayToggleFunc(void *param) {
//   Robot* r = (Robot*) param;
//   const float delayAmnt = 2;
//   while(true){
//     if(master.btnA) {
//       rob.tray.setPIDState(OFF);
//       rob.intake.setPIDState(OFF);
//       rob.trayToggle.moveToPID(3800);
//       //rob.intake.moveNew(20);
//       // rob.lift.moveNew(20);
//       // rob.tray.moveNew(20);

//       lcd::print(7, (string("traytoggle kd: ") + std::to_string(r->trayToggle.pid.kD)).c_str());

//       // delay
//       delay(delayAmnt);
//     }
//   }
// }

void updateSensor(void* param) {
  Robot* r = (Robot*) param;
  const float delayAmnt = 20;//ms delay for velocity calculations
  while(true){
    r->base.computeVel();
    delay(delayAmnt);
  }
}

// void updateTask(void* param){
//   Robot* r = (Robot*) param;
//   ADIEncoder encMo (7, 8, false), encRo (5, 6, false), encLo (3, 4, false);
//   ADILineSensor light (2);
//   while(true){
//     LeftBAvg = avg(r->base.mots[1].get_position(), r->base.mots[2].get_position());//1, 2
//     RightBAvg = -avg(r->base.mots[0].get_position(), r->base.mots[3].get_position());//0, 3
//     encM = encMo.get_value();
//     encL = encLo.get_value();
//     encR = -encRo.get_value();
//   	baseLine = light.get_value();
//     if(rob.base.odom.resetEncoders) {
//       encMo.reset();
//   	  encLo.reset();
//   	  encRo.reset();
//     }

//     lcd::print(0, (string("Pos X: ") + std::to_string( rob.base.odom.pos.X)).c_str());
//     lcd::print(1, (string("Pos Y: ") + std::to_string( rob.base.odom.pos.Y)).c_str());
//     lcd::print(2, (string("Theta: ") + std::to_string( rob.base.odom.pos.heading)).c_str());
//     lcd::print(3, (string("LEnc: ") + std::to_string( encL )).c_str());
//     lcd::print(4, (string("REnc: ") + std::to_string( encR )).c_str());
//     lcd::print(5, (string("MEnc : ") + std::to_string( encM )).c_str());
//     lcd::print(6, (string("Line : ") + std::to_string( baseLine )).c_str());

//     delay(2);
//   }
// }

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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
void opcontrol() {
  bool brake = false;

  // Task sensorUpdates(updateSensor, &rob, "");
  // Task taskUpdate(updateTask, &rob, "");
  Task PIDsUpdate(updatePIDs, &rob, "");
  // Task trayToggleTask(trayToggleFunc, &rob, "");
//Task odometryCalculations(calculatePosBASE, &rob.base.odom, "");
  // rob.base.odom.resetEncoders = true;

  bool pressedIntake = false, pressedTray = false, pressedLift = false;
  bool pressedBase = false, pressedSlow = false;

  while (true) {
    // slow
    if (master.btnDOWN) {
      pressedSlow = true;
    } else if (pressedSlow == true) {
      if (gSlow == 1) gSlow = 2;
      else gSlow = 1;
      pressedSlow = false;
    }
    // intake
    rob.intake.simpleControl(master.btnR1, master.btnR2);
    if (master.btnR2 || master.btnR1) {
      rob.intake.setPIDState(OFF);
      pressedIntake = true;
    } else if (pressedIntake) {
      rob.intake.setPIDGoal(rob.intake.getSensorVal());
      rob.intake.setPIDState(ON);
      pressedIntake = false;
    }

    // lift
    rob.lift.simpleControl(master.btnL2, master.btnL1);
    if (master.btnL1 || master.btnL2) {
      rob.lift.setPIDState(OFF);
      pressedLift = true;

      if (master.btnL1 && rob.lift.getSensorVal() < LIFTTHRESHOLD_LOW) {
        gAdjustTray = TRAYTARGET;
      }
      // if (master.btnL2 && rob.lift.getSensorVal() > LIFTTHRESHOLD_HIGH) {
      //   gAdjustTray = TRAYNEUTRAL; // TRAYRESTING;
      // }
    } else if (pressedLift) {
        rob.lift.setPIDGoal(rob.lift.getSensorVal());
        rob.lift.setPIDState(ON);
        pressedLift = false;
    }

    // tray
    rob.tray.simpleControl(master.btnX, master.btnB);
    if (master.btnX || master.btnB) {
      rob.tray.setPIDState(OFF);
      pressedTray = true;
    } else if (pressedTray) {
        rob.tray.setPIDGoal(rob.tray.getSensorVal());
        rob.tray.setPIDState(ON);
        rob.trayToggle.setPIDState(OFF);
        rob.intake.setPIDState(OFF);
        pressedTray = false;
    }

    // base
    rob.base.driveArcade(master.leftY, master.rightX);
    if (master.rightX != 0 || master.leftY != 0) {
      rob.base.setPIDState(OFF);
      pressedBase = true;
    } else if (pressedBase) {
      rob.base.setPIDGoal(rob.base.getSensorVal());
      rob.base.setPIDState(ON);
      pressedBase = false;
    }

    // // if(!brake) rob.base.driveLR(master.rightY, master.leftY);
    // if(master.btnY) {
    //   brake = !brake;
    //   delay(300);
    // }

    // if(master.btnUP){
    //   rob.base.fwdsNEW(24);
    // }

    // // if(master.btnRIGHT){
    // //   rob.base.odom.resetEncoders = true;
    // //   rob.base.odom.resetAngleSentinel = 90;
    // // }

    // if(master.btnDOWN){
    //   rob.base.turnNEW(90);
    // }

    // if(master.btnLEFT){
    //   rob.base.driveToPoint(-10, 10);
    // }

    delay(2);
  }
}
