#include "main.h"
#include "lowLevel.h"
#include "api.h"
#include "robot.h"
#include "odom.h"

using namespace pros;
using std::string;

extern Controller master;
extern Robot rob;

void resetEncoders();
void updatePIDs(void* param);
void updateTask(void* param);

void init()  {
    rob.reset();
    resetEncoders();

    rob.base.odom.resetEncoders = true;

    // Task sensorUpdates(updateSensor, &rob, "");
    Task taskUpdate(updateTask, &rob, "");
    Task PIDsUpdate(updatePIDs, &rob, "");
    // Task trayToggleTask(trayToggleFunc, &rob, "");
    Task odometryCalculations(calculatePosBASE, &rob.base.odom, "");
}

/*
1=RED
-1=BLUE
*/
void smallAuton(int color=1) {
    rob.intake.setPIDState(OFF);
    rob.base.moveToUntil(12, 1000, 100);
    rob.lift.moveTo(-1200);
    rob.tray.moveTo(600);
    delay(1200);
    rob.lift.moveTo(-100);
    delay(300);
    rob.intake.move(0);
    rob.intake.move(-90);
    rob.base.moveToUntil(34, 2700, 100);
    rob.intake.move(0);
    // rob.base.moveToUntil(-20, 2000, 80);
    rob.base.moveToUntil(-25, 1300, 100);
    // rob.base.moveToUntil(-5, 1000, 80);
    // rob.base.moveToUntil(0, 50);
    rob.base.turnUntil(color * 130, 1300, 110);
    rob.base.moveToUntil(16, 1500, 80);

    // rob.base.moveToUntil(1000, 4000, 70);
    rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 800, 80);
    rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1000, 50);
    rob.tray.moveToUntil(4000);
    rob.base.moveToUntil(-15, 1700, 100);
    // 14800ms up to here
    delay(2000);


    // For reference only
    // rob.intake.move(127);
    // //rob.intake.setPIDState(ON);
    // rob.base.driveToPoint(100, 100);
    // rob.intake.move(0);
    // rob.base.driveToPoint(200, 300);
}

void redSmall() {
  smallAuton(1);
}

void blueSmall() {
  smallAuton(-1);
}

void autonomous() {
    init();
    redSmall();
}
