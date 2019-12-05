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

void redSmall() {
    rob.base.moveToUntil(12, 1000, 100);
    rob.lift.moveTo(-1200);
    rob.tray.moveTo(600);
    delay(2000);
    //rob.base.moveToUntil(12, 500, 100);
    rob.lift.moveTo(-100);
    delay(500);
    //rob.intake.setPIDState(ON);
    rob.intake.move(0);
    rob.intake.move(-80);
    rob.base.moveToUntil(32, 3300, 80);
    //rob.intake.setPIDState(ON);

    rob.intake.move(0);
  //  rob.base.moveToUntil(-20, 2000, 80);
    // rob.base.moveToUntil(3000, 500, 80);

    // rob.base.turnUntil(90);
    // rob.base.moveToUntil(1000, 4000, 70);
    // //rob.intake.setPIDState(ON);
    // rob.intake.moveToUntil(rob.intake.getSensorVal() - 200, 1000, 80);
    // rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1500, 50);
    // rob.tray.moveToUntil(4000);
    // rob.base.move(80);
    // delay(2000);


    // For reference only
    // rob.intake.move(127);
    // //rob.intake.setPIDState(ON);
    // rob.base.driveToPoint(100, 100);
    // rob.intake.move(0);
    // rob.base.driveToPoint(200, 300);
}

void autonomous() {
    init();
    redSmall();
}
