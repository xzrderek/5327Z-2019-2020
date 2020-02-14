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
    rob.lift.moveTo(-1400);
    rob.tray.moveTo(600);
    delay(1200);
    rob.lift.moveTo(-100);
    delay(300);
    rob.intake.move(0);
    rob.intake.move(-90);
    rob.base.moveToUntil(34, 2600, 80);
    rob.intake.move(0);
    // rob.base.moveToUntil(-20, 2000, 80);
    rob.base.moveToUntil(-24, 1200, 100);
    // rob.base.moveToUntil(-5, 1000, 80);
    // rob.base.moveToUntil(0, 50);
    rob.base.turnUntil(130, 1300, 110);
    rob.base.moveToUntil(18, 1700, 80);

    // rob.base.moveToUntil(1000, 4000, 70);
    rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 1000, 70);
    rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1200, 40);
    rob.tray.moveToUntil(4000);
    rob.base.moveToUntil(-15, 1700, 100);
    // 14800ms up to here
    delay(2000);
  }


void turn() {
    rob.base.pointTurn(80);
}
    // For reference only
    // rob.intake.move(127);
    // //rob.intake.setPIDState(ON);
    // rob.base.driveToPoint(100, 100);
    // rob.intake.move(0);
    // rob.base.driveToPoint(200, 300);

void bigAuton() {
  rob.base.moveToUntil(-10, 500, 100);
  rob.base.moveToUntil(15, 1000, 100);
}

void redSmall() {
  smallAuton(1);
}

void blueSmall() {
  smallAuton(-1);
}


void smallAutonOld(int color=1) {
    rob.intake.setPIDState(OFF);
    rob.base.moveToUntil(12, 1000, 100);
    rob.lift.moveTo(-1400);
    rob.tray.moveTo(600);
    delay(1200);
    rob.lift.moveTo(-100);
    delay(300);
    rob.intake.move(0);
    rob.intake.move(-90);
    rob.base.moveToUntil(34, 2600, 80);
    rob.intake.move(0);
    // rob.base.moveToUntil(-20, 2000, 80);
    rob.base.moveToUntil(-24, 1200, 100);
    // rob.base.moveToUntil(-5, 1000, 80);
    // rob.base.moveToUntil(0, 50);
    //rob.base.turnUntil(color * 130, 1300, 110);
    rob.base.moveToUntil(18, 1700, 80);

    rob.base.pointTurn(50);
    delay(1000);
    rob.base.pointTurn(0);

    // rob.base.moveToUntil(1000, 4000, 70);
    rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 1000, 70);
    rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1200, 40);
    rob.tray.moveToUntil(4000);
    rob.base.moveToUntil(-15, 1700, 100);
    // 14800ms up to here
    delay(2000);
}


void Vanden() {
    rob.intake.setPIDState(OFF);
    rob.base.moveToUntil(12, 1000, 100);
    rob.lift.moveTo(-1800);
    rob.tray.moveTo(600);
    delay(1200);
    rob.lift.moveTo(-100);
    delay(300);
    rob.intake.move(0);
    rob.intake.move(-90);
    rob.base.moveToUntil(34, 2600, 80);
    rob.intake.move(0);
    // rob.base.moveToUntil(-20, 2000, 80);
    rob.base.moveToUntil(-24, 1200, 100);
    rob.base.moveToUntil(0, 100, 0);
    // rob.base.moveToUntil(-5, 1000, 80);
    // rob.base.moveToUntil(0, 50);
    //rob.base.turnUntil(color * 130, 1300, 110);
    rob.base.pointTurn(80);
    delay(1200);
    rob.base.pointTurn(0);
    rob.lift.moveTo(-100);
    rob.tray.moveTo(500);
    rob.base.moveToUntil(18, 1700, 80);



    //rob.base.moveToUntil(1000, 4000, 70);
    //rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 1000, 70);
    //rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1200, 40);
    rob.tray.moveToUntil(4000);
    rob.base.moveToUntil(5, 500, 50);
    rob.base.moveToUntil(-15, 1700, 100);
    // 14800ms up to here
    delay(2000);
}


void CV1(int color=1) {
  rob.intake.setPIDState(OFF);

    rob.tray.moveTo(800);
    //task: movetountil(target, 10 seconds)
    //delay(500);
    rob.tray.moveTo(0);
    delay(600);
    rob.lift.moveTo(-100);
    delay(300);
    rob.intake.move(0);
    rob.intake.move(-120);
    rob.base.moveToUntil(34, 2600, 80);
    // rob.intake.move(0);
    // // rob.base.moveToUntil(-20, 2000, 80);
    // rob.base.moveToUntil(-24, 1200, 100);
    // // rob.base.moveToUntil(-5, 1000, 80);
    // // rob.base.moveToUntil(0, 50);
    // //rob.base.turnUntil(color * 130, 1300, 110);
    // rob.base.moveToUntil(18, 1700, 80);
    //
    // rob.base.pointTurn(50);
    // delay(1000);
    // rob.base.pointTurn(0);
    //
    // // rob.base.moveToUntil(1000, 4000, 70);
    // rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 1000, 70);
    // rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 1200, 40);
    // rob.tray.moveToUntil(4000);
    // rob.base.moveToUntil(-15, 1700, 100);
    // // 14800ms up to here
    // delay(2000);
}

void test() {
  // rob.base.setPID(ANGLE, 6, 0, 4); //30 degrees
  // rob.base.turnUntil(30, 10000, 70);
  //  rob.base.moveToUntil(12, 10000, 100);
  // rob.base.setPID(ANGLE, 1.8, 0, 4);
  // rob.base.turnUntil(90, 10000, 80);
  //rob.tray.setPIDState(ON);
  // rob.tray.setPIDState(ON);
  // // rob.intake.move(-63);
  // rob.tray.setPIDState(ON);
  // rob.intake.move(63);
  // rob.tray.moveToUntil(4000);
  // rob.base.moveToUntil(-15, 1700, 100);
  rob.intake.move(-127);

}

void CV2(int color=1) {
    rob.intake.setPIDState(OFF);
    rob.tray.setPIDState(ON);
    rob.lift.setPIDState(ON);
    //rob.intake.setPIDState(OFF);
    rob.base.moveToUntil(12, 1000, 100);
    //rob.lift.moveTo(-1400);
    rob.tray.moveToUntil(1000, 1200, 127);
    rob.tray.setPIDState(OFF);
    rob.lift.moveTo(100);
    delay(1000);
    rob.intake.setSlow(SLOW_NORMAL);
    rob.intake.move(0);
    rob.intake.move(-127);
    rob.base.moveToUntil(34, 3000, 80);
    //rob.intake.move(0);
    // rob.base.moveToUntil(-20, 2000, 80);
    rob.base.moveToUntil(-24, 1200, 100);
    // rob.base.moveToUntil(-5, 1000, 80);
    // rob.base.moveToUntil(0, 50);
    //rob.intake.move(0);
    rob.base.turnUntil(color * 120, 1300, 110);
    rob.base.moveToUntil(15, 1700, 100);

    rob.intake.move(0);

    // rob.base.moveToUntil(1000, 4000, 70);
    //rob.intake.moveToUntil(rob.intake.getSensorVal() - 800, 1000, 70);
    // rob.intake.moveToUntil(rob.intake.getSensorVal() + 600, 500, 200);
    // rob.intake.move(0);
    rob.intake.move(50);

    rob.tray.moveToUntil(4000, 2500, 80);
    rob.base.moveToUntil(-15, 1700, 100);
    //rob.intake.move(-63);
    // 14800ms up to here
    delay(2000);
  }

void autonomous() {
    init();
    // test();
    // bigAuton();
    CV2(-1); //blue
    // CV2(1); //red
}
