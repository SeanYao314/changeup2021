#include "robot.hpp"
#include "recording.h"
#include "screen.h"

using namespace okapi;
using namespace std;
using namespace recording;

pros::Imu imu_sensor(3);
void skill_auton();
void delay(int x) {
    pros::delay(x);
}
void toplock() {
    top_intake.setBrakeMode(AbstractMotor::brakeMode::hold);
}
void topunlock() {
	top_intake.setBrakeMode(AbstractMotor::brakeMode::coast);
}
void turningP(int targetTurn, int voltageMax=127, bool debugLog=false) {
	float kp = 2.4;
	float ki = 0.8;
	float kd = 0.45;

	// the untouchables
	int voltageCap = 0;
	float voltage = 0;
	float errorCurrent;
	float errorLast;
	int errorCurrentInt;
	int errorLastInt;
	int sameErrCycles = 0;
	int same0ErrCycles = 0;
	int p;
	float i;
	int d;
	int sign;
	float error;
	int startTime = pros::millis();

	while(autonomous) {
		error = targetTurn - imu.get_heading();
		errorCurrent = abs(error);
		errorCurrentInt = errorCurrent;
		sign = targetTurn / abs(targetTurn); // -1 or 1

		p = (error * kp);
		if (abs(error) < 10) { // if we are in range for I to be desireable
			i = ((i + error) * ki);
		}
		else
			i = 0;
		d = (error - errorLast) * kd;
		
		voltage = p + i + d;

		if(abs(voltage) > voltageMax) voltage = voltageMax * sign;

		// set the motors to the intended speed
		chassis->getModel()->tank(-voltage/127, voltage/127);

		// timeout utility
		if (errorLastInt == errorCurrentInt) {
			if (errorLast <= 2 and errorCurrent <= 2) { // saying that error less than 2 counts as 0
				same0ErrCycles +=1;
			}
			sameErrCycles += 1;
		}
		else {
			sameErrCycles = 0;
			same0ErrCycles = 0;
		}

		// exit paramaters
		if (same0ErrCycles >= 5 or sameErrCycles >= 60) { // allowing for smol error or exit if we stay the same err for .6 second
			chassis->stop();
			std::cout << pros::millis() << "task complete with error " << errorCurrent << " in " << (pros::millis() - startTime) << "ms" << std::endl;
			return;
		}
		
		// debug
		std::cout << pros::millis() << "error " << errorCurrent << std::endl;
		std::cout << pros::millis() << "voltage " << voltage << std::endl;
        	std::cout << pros::millis() << "theta " << imu.get_rotation() << std::endl;
        
		// for csv output, graphing the function
		// if (debugLog) std::cout << pros::millis() << "," << error << "," << voltage << std::endl;

		// nothing goes after this
		errorLast = errorCurrent;
		errorLastInt = errorLast;
		pros::delay(10);
	}
}
void turnIMU(double targetAngle, double initialSpeed) {
    int sign;
    double error;
    error = targetAngle - imu.get_heading();
    sign = targetAngle/abs(targetAngle);
    while(autonomous && abs(error) < 5) {
        chassis->getModel()->tank(error/2*sign, -error/2*sign);
    }
    chassis->stop();
}


void gyroTurning(int degrees, double chassis_power) {
    imu.reset();
    const double pi = 3.14159265358979323846;
    double current_pos = imu.get_heading() / 10.0;
    double target_pos = current_pos + degrees;

    double delta = sin((target_pos - current_pos) / 180 * pi);
    double last_delta = abs(delta);
    double current_delta = last_delta;

    while (current_delta <= last_delta) {
        last_delta = current_delta;
        if (delta < 0) {
            chassis->getModel()->tank(chassis_power, -chassis_power);
        } else {
            chassis->getModel()->tank(-chassis_power, chassis_power);
        }
        current_pos = imu.get_heading() / 10;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}

void moveForwardPower(int cycles, double power) {
    for (int i=0; i<cycles; ++i) {
        chassis->getModel()->tank(power, power);
        pros::delay(50);
    }
    chassis->stop();
}

void imuNoSleuth(double degrees, double speed) {
    double cPos;
    double degCoef = (17/18) * degrees;
    if(speed > 0) {
        cPos = imu.get_rotation();
    } else if(speed < 0) {
        cPos = abs(int(imu.get_rotation() - 360)) % 360;
    }
    double tPos = degrees;
    double aDelta = abs(tPos - cPos);
    int threshold = 2.5;
    double speedCoef = 1;

    while(aDelta > threshold) {
        if(35 > aDelta > 53) {
            speedCoef = 1/2;
        } else if(0 > aDelta > 34) {
            speedCoef = 1/7;
        } else if(aDelta < 0) {
            speedCoef = -1/2;
        } else {
            speedCoef = 1;
        } 

        chassis->getModel()->tank(-speedCoef * speed/200, speedCoef * speed/200);

        if(speed > 0) {
            cPos = imu.get_rotation();
        } else if(speed < 0) {
            cPos = abs(int(imu.get_rotation() - 360)) % 360;
       }
        tPos = degrees;
        aDelta = abs(tPos - cPos);

    }
}
void intakeRun(int topspeed, int bottomspeed, int rollerspeed) {
    intake_drive(-rollerspeed, -rollerspeed);
    intake_roller_drive(bottomspeed, -topspeed);
}
void intakeStop() {
    intakeRun(0,0,0);
}

/* ---------------------------------- START OF AUTONS ---------------------------------- */
/* ------------RED 1--------------- */
void test_auton() {
    intakeRun(200, 200, 200);
    pros::delay(190);
    intakeRun(0, 0, 0);
    chassis->setMaxVelocity(1);
    chassis->moveDistance(1_in);
    chassis->turnAngle(1_deg);
    moveForwardPower(20,200);
}
void red_1() {
    intakeRun(200, 0, 0);
    pros::delay(290);
    intakeStop();
    chassis->setMaxVelocity(86);
    chassis->moveDistance(32_in);
    chassis->setMaxVelocity(326);
    chassis->turnAngle(-229.5_deg);

    intakeRun(11, 50, 200);
    moveForwardPower(21, 0.5);
    pros::delay(300);


    moveForwardPower(6, 0.8);
    moveForwardPower(9,0.5);
    intakeRun(-70,-70,-70);
    pros::delay(300);
    intakeRun(200, 200, 200);
    pros::delay(1100);
    moveForwardPower(2,-0.7);
    pros::delay(450);
    moveForwardPower(2, 0.7);

    intakeRun(200, 200, -40);
    pros::delay(350);
    intakeRun(-100,200,-20);
    pros::delay(350);
    chassis->setMaxVelocity(126);
    chassis->moveDistance(-5_in);
    intakeStop();
    chassis->setMaxVelocity(1100);
    chassis->turnAngle(-60_deg);
    moveForwardPower(7,0.7);
    moveForwardPower(20, -0.5);
    intakeRun(-200, -200, -200);
    pros::delay(750);

}
/* ------------BLUE 1--------------- */
void blue_1() {
    intakeRun(200, 0, 0);
    pros::delay(290);
    intakeStop();
    chassis->setMaxVelocity(86);
    chassis->moveDistance(32_in);
    chassis->setMaxVelocity(326);
    chassis->turnAngle(217.5_deg);

    intakeRun(11, 50, 200);
    moveForwardPower(21, 0.5);
    pros::delay(300);


    moveForwardPower(6, 0.8);
    moveForwardPower(9,0.5);
    intakeRun(-70,-70,-70);
    pros::delay(300);
    intakeRun(200, 200, 200);
    pros::delay(1100);
    moveForwardPower(2,-0.7);
    pros::delay(450);
    moveForwardPower(2, 0.7);

    intakeRun(200, 200, -40);
    pros::delay(350);
    intakeRun(-100,200,-20);
    pros::delay(350);
    chassis->setMaxVelocity(126);
    chassis->moveDistance(-5_in);
    intakeStop();
    chassis->setMaxVelocity(1100);
    chassis->turnAngle(50_deg);
    moveForwardPower(7,0.7);
    moveForwardPower(20, -0.5);
    intakeRun(-200, -200, -200);
    pros::delay(750);

 }
 void red_2() {
    // intakeRun(200, 0, 0);
    // pros::delay(60);
    // intakeStop();
    // chassis->setMaxVelocity(186);
    // chassis->moveDistance(25_in);
    // chassis->setMaxVelocity(1226);
    // chassis->turnAngle(-225.5_deg);

    // intakeRun(0, 200, 200);
    // moveForwardPower(7.5, 0.9);
    // pros::delay(300);
    intakeRun(200,0,0);
    pros::delay(100);
    intakeStop();
    intakeRun(0,200,200);
    chassis->setMaxVelocity(150);
    chassis->moveDistance(10_in);

    chassis_tank_drive(120,90);
    pros::delay(120);
    chassis_tank_drive(0,0);

    chassis->setMaxVelocity(1226);
    chassis->turnAngle(-113_deg);

    moveForwardPower(7,0.6);
    pros::delay(400);

    chassis_tank_drive(-8,-8);
    moveForwardPower(2,-0.55);
    intakeRun(200, 140, 200);
    pros::delay(970);
    chassis_tank_drive(0,0);

    intakeRun(200, 200, -40);
    pros::delay(950);
    intakeRun(-100,200,-20);
    pros::delay(350);
    chassis->setMaxVelocity(126);

    moveForwardPower(13, -0.5);
    intakeRun(-200, -100, -200);
    chassis->setMaxVelocity(900);
    chassis->turnAngle(-254.5_deg);
    intakeRun(200,200,-120);
    chassis->setMaxVelocity(170);
    chassis->moveDistance(48.2_in);
    chassis->setMaxVelocity(1200);
    chassis->turnAngle(154_deg);
    intakeRun(200,200,200);
    moveForwardPower(13,0.4);
    pros::delay(300);

    pros::delay(2050);
    moveForwardPower(5,-0.6);
 }
 void skills() {
    intakeRun(200,0,0);
    delay(100);
    intakeRun(-200,0,0);
    delay(200);
    intakeStop();
    toplock();

    intakeRun(0,200,200);
    moveForwardPower(2,0.6);
    moveForwardPower(10, 0.3);
    moveForwardPower(9 ,0.56);
    chassis->setMaxVelocity(1200);
    delay(200);
    chassis->turnAngle(-232_deg);

    topunlock();

    intakeStop();
    moveForwardPower(22.5, 0.5);
    delay(300);

    moveForwardPower(3, 0.5);
    delay(100);
    moveForwardPower(1,-0.3);

    delay(200);

    intakeRun(200,200,200);
    delay(320);

    intakeRun(-20,200,-200);
    moveForwardPower(8.5,-0.4);

    delay(200);
    intakeStop();    

    chassis->turnAngle(231_deg);

    delay(100);
    moveForwardPower(15, -0.4);
    moveForwardPower(4.5, -0.2);


    delay(400);
    moveForwardPower(9, 0.6);
    chassis->setMaxVelocity(140);
    chassis->moveDistance(38_in);

    intakeRun(0,200,200);

    moveForwardPower(14.5, 0.4);
    delay(700);
    intakeStop();

    chassis->setMaxVelocity(1200);
    chassis->turnAngle(-154_deg);
    delay(200);

    moveForwardPower(7, 0.5);
    intakeRun(-200,200,200);
    delay(120);
    intakeRun(200,200,200);

    delay(400);
    intakeStop();

 }
/* ------------BLUE 2--------------- */
void blue_2() {
 
}
void test_auton2() {
    turningP(-10);
}
/* -------FOUR CUBE UNPROTECTED----- */
void four_cube_red() {
   
}
void four_cube_blue() {
   

}
void autonomous() {

    auto program = screen::get_selected_program();
    if (program == "Joker") {
        skills();
        return;
    } else if (program == "Alpha") {
        red_1();
        return;
    } else if (program == "Bravo") {
        red_2();
    } else if (program == "Kilo") {
        blue_1();
    } else if (program == "Lima") {
       blue_2();
    } else {
        skills();
        cout << "yaw = " << imu.get_yaw() << ", cPos = " << imu.get_rotation() << ", k = " << sin(imu.get_rotation()) << endl;
    }


}
