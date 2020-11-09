#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace pros;


void gyroTest() {
	if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X) > 0) {
		// std::cout << "light: " << pros::c::adi_analog_read('H');
	} 
}


//for chassis
void chassis_control() {
	int right_power = master.get_analog(ANALOG_LEFT_Y);
	int left_power = master.get_analog(ANALOG_RIGHT_Y);
	chassis_tank_drive(left_power, right_power);

}

void auton_simulator() {
	if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
		int x_power = master.get_analog(ANALOG_RIGHT_X);
		if (abs(x_power) > 120) {
			autonomous();
		}

		int left_x_power = master.get_analog(ANALOG_LEFT_X);
		if (abs(left_x_power) > 120) {
			recording::printout();
		}
	}
}
int test = 0;
//intake
void intake_control() {
	if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
		intake_roller_drive(20,-60);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)&&test==1) {
		intake_drive(200,200);
	} else {
		intake_drive(0,0);
		intake_roller_drive(0,0);
	}
}

int ball_check = 2170;
int ball_threshold = 200;
int ball_status = 0;
int test1 = 0;
pros::ADIAnalogIn linetracker ('H');
void roller_control() {
	int ball_detect = linetracker.get_value();
	if (master.get_digital(E_CONTROLLER_DIGITAL_L1) && ball_detect > ball_check) {
		intake_roller_drive(200, -180);
		intake_drive(-200, -200);
		pros::delay(40);
		top_intake.setBrakeMode(AbstractMotor::brakeMode::coast);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_L1) && ball_detect <= ball_check) {
		intake_roller_drive(300, -0);
		intake_drive(-200, -200);
		top_intake.setBrakeMode(AbstractMotor::brakeMode::hold);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		top_intake.setBrakeMode(AbstractMotor::brakeMode::coast);
		intake_roller_drive(200, -200);
		intake_drive(20, 20);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_UP)) {
		intake_roller_drive(80, -100);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_DOWN)) {
		intake_roller_drive(30, 130);
		intake_drive(-80,80);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		intake_roller_drive(200, 200);
		intake_drive(-100, -100);
		top_intake.setBrakeMode(AbstractMotor::brakeMode::coast);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_B)) {
		intake_roller_drive(-200, 200);
		intake_drive(100, 100);
		top_intake.setBrakeMode(AbstractMotor::brakeMode::coast);
	} else if(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
		//test 
		top_intake.moveRelative(-4750,-200);
		pros::delay(120);
		top_intake.moveRelative(0,-200);
		pros::delay(200);

	} else {
		intake_roller_drive(0, 0);
		intake_drive(0,0);
	}
}