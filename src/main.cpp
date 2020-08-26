#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace pros;

void disabled() {}

void competition_initialize() {

}

void opcontrol() {
	while (true) {
		//gyrodisplay
		gyroTest();

		//auton
		auton_simulator();

		//chassis stuff
		chassis_control();

		//intake
		intake_control();

		//roller
		roller_control();


		recording::record();
		pros::delay(ITERATION_INTERVAL);
		
	}
}
