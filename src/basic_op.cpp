#include "main.h"
#include "okapi/api.hpp"
#include "robot.hpp"

using namespace okapi;
using namespace pros;
//chassis
void chassis_tank_drive(int left, int right) {
    const double chassis_movement_threshold = 0.05;
    chassis->getModel()->tank(right / 127.0, left / 127.0, chassis_movement_threshold);
	// chassis_left_front.moveVelocity(-left);
	// chassis_left_rear.moveVelocity(left);
	// chassis_right_front.moveVelocity(-right);
	// chassis_right_rear.moveVelocity(right);
}
//intake
void intake_drive(float left_intake_speed, float right_intake_speed) {
	intake_motor_left.moveVelocity(left_intake_speed);
	intake_motor_right.moveVelocity(-right_intake_speed);
}

void intake_roller_drive(float speed, float speed2) {
	front_intake.moveVelocity(speed);
	top_intake.moveVelocity(speed2);

}

