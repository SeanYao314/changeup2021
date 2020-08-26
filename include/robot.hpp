#ifndef __ROBOT_HPP
#define __ROBOT_HPP

#include "main.h"
#include "pros/misc.h"
#include "okapi/api.hpp"

// #define GYRO_PORT 1

//sensors

//chassis motor ports
const int CHASSIS_LEFT_FRONT  = 3;
const int CHASSIS_LEFT_REAR   = 10;
const int CHASIIS_RIGHT_FRONT = 2;
const int CHASSIS_RIGHT_REAR  = 21;

//inkate motor ports
const int INTAKE_MOTOR_LEFT   = 4;
const int INTAKE_MOTOR_RIGHT  = 1;

//lever motor 
const int FRONT_INTAKE = 12;

//arm motor
const int TOP_INTAKE = 20;


const int GYRO_PORT = 8;

const char LINE_TRACKER = 'H';

//chassis
// extern okapi::Motor chassis_left_front;
// extern okapi::Motor chassis_left_rear;
// extern okapi::Motor chassis_right_front;
// extern okapi::Motor chassis_right_rear;
extern okapi::Motor intake_motor_left;
extern okapi::Motor intake_motor_right;
extern okapi::Motor front_intake;
extern okapi::Motor top_intake;
extern pros::Controller master;
extern std::shared_ptr<okapi::OdomChassisController> chassis;

//sensors
extern pros::ADIGyro gyro;
extern pros::Imu imu;

//functions 
void chassis_tank_drive(int left, int right);

void chassis_control();
void intake_drive(float left_intake_speed, float right_intake_speed);
void arm_drive(int pos);

void intake_control();
void intake_roller_drive(float speed, float speed2);
void roller_control();

void auton_simulator();
void gyroTest();

#endif