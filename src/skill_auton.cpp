#include "robot.hpp"
#include "recording.h"

using namespace okapi;
using namespace std;
using namespace recording;

void replay_dp(vector<vector<int>>&);
void turning(int);

double exponential_speed_calculation(int distance_in_tick, double max_speed, double min_speed) {
    
    double exp_speed = pow(abs(distance_in_tick), 1 / 3.0) / 20.0;
    if (exp_speed > max_speed) {
        exp_speed = max_speed;
    } else if (exp_speed < min_speed) {
        exp_speed = min_speed;
    }
    if (distance_in_tick < 0) {
        exp_speed *= -1;
    }
    return exp_speed;
}

void move_straight(double distance_in_inch, double power) {
    const double pi = 3.14159265358979323846;
    const double inch_per_tick = 0.013962634015955;
    const double threshold = 0.5 / inch_per_tick;
    int target_position = distance_in_inch / inch_per_tick;

    auto left_motor = ((SkidSteerModel *)chassis->getModel().get())->getLeftSideMotor();
    auto right_motor = ((SkidSteerModel *)chassis->getModel().get())->getRightSideMotor();

    left_motor->tarePosition();
    right_motor->tarePosition();

    int old_delta = abs(target_position);
    int new_delta = old_delta - 1;
    double left_power = power;
    double right_power = power;
    double left_coef = 1.0;
    double right_coef = 1.0;

    double initial_position = gyro.get_value();

    while (new_delta <= old_delta) {
        old_delta = new_delta;

        left_power = exponential_speed_calculation(old_delta, power, 0.1);
        right_power = left_power;

        double current_position = gyro.get_value();
        double delta_degree = (current_position - initial_position) / 10;

        if (sin(delta_degree / 180 * pi) > sin(5 / 180 * pi)) {
            left_coef *= 0.999;
            right_coef *= 1.001;
            std::cout << ">>> drift right, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        } else if (sin(delta_degree / 180 * pi) < sin(-5 / 180 * pi)) {
            left_coef *= 1.001;
            right_coef *= 0.999;
            cout << "<<< drift left, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        } else {
            cout << "moving straight, left_coef = " << left_coef << ", right_coef = " << right_coef << endl;
        }
        left_power *= left_coef;
        right_power *= right_coef;

        chassis->getModel()->tank(left_power, right_power);
        pros::delay(20);

        int current_encoder_position = max(abs(left_motor->getPosition()), abs(right_motor->getPosition()));
        new_delta = abs(target_position - current_encoder_position);

        cout << "left power = " << left_power << ", right power = " << right_power << endl;
        // cout << "old delta = " << old_delta << ", new delta = " << new_delta << endl;
    }
    chassis->stop();
}

/*
void autonomous() {
    // recording::replay();
    move_straight(24, 0.5);
}
*/

vector<AbstractMotor*>& get_motor_group();

void raise_the_arm_and_release_anti_tip() {

}

void leverBack() {

}

void move_forward_take_9_cubes() {

}

void move_forward(int cycles, double power) {
    for (int i=0; i<cycles; ++i) {
        chassis->getModel()->tank(power, power);
        pros::delay(50);
    }
    chassis->stop();
}

void stack_9_cubes() {

}

void turning(int degrees, double chassis_power) {
    const double pi = 3.14159265358979323846;
    double current_pos = gyro.get_value() / 10.0;
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
        current_pos = gyro.get_value() / 10;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}

void replay_dp(vector<vector<int>>& data_points) {

    vector<RecordUnit> replay_recording;
    for (int i=0; i<data_points.size(); ++i) {
        auto dp = data_points[i];
        RecordUnit ru;
        ru.tick = dp[0];
        for (int j=1; j<dp.size(); ++j) {
            ru.units.push_back(dp[j]);
        }
        replay_recording.push_back(ru);
    }

    std::cout << "about to replay " << replay_recording.size() << " steps" << endl;
    int starting_tick = replay_recording.front().tick;
    int ending_tick = replay_recording.back().tick;

    vector<AbstractMotor*>& motor_group = get_motor_group();    

    int r_index = 0;
    for (int t = starting_tick; t < ending_tick; ++t) {
        RecordUnit& unit = replay_recording[r_index];
        int tick = unit.tick;
        if (tick == t) {
            for (int j=0; j<motor_group.size(); ++j) {
                motor_group[j]->moveVoltage(unit.units[j]);
            }
            r_index ++;
        }
        pros::delay(50);
    }
}

void turningImu(int degrees, double chassis_power) {
    const double pi = 3.14159265358979323846;
    double current_pos = (imu.get_rotation() + (gyro.get_value()/10))/2;
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
        current_pos = (imu.get_rotation() + (gyro.get_value()/10))/2;
        current_delta = abs(sin((target_pos - current_pos) / 180 * pi));

        cout << "target pos = " << target_pos << ", current pos = " << current_pos << ", delta = " << delta << endl;
        pros::delay(20);
    }
    chassis->getModel()->stop();
}
void ten_Cubes() {
  
}

void imuAbs(double degrees, double speed) {
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
    chassis->getModel()->tank(0,0);
}
void imuAbsolute(double speed, double degrees) {
    double cPos;
    if(speed > 0) {
        cPos = imu.get_rotation();
    } else if(speed < 0) {
        cPos = abs(int(imu.get_rotation() - 360)) % 360;
    }
    double tPos = degrees;
    double aDelta = abs(tPos - cPos);
    int threshold = 2.5;
    int rotationDir = (abs(tPos - cPos))/(tPos-cPos);
    while(aDelta > threshold) {
        chassis->getModel()->tank(speed/200 * rotationDir, speed/200 * rotationDir);
        if(speed > 0) {
            cPos = imu.get_rotation();
        } else if(speed < 0) {
            cPos = abs(int(imu.get_rotation() - 360)) % 360;
        }
        tPos = degrees;
        aDelta = abs(tPos - cPos);
        int rotationDir = (abs(tPos - cPos))/(tPos-cPos);
    }
    chassis->getModel()->tank(0,0);
}
void gyroTurnR(double targetAngle, double speed) {
    int threshold = 2;
    int cPos = abs((int)imu.get_rotation() - 360) % 360;
    int tPos = targetAngle;
    int aDelta = abs(cPos - tPos);
    int delta = tPos - cPos;

    std::cout << "the current position is " << cPos << endl;

    while (aDelta >= threshold) {
        // double speedCoef = -(pow(1.0003, 2*(delta+5442.91139001))+226.996)/200;
        double speedCoef = ((5/(pow(10,0.1093637)))*pow(delta,(33/63)))/200;

        chassis->getModel()->tank(-speedCoef*speed/10, speedCoef*speed/10);

        cPos = imu.get_rotation();
        aDelta = abs(cPos - tPos);
        delta = tPos - cPos;
        //std::cout << "the current position is " << speedCoef*speed << endl;
    }
    chassis->getModel()->tank(0,0);
}

void turnP(int targetTurn, int voltageMax=127, bool debugLog=false) {
	float kp = 1.6;
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
		error = targetTurn - imu.get_rotation();
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

void skill_auton() {
   
}