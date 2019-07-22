#include "opcontrol.h"

#include "main.h"
#include "constants.h"
#include "pid.h"
#include "global.h"
#include "autonomous.h"
#include <sstream>
#include <chrono>

Constants* constants = Constants::GetInstance();
auto awning_pid = new Pid(&constants->awning_kp, &constants->awning_ki, &constants->awning_kd);
auto lift_pid = new Pid(&constants->lift_kp, &constants->lift_ki, &constants->lift_kd);


template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 0)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}
// Display Values on Screen

void LogValues () {
    std::cout << to_string_with_precision(pros::millis(),7) << "," <<
                 to_string_with_precision(left_front_wheels.get_position(),7) << "," <<
                 to_string_with_precision(left_center_wheels.get_position(),7) << "," <<
                 to_string_with_precision(left_back_wheels.get_position(),7) << "," <<
                 to_string_with_precision(right_front_wheels.get_position(),7) << "," <<
                 to_string_with_precision(right_center_wheels.get_position(),7) << "," <<
                 to_string_with_precision(right_back_wheels.get_position(),7) << "," <<
                 to_string_with_precision(lift_motor.get_position(),7) << "," <<
                 to_string_with_precision(awning_motor.get_position(),7) << std::endl;
}

void DisplayValues () {
    if (pros::lcd::is_initialized() == false) {
        pros::lcd::initialize();
    }
    pros::lcd::clear_line(1);
    pros::lcd::set_text(1, "Position: " +
                           to_string_with_precision(left_front_wheels.get_position()) + "," +
                           to_string_with_precision(left_center_wheels.get_position()) + "," +
                           to_string_with_precision(left_back_wheels.get_position()) + "," +
                           to_string_with_precision(right_front_wheels.get_position()) + "," +
                           to_string_with_precision(right_center_wheels.get_position()) + "," +
                           to_string_with_precision(right_back_wheels.get_position()) + "," +
                           to_string_with_precision(lift_motor.get_position()) + "," +
                           to_string_with_precision(awning_motor.get_position()));
    pros::lcd::clear_line(2);
    pros::lcd::set_text(2, "Velocity: " +
                           to_string_with_precision(left_front_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(left_center_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(left_back_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(right_front_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(right_center_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(right_back_wheels.get_actual_velocity()) + "," +
                           to_string_with_precision(lift_motor.get_actual_velocity()) + "," +
                           to_string_with_precision(awning_motor.get_actual_velocity()));
    pros::lcd::set_text(6, "Awning Constants: kp - " + std::to_string(*&constants->awning_kp) +
                           ", ki - " + std::to_string(*&constants->awning_ki) + ", kd - " +
                           std::to_string(*&constants->awning_kd));
    pros::lcd::set_text(7, "Lift Constants: kp - " + std::to_string(*&constants->lift_kp) + ", ki - " +
                           std::to_string(*&constants->lift_ki) + ", kd - " +
                           std::to_string(*&constants->lift_kd));
}

// Random Useful Functions

bool soft_stop(bool is_maximum, double stop_position, double current_position) {
	if (is_maximum == true and stop_position >= current_position) {
		return false;
	}
	else if (is_maximum == false and stop_position <= current_position) {
		return false;
	}
	else {
		return true;
	}
}

// Analog
// Drives

void ArcadeDrive() {
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_LEFT_X);
    int left = power + turn;
    int right = power - turn;

    left_front_wheels.move(left);
    left_center_wheels.move(left);
    left_back_wheels.move(left);
    right_front_wheels.move(right);
    right_center_wheels.move(right);
    right_back_wheels.move(right);
}
void setDriveLeft(int left){
  left_front_wheels.move(left);
  left_center_wheels.move(left);
  left_back_wheels.move(left);

}
void setDriveRight(int right){
  right_front_wheels.move(right);
  right_center_wheels.move(right);
  right_back_wheels.move(right);

}
void SplitArcadeDrive() {
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int left = power + turn;
    int right = power - turn;

    left_front_wheels.move(left);
    left_center_wheels.move(left);
    left_back_wheels.move(left);
    right_front_wheels.move(right);
    right_center_wheels.move(right);
    right_back_wheels.move(right);
}

void TankDrive() {
		left_front_wheels.move(master.get_analog(ANALOG_LEFT_Y));
		left_center_wheels.move(master.get_analog(ANALOG_LEFT_Y));
		left_back_wheels.move(master.get_analog(ANALOG_LEFT_Y));
		right_front_wheels.move(master.get_analog(ANALOG_RIGHT_Y));
		right_center_wheels.move(master.get_analog(ANALOG_RIGHT_Y));
		right_back_wheels.move(master.get_analog(ANALOG_RIGHT_Y));
}

// Digital
// Miscellaneous

void AwningPositions() {

}

void Awning() {

    if (master.get_digital(DIGITAL_UP)) {
        awning_motor.move_velocity(100);
        awning_setpoint = awning_motor.get_position();
        awning_pid->ResetError();
    } else if (master.get_digital(DIGITAL_LEFT)) {
        awning_motor.move_velocity(-100);
        awning_setpoint = awning_motor.get_position();
        awning_pid->ResetError();
    } else {
        awning_output = awning_pid->Update(awning_setpoint, awning_motor.get_position());
        awning_motor.move_velocity(awning_output);
    }

}

void AwningControl(int awning_pos) {
	if (awning_pos == 0 && awning_motor.get_position() > 0)
		awning_motor.move_velocity(-25);
	else if (awning_pos == 1 && awning_motor.get_position() < 300 + pros::battery::get_capacity()/11)
		awning_motor.move_velocity(100);
	else if (awning_pos == 2) {
		Awning();
	} else
        //awning_output = awning_pid->Update(0, awning_setpoint);
		awning_motor.move_velocity(0);
}

void Lift() {
    if (master.get_digital(DIGITAL_R1)) {
        lift_motor.move_velocity(150);
        lift_setpoint = lift_motor.get_position();
        lift_pid->ResetError();
    } else if (master.get_digital(DIGITAL_R2)) {
        lift_motor.move_velocity(-100);
        lift_setpoint = lift_motor.get_position();
        lift_pid->ResetError();
    } else {
        //lift_output = lift_pid->Update(lift_setpoint, lift_motor.get_position());
        lift_motor.move_velocity(0);
    }
}

#define VISION_PORT 10


#define EXAMPLE_SIG 1
/**
vision::signature SIG_1 (1, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_2 (2, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature TOPBLUE (3, 0, 0, 0, 0, 0, 0, 2.900, 0);
vision::signature SIG_4 (4, -2979, 1, -1489, 697, 7877, 4287, 0.000, 0);
vision::signature SIG_5 (5, -2919, 1, -1459, 841, 8643, 4742, 1.200, 0);
vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3.000, 0);
vex::vision vision1 ( vex::PORT1, 50, SIG_1, SIG_2, TOPBLUE, SIG_4, SIG_5, SIG_6, SIG_7 );

vision::signature SIG_1 (1, -4275, 1, -2137, -8121, -983, -4552, 1.100, 0);
vision::signature SIG_2 (2, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3.000, 0);
vex::vision vision1 ( vex::PORT1, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7 );
**/
void sensors() {
  pros::Vision sensor(VISION_PORT);
  sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  // values acquired from the vision utility
  pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(EXAMPLE_SIG, -2809, -1069, -1939, 4587, 11147, 7867, 1.900, 0);

  sensor.set_signature(EXAMPLE_SIG, &RED_SIG);
  while (true) {
    pros::vision_object_s_t rtn = sensor.get_by_sig(0, EXAMPLE_SIG);
    // Gets the largest object of the EXAMPLE_SIG signature
    pros::lcd::set_text(2, std::to_string(rtn.signature));
    //pros::lcd::set_text(3, std::to_string(rtn.angle));
    pros::lcd::set_text(4, std::to_string(rtn.x_middle_coord));
    pros::lcd::set_text(5, std::to_string(rtn.y_middle_coord));
    pros::lcd::set_text(6, std::to_string(rtn.width));
    pros::lcd::set_text(7, std::to_string(rtn.height));
    if (master.get_digital(DIGITAL_B)) {
		    if((rtn.width>50||rtn.height>50)&&(abs(rtn.x_middle_coord)<70)&&(rtn.width<310)){
          setDriveLeft(-50);
          setDriveRight(-50);
        }else{
          setDriveLeft(0);
          setDriveRight(0);
        }
    }else if(master.get_digital(DIGITAL_A)){
      if((rtn.width>50||rtn.height>50)&&(rtn.signature==1)&&(rtn.x_middle_coord<-60)){
        pros::lcd::set_text(3, "greater Than 0");
        setDriveLeft(50);
        setDriveRight(-50);
      } else if((rtn.width>50||rtn.height>50)&&(rtn.signature==1)&&(rtn.x_middle_coord>60)) {
          pros::lcd::set_text(3, "less than 0");
          setDriveLeft(-50);
          setDriveRight(50);
        } else{
          pros::lcd::set_text(3, "nothing");
          setDriveLeft(0);
          setDriveRight(0);
    }
    // Prints "sig: 1"

  }else{
   pros::lcd::set_text(3, "nothing");
   setDriveLeft(0);
   setDriveRight(0);
}
  pros::delay(200);
}

}
#define ECHO 7
#define PING 8
#define GYRO1 6
#define GYRO2 5
#define ECHO2 3
#define PING2 4
#include "pros/api_legacy.h"
void ultraSonic(){
    pros::ADIUltrasonic ultrasonic (ECHO, PING);
    pros::ADIUltrasonic ultrasonic2 (ECHO2, PING2);
    pros::ADIAnalogIn gyro1 (GYRO1);
    pros::ADIAnalogIn gyro2 (GYRO2);
  gyro2.calibrate();
  gyro1.calibrate();
  int distance = 0;
  int distance2 = 0;
  int angle = 0;
  int angle2 = 0;

  while(1) {
    pros::delay(20);
    distance = ultrasonic.get_value();
    distance2 = ultrasonic2.get_value();
    if(abs(gyro1.get_value()-1844)>2){
        angle = angle + 1844 - gyro1.get_value();
    }
    if(abs(gyro2.get_value()-1859)>2){
        angle2 = angle2 + 1859 - gyro2.get_value();
    }
    pros::lcd::set_text(1, std::to_string(gyro1.get_value()));
    pros::lcd::set_text(2, std::to_string(gyro2.get_value()));
    pros::lcd::set_text(3, std::to_string(distance));
    pros::lcd::set_text(4, std::to_string(distance2));
    pros::lcd::set_text(5, std::to_string(angle)+"<-- value, angle:  "+ std::to_string(angle*90/5819));
    pros::lcd::set_text(6, std::to_string(angle2)+"<-- value, angle:  "+ std::to_string(angle2*720/49033));

    if (master.get_digital(DIGITAL_B)) {
      if(abs(angle+angle2)<6000 || abs(distance-distance2)>10 ){
        setDriveLeft(-30);
        setDriveRight(30);

      }else{
        setDriveLeft(0);
        setDriveRight(0);
      }
    } else if (master.get_digital(DIGITAL_A)){
        angle = 0;
        angle2 = 0;
    } else if (master.get_digital(DIGITAL_X)){
        if(abs(angle+angle2)<6000 || abs(distance-distance2)>10 ){
          setDriveLeft(30);
          setDriveRight(-30);
        }else{
          setDriveLeft(0);
          setDriveRight(0);
        }
    } else if(master.get_digital(DIGITAL_Y)){
      if(abs(angle*90/5819+angle2*720/49033)+5<720){
        setDriveLeft(30);
        setDriveRight(-30);

      } else{
        setDriveLeft(0);
        setDriveRight(0);
      }
    } else {
      setDriveLeft(0);
      setDriveRight(0);
    }
  }




}
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
	int awning_pos = 0; //0 = back, 1 = up, 2 = manual
	int lift_pos = 0; //0 = manual, 1 = flag toggle
//  sensors();
//ultraSonic();
    std::uint32_t now = pros::millis();

	while (true) {
        TankDrive();
        Lift();
		AwningPositions();
        DisplayValues();
        LogValues();

		if (master.get_digital(DIGITAL_X)) {
			lift_pos = 1;
        }

		if (lift_pos == 1) {
			if (lift_motor.get_position() < 850)
				lift_motor.move_velocity(100);
			else if (lift_motor.get_position() > 950)
				lift_motor.move_velocity(-100);
			else {
				lift_motor.move_velocity(0);
				lift_pos = 0;
			}
		}

		if (master.get_digital(DIGITAL_L2)) {
			awning_pos = 0;
        }
		if (master.get_digital(DIGITAL_L1)) {
			awning_pos = 1;
        }
		if (master.get_digital(DIGITAL_RIGHT)) {
			awning_motor.tare_position();
        }
		if (master.get_digital(DIGITAL_DOWN)) {
			awning_pos = 2;
        }

		AwningControl(awning_pos);

        pros::Task::delay_until(&now, 20);
	}
}
