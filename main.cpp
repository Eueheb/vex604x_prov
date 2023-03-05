#include "main.h"
#include "pros/rtos.h"
using namespace pros;
Motor rightFrontMotor(20);
Motor rightBackMotor(6);
Motor leftFrontMotor(13);
Motor leftBackMotor(4);
Motor leftTopMotor(18);
Motor rightTopMotor(11);
Motor intake(9);
Motor catapult(19);
ADIDigitalOut piston(3);
ADIDigitalIn  cata(8);
ADIDigitalOut launcherA(7);
ADIDigitalOut band_boost(6);
Imu gyro(8);



Controller master(E_CONTROLLER_MASTER);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// Vision vision(9);
// vision_signature_s_t blue = Vision::signature_from_utility(1, -3109, -395, -1752, 489, 10795, 5642, 0.800, 0);
// vision_signature_s_t til = Vision::signature_from_utility( 2, -5121, -4151, -4636, -2257, -1637, -1948, 3.000, 0);
// int disc_Pos[120]/* = (ODOM POSITION OF EACH DISC (x value, y value))*/;

double catapult_state = 0;

//driving straight by duration
void drive(double target, double power, double k_t_p){
	double t_error;
	double past_time = millis();
	double delta_time;
	while(delta_time < target){
		delta_time = millis() - past_time;
		t_error = -gyro.get_rotation();
		rightFrontMotor = power - (t_error*k_t_p);
		rightBackMotor = power - (t_error*k_t_p);
		rightTopMotor = -power + (t_error*k_t_p);
		leftBackMotor = -power - (t_error*k_t_p);
		leftFrontMotor = -power - (t_error*k_t_p);
		leftTopMotor = power + (t_error*k_t_p);
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
	leftTopMotor = 0;
}
//PID drive with acceleration
void pidDrive_complex(double target, double init_power,double kp, double ki, double kd, double k_a_p, double k_t_p, double timeout,double accuracy){
	double a_error;
	double a_target = target/2;
	double max_time = 2000;
	double start_time = millis();
  	gyro.tare_rotation();
	double t_error;
	double power;
	rightFrontMotor.tare_position();

	double derivative = 0;
	double integral = 0;
	double past_error = 0;
	double error = target - rightFrontMotor.get_position();

	double delta_time;
	double past_time;
	bool exit = false;
	while(fabs(error) > 5 && exit == false && millis()-start_time < max_time){
		past_error = error;
		error = target - rightFrontMotor.get_position();
		pros::screen::print(TEXT_MEDIUM, 1,"Current rotation %f",error);
		if(past_error != error || fabs(target - error) < accuracy){
			delta_time = millis() - past_time;
			past_time = millis();
			if(fabs(error) < 50){
			integral = integral + error;
			}
			else{
				integral = 0;
			}
			derivative = error - past_error;
			a_error = rightFrontMotor.get_position();
			t_error = -gyro.get_rotation();
			if (fabs(rightFrontMotor.get_position()) < fabs(a_target)){
				power = (a_error*k_a_p) + init_power - (t_error*k_t_p);
			}
			else {
				power = (error*kp) + (derivative*kd) + (integral*ki);
			}
			if(fabs(power) < 15){
				power = power/fabs(power)*15;
			}
			rightFrontMotor = power - (t_error*k_t_p);
			rightBackMotor = power - (t_error*k_t_p);
			rightTopMotor = -power + (t_error*k_t_p);
			leftBackMotor = -power - (t_error*k_t_p);
			leftFrontMotor = -power - (t_error*k_t_p);
			leftTopMotor = power + (t_error*k_t_p);
			}
		else if(delta_time > timeout && error < accuracy){
			exit = true;
		}
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
	leftTopMotor = 0;
}


// Normal PID drive
void pidDrive(double target, double kp, double ki, double kd, double k_t_p, double timeout,double accuracy){
  	gyro.tare_rotation();
	double t_error;
	double power;
	rightFrontMotor.tare_position();
	double start_time = millis();
	double max_time = 1000;
	double derivative = 0;
	double integral = 0;
	double past_error = 0;
	double error = target - rightFrontMotor.get_position();

	double delta_time;
	double past_time;
	bool exit = false;
	while(fabs(error) > 3 && exit == false && millis() - start_time < max_time){
		past_error = error;
		error = target - rightFrontMotor.get_position();
		pros::screen::print(TEXT_MEDIUM, 1,"Current rotation %f",error);
		if(past_error != error || fabs(target - error) < accuracy){
			delta_time = millis() - past_time;
			past_time = millis();
			if(fabs(error) < 10){
			integral = integral + error;
			}
			else{
				integral = 0;
			}
			derivative = error - past_error;
			t_error = -gyro.get_rotation();
			power = (error*kp) + (derivative*kd) + (integral*ki);
			if (fabs(power) >= 100){
				if(power < 0){
					power = -100;
				}
				else{
					power = 100;
				}
			}
			rightFrontMotor = power - (t_error*k_t_p);
			rightBackMotor = power - (t_error*k_t_p);
			rightTopMotor = -power + (t_error*k_t_p);
			leftBackMotor = -power - (t_error*k_t_p);
			leftFrontMotor = -power - (t_error*k_t_p);
			leftTopMotor = power + (t_error*k_t_p);
			}
		else{
			delta_time = millis() - past_time;
			if(delta_time > timeout && error < accuracy){
				exit = true;
			}
		}
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
	leftTopMotor = 0;
}
// PID turn with acceleration
void PID_Turn_complex(double target, double init_power, double kp, double ki, double kd, double k_a_p, double timeout, double accuracy){

	gyro.tare_rotation();
	double max_time = 2000;
	double start_time = millis();
	double a_error;
	double a_target = target/2;

	double derivative = 0;
	double integral = 0;
	double past_error = 0;
	double error = target - gyro.get_rotation();

	double delta_time;
	bool exit = false;
	double past_time;

	double power;
	while(fabs(error) > 1 && exit == false && millis() - start_time < max_time){
		past_error = error;
		error = target - gyro.get_rotation();
		if(fabs(past_error - error) > 1 || fabs(target - error) < accuracy){
			if(fabs(error) <5){
				integral = integral + error;
			}
			else{
				integral = 0;
			}
			derivative = error - past_error;

			a_error = gyro.get_rotation();
			if (fabs(gyro.get_rotation()) < a_target){
				power = (a_error*k_a_p) + init_power;
			}
			else{
				power = (error*kp) + (derivative*kd) + (integral*ki);
			}
			rightFrontMotor = -power;
			rightBackMotor = -power;
			rightTopMotor = power;
			leftBackMotor = -power;
			leftFrontMotor = -power;
			leftTopMotor = power;
		}
		else if(delta_time > timeout){
			exit = true;
		}
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
	leftTopMotor = 0;
}


//Normal PID turn
void PID_Turn(double target, double kp, double ki, double kd, double timeout, double accuracy){

	gyro.tare_rotation();

	double derivative = 0;
	double integral = 0;
	double past_error = 0;
	double error = target - gyro.get_rotation();
	double max_time = 1200;
	double start_time = millis();
	double delta_time;
	bool exit = false;
	double past_time;

	double power;
	while(fabs(error) > 0.5 && exit == false){
		past_error = error;
		error = target - gyro.get_rotation();
		if((fabs(past_error - error) > 1 || fabs(error) > accuracy )&& millis()-start_time < max_time ){
			past_time = millis();
			if(fabs(error) <5){
				integral = integral + error;
			}
			else{
				integral = 0;
			}
			derivative = error - past_error;
			power = (error*kp) + (derivative*kd) + (integral*ki);
			rightFrontMotor = -power;
			rightBackMotor = -power;
			rightTopMotor = power;
			leftBackMotor = -power;
			leftFrontMotor = -power;
			leftTopMotor = power;
		}
		else{
			delta_time = millis() - past_time;
			if(delta_time > timeout){
				exit = true;
			}
		}
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
	leftTopMotor = 0;
}

void cata_reload(){
	int cata_state = 0;
	while(cata_state == 0){
		
		if(cata.get_value() == 1){
			catapult = 0;
			cata_state = 1;
		}
		else{
			catapult = 110;
		}
	}
	catapult = 0;
	delay(10);
}
bool inAuto = true;
int catastate = 0;
void cata_auto(){
	while(inAuto){
		c::delay(10);
		if(cata.get_value() == 1 && catastate != 2){
			catapult = 0;
			catastate = 1;
		}
		else if(catastate == 0){
			catapult = 110;
		}
		else if(catastate != 2){
			catapult = 0;
		}
	}
}
void shoot(){
	catastate = 2;
	catapult = 127;
	delay(800);
	catastate = 0;
}
//
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	gyro.tare_rotation();
	piston.set_value(false);
	band_boost.set_value(false);
	launcherA.set_value(false);
}
//vision sensor
/*void toggle(){

	double x = rtn.x_middle_coord;
	double y = rtn.y_middle_coord;
	double past_x;
	double past_y;
	double before_x;
	double before_y;
	double error;
	double kp = 0.1;
	double power;
	double av_x;
	double av_y;
	while(fabs(x) >= 2){
		before_x = past_x;
		before_y = past_y;
		past_x = x;
		past_y = y;
		x = rtn.x_middle_coord;
		y = rtn.y_middle_coord;
		av_x = (before_x +past_x + x)/(3);
		error = av_x;
		power = error * kp;
		rightFrontMotor = -power;
		rightBackMotor = -power;
		leftBackMotor = power;
		leftFrontMotor = power;
		//pros::screen::print(TEXT_MEDIUM, 1,"%f", rtn.x_middle_coord);
	}
	rightFrontMotor = 0;
	rightBackMotor = 0;
	leftBackMotor = 0;
	leftFrontMotor = 0;
}
*/
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 

 void Intake(double power){
   intake = power;
}

void powerDrive(double power){
	rightFrontMotor = power;
	rightBackMotor = power;
	rightTopMotor = -power;
	leftBackMotor = -power;
	leftFrontMotor = -power;
	leftTopMotor = power;
}

void match_auto_left(){
	//roller
	pidDrive(-30, 0.8, 1, 1, 1, 100, 40);
	//turning the rollar
	intake = 90;
	delay(180);
	intake = 0;
	//getting to center for shooting position
	pidDrive(380,0.25,0,1,1,300,500);
	rightFrontMotor = 35;
	rightBackMotor = 35;
	rightTopMotor = -35;
	delay(290);
	rightBackMotor = 0;
	rightFrontMotor = 0;
	rightTopMotor = 0;
	delay(100);
	shoot();
	Task cataTask(cata_auto);
	delay(500);
	//intake 3 more disks 
	PID_Turn(-112,0.94,1.1,1,500,10);
	intake = -120;
	delay(200);
	pidDrive_complex(-1750,-20,0.15,0,2,0.07,1,300,500);
	pidDrive_complex(-630,-20,0.3,0,1.5,0.2,1,300,500);
	delay(500);
	//aiming
	PID_Turn(89,1,0,1,300,10);
	delay(450);
	shoot();
	delay(500);
	//intaking two more disks
	PID_Turn(-103,1,0,1,300,10);
	delay(250);
	pidDrive_complex(-1200,-40,0.5,0,2,0.5,1,300,500);
	delay(200);
	//aiming
	PID_Turn(90,1,0,1,300,10);
	pidDrive_complex(-150,-40,1,0,1.5,1,1,300,500);
	delay(450);
	shoot();
	//releasing bandboost
	band_boost.set_value(true);

	


	/*PID_Turn(90,0.9,0.5,3,300,10);
	catapult = 127;
	delay(500); 
	cata_reload();*/
	/*PID_Turn(45,1.3,0.3,1,500,10);
	delay(200);
	pidDrive(2800,0.17,0,3,1,500,500);
	delay(300);
	PID_Turn(-87,0.9,0.5,3,300,10);
	pidDrive(200,0.25,0,1,1,300,500);
	delay(500);
	catapult = 127;
	delay(1800);
	catapult = 0;
	//go to the other roller on the same side
	pidDrive(-300,0.2,0,1,1,300,500);
	delay(500);
	PID_Turn(-93,0.9,0.5,3,300,10);
	intake = 127;
	delay(500);
	pidDrive(-3850,0.15,0,1,1,500,500);
	delay(200);
	intake = 0; 
	PID_Turn(45,1.2,0.3,2,300,10);
	delay(500);
	pidDrive(-250,0.4,0,1,1,300,500);
	delay(100);
	intake = 127;
	delay(500);
	intake = 0;*/
}
void match_auto_right(){
	//the roller
	//getting to the rollar
	pidDrive_complex(-950,-30,0.3,0,1.5,0.1,1.1,300,500);
	delay(250);
	PID_Turn(90,1,0,1,300,10);
	//turning the rollar
	delay(250);
	intake = 90;
	pidDrive_complex(-290,-30,0.3,0,1.5,0.1,1,300,500);
	//getting to the shooting position to shoot two disks
	pidDrive_complex(800,30,0.3,0,1.5,0.1,1,300,500);
	intake = 0;
	delay(500);
	//turning to aim
	leftFrontMotor = -35;
	leftBackMotor = -35;
	leftTopMotor = 35;
	rightFrontMotor = -35;
	rightBackMotor = -35;
	rightTopMotor = 35;
	delay(160);
	leftFrontMotor = 0;
	leftBackMotor = 0;
	leftTopMotor = 0;
	rightFrontMotor = 0;
	rightBackMotor = 0;
	rightTopMotor = 0;
	delay(250);
	shoot();
	Task cataTask(cata_auto);
	//intaking to two more disks
	pidDrive_complex(-500,-30,0.3,0,1.5,0.1,1,300,500);
	delay(500);
	PID_Turn(126,0.9,0,1,300,10);
	intake = -120;
	pidDrive_complex(-2350, -20, 0.15, 0, 1.5, 0.1,1.8,200, 2000);
	delay(500);
	//returning to orginial shooting positon
	pidDrive_complex(2150, 20, 0.15, 0, 1.5, 0.1,1.8,200, 2000);
	delay(200);
	PID_Turn(-123,0.9,0,1,300,10);
	delay(200);
	pidDrive_complex(300,30,0.3,0,1.5,0.1,1,300,500);
	delay(1000);
	shoot();
	//old version
	/*PID_Turn(-90,1.3,0,1,300,10);
	delay(200);
	pidDrive_complex(-190,-30,0.3,0,1.5,0.1,1,300,500);
	delay(600);
	shoot();
	PID_Turn(-90,0.9,0,1,300,10);
	pidDrive_complex(-1500,-30,0.3,0,1.5,0.1,1,300,500);
	*/
	//releasing bandboost
	delay(500);
	band_boost.set_value(true);
	




}
void skill_auto(){
	//do the first two rollers and intake one disc
	pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	intake = 90;
	delay(180);
	intake = 0;
	pidDrive(90,0.55,1,1,1,300,500);
	delay(400);
	PID_Turn(135,0.92,1.1,1,500,10);
	delay(500);
	intake = -127;
	pidDrive(-1200,0.1,0,2,1,300,500);
	delay(400);
	PID_Turn(-45,1.3,0,1,300,10);
	intake = 0;
	delay(200);
	pidDrive(-550, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(180);
	intake = 0;
	//shoot three disc
	pidDrive(60,0.6,0,1,1,200,90);
	delay(500);
	PID_Turn(-92,1,0.1,1.5,300,10);
	delay(500);
	pidDrive_complex(3200, 30, 0.23, 0, 1.5, 0.11,1.8,200, 2000);
	delay(200);
	PID_Turn(10.5,3.2,0.5,3,300,5);
	delay(200);
	catapult = 127;
	delay(500);
	cata_reload();
	PID_Turn(-10.5,3.2,0.5,3,300,5);
	delay(500);
	//back up and turn to get other three discs
	pidDrive_complex(-2800, -30, 0.1, 0, 1, 0.1,1.8,200, 2000);
	delay(200);
	PID_Turn(-132,0.92,1.1,1,500,10);
	delay(200);
	intake = -127;
	pidDrive_complex(-3250,-30,0.1,0,1,0.04,1,200,2000);
	delay(300);
	//another route
	/*pidDrive(1200,0.09,0,2,1,300,500);
	intake = 0;
	delay(200);
	PID_Turn(45,1.3,0,1,300,10);
	delay(200);
	pidDrive(1600,0.08,0,2,1,300,500);
	delay(200);
	PID_Turn(90,0.9,0,1,300,10);
	delay(200);
	pidDrive(1500,0.09,0,2,1,300,500);
	delay(200);
	PID_Turn(11,3.2,0.5,3,300,5);
	delay(200);
	catapult = 127;
	delay(1800);
	catapult = 0;
	PID_Turn(-11,3.2,0.5,3,300,5);
	delay(200);
	pidDrive(-1200,0.09,0,2,1,300,500);
	delay(200);
	PID_Turn(-90,0.9,0,1,300,10);
	delay(200);
	pidDrive(-1200,0.09,0,2,1,300,500);
	delay(200);
	PID_Turn(-45,1.3,0,1,300,10);*/
	PID_Turn(90,0.9,0,1,300,10);
	delay(200);
	//shoot three discs
	pidDrive(530, 0.15, 0, 1, 1, 300, 300);
	intake = 0;
	catapult = 127;
	delay(500);
	cata_reload();
	//back up and turn to get three discs
	pidDrive(-690, 0.1, 0, 1, 1, 300, 300);
	delay(200);
	PID_Turn(-90,0.9,0,1,300,10);
	delay(200);
	pidDrive_complex(-1500,-20,0.1,0,2,0.08,1,300,500);
	intake = -127;
	delay(700);
	intake = -127;
	pidDrive_complex(-1370,-15,0.2,0,1.5,0.05,1,300,500);
	delay(200);
	PID_Turn(47,1.3,0,1,300,10);
	delay(200);
	//shoot three discs
	pidDrive_complex(2600,20,0.2,0,1.5,0.095,1,200,2000);
	delay(500);
	PID_Turn(-8.5,3.2,0.5,3,300,5);
	delay(200);
	catapult = 127;
	delay(500);
	cata_reload();
	PID_Turn(10,3.3,0.5,3,300,5);
	delay(500);
	//back up and do other two rollers
	pidDrive_complex(-3340,-20,0.15,0,1.5,0.09,1.8,200,2000);
	delay(200);
	intake = 0;
	PID_Turn(-97,0.93,0,1,300,10);
	delay(200);
	pidDrive(-900,0.25,0,1,1,300,500);
	//pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	delay(500);
	intake = 127;
	delay(350);
	intake = 0;
	delay(400);
	pidDrive(100,0.55,0,1,1,300,500);
	delay(400);
	PID_Turn(135,0.92,1.1,1,500,10);
	delay(500);
	intake = -127;
	pidDrive(-1200,0.09,0,2,1,300,500);
	delay(400);
	PID_Turn(-45,1.2,0,1,300,10);
	intake = 0;
	delay(200);
	intake = 80;
	pidDrive(-160, 0.6, 0, 1, 1, 300, 300);
	delay(500);
	intake = 80;
	delay(250);
	intake = 0;
	delay(200);
	pidDrive(300,0.55,0,1,1,300,500);
	delay(200);
	PID_Turn(137,0.92,1.1,1,300,10);
	delay(200);
	//launch string
	launcherA.set_value(true);
	delay(500);
	launcherA.set_value(false);
	delay(500);
	launcherA.set_value(true);
}


void skill_auto_ver2(){
	pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	intake = 90;
	delay(250);
	intake = 0;
	pidDrive(90,0.5,1,1,1,300,500);
	delay(300);
	PID_Turn(135,0.94,1.1,1,500,10);
	intake = -127;
	pidDrive(-1200,0.17,0,2,1,300,500);
	delay(200);
	PID_Turn(-45,1.3,0,1,300,10);
	intake = 0;
	delay(200);
	pidDrive(-590, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	//shoot three disc
	pidDrive(60,0.7,0,1,1,200,90);
	delay(500);
	PID_Turn(-89.5,1,0.1,1.5,300,10);
	delay(500);	
	pidDrive_complex(2820, 30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	delay(200);
	PID_Turn(8,3.2,0.5,3,300,5);
	delay(300);
	shoot();
	Task catatask(cata_auto);
	PID_Turn(-98, 1,0.1,1.5,300,10);
	delay(200);
	intake = -127;
	pidDrive_complex(-1650,-20,0.1,0,1,0.1,1,300,500);
	delay(300);
	pidDrive_complex(1560,20,0.1,0,1,0.1,1,300,500);
	delay(200);
	PID_Turn(99, 1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	delay(300);
	shoot();
	PID_Turn(-49,1.3,0,1,300,10);
	intake = -127;
	delay(200);
	pidDrive_complex(-1600,-20,0.2,0,2,0.15,1,300,500);
	delay(200);
	PID_Turn(-90,1,0.1,1.5,300,10);
	delay(200);
	pidDrive_complex(-1500,-20,0.2,0,2,0.15,1,300,500);
	delay(300);
	PID_Turn(90,1,0.1,1.5,300,10);
	delay(300);
	pidDrive(550, 0.15, 0, 1, 1, 300, 300);
	intake = 0;
	shoot();
	pidDrive(-340, 0.15, 0, 1, 1, 300, 300);
	delay(300);
	PID_Turn(-136,0.94,1.1,1,500,10);
	intake = -127;
	delay(200);
	pidDrive_complex(-2270,-20,0.1,0,2,0.08,1,300,500);
	delay(200);
	PID_Turn(87,1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-60,0.7,0,1,1,200,90);
	delay(300);
	intake = 0;
	shoot();
	PID_Turn(45,1.3,0,1,300,10);
	delay(200);intake = -127;
	pidDrive_complex(-2000,-20,0.02,0,2,0.02,1,300,500);
	delay(300);
	pidDrive_complex(-670,-15,0.3,0,1.5,0.1,1,300,500);
	pidDrive_complex(1950,20,0.08,0,2,0.06,1,300,500);
	delay(300);
	PID_Turn(-39,1.3,0,1,300,10);
	delay(200);
	pidDrive(-230, 0.15, 0, 1, 1, 300, 300);
	intake = 0;
	shoot();
	delay(200);
	intake = -127;
	PID_Turn(23,2,0,1,300,10);
	delay(500);
	pidDrive_complex(-2900,-40,0.04,0,2,0.03,1,300,500);
	pidDrive_complex(-350,-15,1,0,1.5,0.5,1,300,500);
	delay(500);
	pidDrive_complex(3050,40,0.09,0,2,0.07,1,300,500);
	delay(200);
	PID_Turn(-19,2,0,1,300,10);
	delay(400);
	shoot();
	intake = -127;
	pidDrive_complex(-2150, -30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	intake = 0;
	PID_Turn(-90,1,0.1,1.5,300,10);
	delay(200);
	intake = 90;
	pidDrive(-600, 0.2, 0, 1, 1, 300, 300);
	delay(100);
	intake = 0;
	pidDrive(1000, 0.2, 0, 1, 1, 300, 300);
	delay(250);
	PID_Turn(90,1,0.1,1.5,300,10);
	intake = 90;
	pidDrive(-1200, 0.2, 0, 1, 1, 300, 300);
	delay(100);
	intake = 0;
	pidDrive(1100, 0.2, 0, 1, 1, 300, 300);
	PID_Turn(133,0.94,1.1,1,500,10);
	pidDrive(1100, 0.2, 0, 1, 1, 300, 300);

	/*delay(200);
	PID_Turn(-87,1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-400, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	pidDrive(500,0.5,1,1,1,300,500);
	delay(300);
	PID_Turn(-135,0.92,1.1,1,500,10);
	delay(300);
	pidDrive(-300, 0.1, 0, 1, 1, 300, 300);
	intake = -110;
	pidDrive(-1200,0.17,0,2,1,300,500);
	delay(400);
	PID_Turn(-45,1.2,0,1,300,10);
	intake = 0;
	delay(200);
	pidDrive(-300, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	delay(120);
	pidDrive(500,0.55,0,1,1,300,500);
	delay(200);
	PID_Turn(137,0.92,1.1,1,300,10);
	delay(200);*/
	//launch string
	launcherA.set_value(true);
	delay(500);
	launcherA.set_value(false);
	delay(500);
	launcherA.set_value(true);
}

void skill_auto_ver3(){
	pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	intake = 90;
	delay(250);
	intake = 0;
	pidDrive(90,0.5,1,1,1,300,500);
	delay(300);
	PID_Turn(135,0.94,1.1,1,500,10);
	intake = -127;
	pidDrive(-1200,0.17,0,2,1,300,500);
	delay(200);
	PID_Turn(-45,1.3,0,1,300,10);
	intake = 0;
	delay(200);
	pidDrive(-590, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	//shoot three disc
	pidDrive(60,0.7,0,1,1,200,90);
	delay(500);
	PID_Turn(-89.5,1,0.1,1.5,300,10);
	delay(500);	
	pidDrive_complex(2820, 30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	delay(200);
	PID_Turn(8,3.2,0.5,3,300,5);
	delay(300);
	shoot();
	Task catatask(cata_auto);
	PID_Turn(-98, 1,0.1,1.5,300,10);
	delay(200);
	intake = -127;
	pidDrive_complex(-1650,-20,0.1,0,1,0.1,1,300,500);
	delay(300);
	pidDrive_complex(1560,20,0.1,0,1,0.1,1,300,500);
	delay(200);
	PID_Turn(99, 1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-50, 0.8, 1, 1, 1, 100, 10);
	delay(300);
	shoot();
	PID_Turn(-49,1.3,0,1,300,10);
	intake = -127;
	delay(200);
	pidDrive_complex(-1600,-20,0.2,0,2,0.15,1,300,500);
	delay(200);
	PID_Turn(-90,1,0.1,1.5,300,10);
	delay(200);
	pidDrive_complex(-1500,-20,0.2,0,2,0.15,1,300,500);
	delay(300);
	PID_Turn(90,1,0.1,1.5,300,10);
	delay(300);
	pidDrive(550, 0.15, 0, 1, 1, 300, 300);
	intake = 0;
	shoot();
	pidDrive(-340, 0.15, 0, 1, 1, 300, 300);
	delay(300);
	PID_Turn(-136,0.94,1.1,1,500,10);
	intake = -127;
	delay(200);
	pidDrive_complex(-2270,-20,0.1,0,2,0.08,1,300,500);
	delay(200);
	PID_Turn(87,1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-60,0.7,0,1,1,200,90);
	delay(300);
	intake = 0;
	shoot();
	PID_Turn(45,1.3,0,1,300,10);
	delay(200);intake = -127;
	pidDrive_complex(-2000,-20,0.02,0,2,0.02,1,300,500);
	delay(300);
	pidDrive_complex(-670,-15,0.3,0,1.5,0.1,1,300,500);
	pidDrive_complex(1950,20,0.08,0,2,0.06,1,300,500);
	delay(300);
	PID_Turn(-39,1.3,0,1,300,10);
	delay(200);
	pidDrive(-230, 0.15, 0, 1, 1, 300, 300);
	intake = 0;
	shoot();
	pidDrive_complex(-1700, -30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	delay(250);
	PID_Turn(-90,1,0.1,1.5,300,10);
	intake = 90;
	pidDrive(-300, 0.15, 0, 1, 1, 300, 300);
	delay(250);
	intake = 0;
	pidDrive(400, 0.2, 0, 1, 1, 300, 300);
	PID_Turn(-90,1,0.1,1.5,300,10);
	PID_Turn(-90,1,0.1,1.5,300,10);
	intake = -127;
	pidDrive(-1100, 0.04, 0, 1, 1, 300, 300);
	pidDrive_complex(-620,-15,0.3,0,1.5,0.1,1,300,500);
	pidDrive(90,0.5,1,1,1,300,500);
	delay(200);
	PID_Turn(-90,1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-1330, 0.2, 0, 1, 1, 300, 300);
	delay(200);
	intake = 0;
	pidDrive(400, 0.2, 0, 1, 1, 300, 300);
	PID_Turn(-86,1,0.1,1.5,300,10);
	delay(200);
	pidDrive_complex(2100, 30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	delay(250);
	shoot();
	pidDrive_complex(-2500, -30, 0.18, 0, 1.5, 0.12,1.3,200, 2000);
	PID_Turn(-133,0.94,1.1,1,500,10);

	/*delay(200);
	PID_Turn(-87,1,0.1,1.5,300,10);
	delay(200);
	pidDrive(-400, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	pidDrive(500,0.5,1,1,1,300,500);
	delay(300);
	PID_Turn(-135,0.92,1.1,1,500,10);
	delay(300);
	pidDrive(-300, 0.1, 0, 1, 1, 300, 300);
	intake = -110;
	pidDrive(-1200,0.17,0,2,1,300,500);
	delay(400);
	PID_Turn(-45,1.2,0,1,300,10);
	intake = 0;
	delay(200);
	pidDrive(-300, 0.1, 0, 1, 1, 300, 300);
	intake = 90;
	delay(250);
	intake = 0;
	delay(120);
	pidDrive(500,0.55,0,1,1,300,500);
	delay(200);
	PID_Turn(137,0.92,1.1,1,300,10);
	delay(200);*/
	//launch string
	launcherA.set_value(true);
	delay(500);
	launcherA.set_value(false);
	delay(500);
	launcherA.set_value(true);
}
void autonomous(){
	//skill_auto_ver2();
	//skill_auto_ver3();
	//match_auto_left();
	match_auto_right();
	//match_auto_left();


	pros::screen::print(TEXT_MEDIUM, 3,"Time %f",millis());
	while(true){
		pros::screen::print(TEXT_MEDIUM, 1,"Current rotation %f",gyro.get_rotation());
		pros::screen::print(TEXT_MEDIUM, 2,"Current position %f",rightFrontMotor.get_position());
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

 //blue.x_middle_coord;


/*
// ODOM FUNCTIONS
double flywheel_distancePower(int position[2]){
  int a = position[0]*position[0];
  int b = position[1]*position[1];
  int dist = sqrt(a+b);
  int min;
  double kp;
  double ki;
  double power;
  power = dist*kp;
  if (power < min){
    ki = 0.5;
    power += dist*ki;
  }
  return power;
}
*/
/*void odomPID_skill(int position[120]){
  double currentPosition[2];
  double target_x;
  double target_y;
  double target_dist;
  double angle;
  for(int i; i < 120; i+=2){
    double *pointer = odomPosition();
    currentPosition[0] = pointer[0];
    currentPosition[1] = pointer[1];
    target_x = position[i];
    target_y = position[i+1];
    target_dist = sqrt(((target_x - currentPosition[0])*(target_x - currentPosition[0]))+((target_y - currentPosition[1])*(target_y - currentPosition[1])));
    angle = acos((((target_x - currentPosition[0])*(target_x - currentPosition[0]))+((target_y - currentPosition[1])*(target_y - currentPosition[1])) - target_dist
  )/(2*(target_x - currentPosition[0])*(target_y - currentPosition[1])));
  pidTurn(angle, 50);
  pidDrive(target_dist, 50);
  }
}
*/
//AUTO FUNCTIONS
/*void auto_shoot(){
  double position[2];
  double *pointer = odomPosition();
  position[0] = pointer[0];
  position[1] = pointer[1];
  int a = position[0]*position[0];
  int b = position[1]*position[1];
  int dist = sqrt(a+b);
  int min;
  double kp;
  double ki;
  double power;
  power = dist*kp;
  if (power < min){
    ki = 0.5;
    power += dist*ki;
  }
  flywheelL = power;
  flywheelR = -power;
}
*/

void opcontrol() {
	double x;
	double y;
	double p;
	double w;
	double past_p;
	double start_time= 0;
	double cata_state = 0;
	double cata_time = 0;
	bool first = true;
	while(true){
		y = master.get_analog(ANALOG_LEFT_Y);
		x = master.get_analog(ANALOG_RIGHT_X);
		rightFrontMotor = y + -x;
		rightBackMotor = y + -x;
		rightTopMotor = -y + x;
		leftBackMotor = -y + -x;
		leftFrontMotor = -y + -x;
		leftTopMotor = y + x;
		//launch
		if(master.get_digital(DIGITAL_R2) == 1){
			catapult = 127;
			cata_time = c::millis();
			delay(500);
			cata_state = 0;
			first = false;
		}
		//stop motor when the limit switch is pressed (to keep the basket at standard position)
		else if(cata.get_value() == 1){
			catapult = 0;
			cata_state = 1;
		}
		//restore after launching
		else if(cata_state == 0 && first == false){
				catapult = 110;
		}
		//pros::screen::print(TEXT_MEDIUM, 1,"Cata: %f",cata_state);
		//pros::screen::print(TEXT_MEDIUM, 2,"Catatemp: %f",catapult.get_temperature());
		//intake
		if((master.get_digital(DIGITAL_L1) == 1) && (cata_state != 0 || first == true)){
			intake = 127;
		}
		//outtake
		else if((master.get_digital(DIGITAL_R1) == 1) && (cata_state != 0 || first == true)){
			intake = -120;
		}
		//pause 
		else{
			intake = 0;
		}
		if(master.get_digital(DIGITAL_A) == 1){
			cata_state = 1;
		}
		if(master.get_digital(DIGITAL_L2) == 1){
			intake = 115;
		}
		//end_game launch
		if (master.get_digital(DIGITAL_X) == 1){
			launcherA.set_value(true);
			delay(500);
		}
		if(master.get_digital(DIGITAL_UP) == 1){
			band_boost.set_value(true);
		}
		}
	delay(20);
	}