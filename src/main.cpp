#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/vision.hpp"
#include "pros/gps.hpp"
#include "pros/device.hpp"
#include "pros/misc.hpp"
#include "pros/vision.hpp"
#include "pros/llemu.hpp"
#include "pros/gps.hpp"
#include "lemlib/api.hpp"
#include <vector>

#define VISION_PORT 12
#define RED_SIG_ID  1     // SIG_1
#define BLUE_SIG_ID 2 	  // SIG_2
#define MIN_AREA    500

pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(
  RED_SIG_ID,
  /* uMin */ 32717, /* uMax */ 32819, /* uMean */ 32768,
  /* vMin */ -51,   /* vMax */ 51,    /* vMean */ 0,
  /* range */ 2.5,  /* type */ 0
);

pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(
  BLUE_SIG_ID,
  /* uMin */ -51,   /* uMax */ 51,    /* uMean */ 0,
  /* vMin */ 32717, /* vMax */ 32819, /* vMean */ 32768,
  /* range */ 2.5,  /* type */ 0
);
/**
 * A robot configration code
 */

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Drivetrain MortorGroup
pros::MotorGroup leftMotors({1,2});
pros::MotorGroup rightMotors({19,20});
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

//Intake MotorGroup
pros::Motor intake(11);

pros::Vision vision_sensor (VISION_PORT);
pros::Gps gps(12);


lemlib::Chassis chassis;

struct GameObject{
	int x;
  	int y;
  	GameObject(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
};

struct Goal : public GameObject {
	int id;
  	Goal(int x_, int y_, int id_) : GameObject(x_, y_), id(id_) {} 
};

struct Loader : public GameObject {
	int id;
	Loader(int x_, int y_, int id_ = 0) : GameObject(x_, y_), id(id_) {}
};

std::vector<Goal> GOALS = {
  	Goal(-620, 1200, 1),
  	Goal(-620,-1200, 2),
  	Goal( 620, 1200, 3),
  	Goal( 620,-1200, 4),
  	Goal(-205,  200, 5),
  	Goal(-205, -200, 6),
  	Goal( 205,  200, 7),
  	Goal( 205, -200, 8),
};

std::vector<GameObject> GameObjects = {
	GameObject( 0,  1200),
	GameObject( 0, -1200),
	GameObject(-600, 600),
	GameObject(-600,-600),
	GameObject( 600,-600),
	GameObject( 600, 600),
	GameObject(-1700,  0),
	GameObject( 1700,  0)
};

std::vector<Loader> LOADERS = {
	Loader(-1725,  1200, 1),
	Loader(-1725, -1200, 2),
	Loader( 1725,  1200, 3),
	Loader( 1725, -1200, 4),
};

static inline const char* detect_color_once() {
  // 0 = 가장 큰 객체
  pros::vision_object_s_t r = vision_sensor.get_by_sig(0, RED_SIG_ID);
  pros::vision_object_s_t b = vision_sensor.get_by_sig(0, BLUE_SIG_ID);

  int r_area = (r.signature == RED_SIG_ID) ? r.width * r.height : 0;
  int b_area = (b.signature == BLUE_SIG_ID) ? b.width * b.height : 0;

  if (r_area < MIN_AREA) r_area = 0;
  if (b_area < MIN_AREA) b_area = 0;

  if (r_area == 0 && b_area == 0) return "NONE";
  return (r_area > b_area) ? "RED" : "BLUE";
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	GameObject object(0,0);
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	std::cout << "--------GPS-------" << "\n"<< "X: " << gps.get_position_x() << "\n" <<"Y: " << gps.get_position_y() << "\n";
	pros::lcd::print(2,"X: %f \nY: %f", gps.get_position_x(), gps.get_position_y());
	//Vision Sensor Initialize
  	vision_sensor.set_signature(RED_SIG_ID,  &RED_SIG);
  	vision_sensor.set_signature(BLUE_SIG_ID, &BLUE_SIG);
}

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
void autonomous() {
	
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
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({-1, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-19, -20});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move_velocity(127);
			pros::delay(10);
		}
		else{
			intake.move_velocity(0);
		}
		const char* c = detect_color_once();
		pros::lcd::print(0, "Detect: %s", c);
		//pros::delay(100);

		pros::delay(20);// Run for 20 ms then update
	}
}
