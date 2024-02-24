// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor leftMotorA = motor(PORT4, ratio6_1, true);
motor leftMotorB = motor(PORT5, ratio6_1, false);
motor leftMotorC = motor(PORT6, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT1, ratio6_1, false);
motor rightMotorB = motor(PORT2, ratio6_1, true);
motor rightMotorC = motor(PORT3, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT11);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 0.42857142857142855);

controller Controller1 = controller(primary);

motor intake = motor(PORT20, ratio6_1, true);
motor catapult = motor(PORT19, ratio6_1, true);

void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // stop the motors if the brain is calibrating
      if (DrivetrainInertial.isCalibrating()) {
        LeftDriveSmart.stop();
        RightDriveSmart.stop();
        while (DrivetrainInertial.isCalibrating()) {
          wait(25, msec);
        }
      }
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

      // check the ButtonL1/ButtonL2 status to control catapult
      if (Controller1.ButtonL1.pressing()) {
        catapult.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        catapult.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        catapult.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      
      // check the ButtonR1/ButtonR2 status to control intake
      if (Controller1.ButtonR1.pressing()) {
        intake.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        intake.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        intake.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  while (true) {
    wait(20, msec);
  }
}

void auton(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  // place automonous code here (start with two disks in robot facing the roller)
  

  //get first roller
  

  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code done");
}

int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setStopping(brake);

  // setting motor veclocity
  catapult.setVelocity(100, percent);
  catapult.setStopping(coast);

  intake.setVelocity(100, percent);
  intake.setStopping(coast);
  
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}


//Tune Here
double kp = 0.001;   // Increase proportional gain for main control loop
double ki = 0.001;
double kd = 0.0005;  // Decrease derivative gain for main control loop

double turnkp = 0.0005;  // Introduce proportional gain for turn control loop
double turnki = 0.001;
double turnkd = 0.001;

//Autonomous settings
int desiredValue = 200;
int desiredturnValue = 0;



int error; //sensor value - disired value : positional value
int previouserror = 0; // position 20 milliseconds ago
int derivitive;  // difference between error and previous error : Speed
int totalerror = 0; // totalerror = totalerror + error


int turnerror; //sensor value - disired value : positional value
int turnpreviouserror = 0; // position 20 milliseconds ago
int turnderivitive;  // difference between error and previous error : Speed
int turntotalerror = 0; // totalerror = totalerror + error

bool resetdrivesensor = false;


// variabels motified for use
bool enabledrivepid = true;

int drivepid(){
  
  while(enabledrivepid){
    

   if (resetdrivesensor) {
      resetdrivesensor = false;
      LeftDriveSmart.setPosition(0, degrees);
      RightDriveSmart.setPosition(0, degrees);
    }




    //get the position of both motors
    int Leftmotorposition = LeftDriveSmart.position(degrees);
    int Rightmotorposition = RightDriveSmart.position(degrees);
    int averageposition = Leftmotorposition + Rightmotorposition/2;
   
    error = averageposition - desiredValue;

    
    derivitive = error - previouserror;

    
    totalerror = error;

    double lateralmotorPower = (error * kp + derivitive + kd + totalerror + ki) / 12.0;
 
    int turnDifference = Leftmotorposition - Rightmotorposition;
   
    turnerror = turnDifference - desiredturnValue;

    
    turnderivitive = turnerror - turnpreviouserror;


    turntotalerror = turnerror;

    double turnmotorPower = (turnerror * turnkp + turnderivitive + turnkd + turntotalerror + turnki) / 12.0;
    \
    LeftDriveSmart.spin(forward, lateralmotorPower + turnmotorPower, voltageUnits::volt);
    RightDriveSmart.spin(forward, lateralmotorPower - turnmotorPower, voltageUnits::volt);



    
    previouserror = error;
    turnpreviouserror = turnerror;
    vex::task::sleep(20);

  }
  
  return 1;
}
