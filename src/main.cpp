/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       andyc                                                     */
/*    Created:      11/23/2024, 9:31:21 PM                                    */
/*    Description:  V5 project                                                */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <iostream>

using namespace vex;
using std::cout;
using std::endl;

// A global instance of competition
competition Competition;

// Brain
brain Brain;

// Controller
controller Controller1 = controller(primary);

// Drivetrain
motor L1 = motor(PORT16, true);
motor L2 = motor(PORT9, true);
motor L3 = motor(PORT5, false);
motor R1 = motor(PORT1, true);
motor R2 = motor(PORT8, false);
motor R3 = motor(PORT14, false);
double TurnSpeed = .55;

// Intake and Conveyer
motor Intake = motor(PORT13, ratio18_1, false);

// Lady Brown
motor LadyBrown1 = motor(PORT10, ratio18_1, false);
motor LadyBrown2 = motor(PORT20, ratio18_1, true);

// Rotation for Lady Brown
rotation RotationLB = rotation(PORT2, false);

// Goal
digital_out Goal = digital_out(Brain.ThreeWirePort.A);

// Doinker
digital_out Doinker = digital_out(Brain.ThreeWirePort.B);
  
// Optical
optical ColorSensor = optical(PORT15);

// Inertial
inertial Inertial = inertial(PORT12);

// Distance
distance Distance = distance(PORT3);

// Rotation
rotation Rotation = rotation(PORT4, true);

// GPS
gps GPS = gps(PORT20, -152, 0, distanceUnits::mm, -90, turnType::right);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void vexcodeInit(void);

void moveForward(double distanceWanted) {
  const double MAX_SPEED = 50;
  const double MIN_SPEED = 20;
  const double fP = 0.1;
  double error;
  double motorSpeed;

  Rotation.resetPosition();

  while (true) {
    double currentPosition = Rotation.position(degrees);
        
    error = distanceWanted - currentPosition; // Calculate error for forward movement

    // Break if within 5 degrees of the desired position
    if (fabs(error) <= 5.0) {
      break; 
    }

    motorSpeed = error * fP;

    double distanceRemaining = fabs(error);
    motorSpeed = fmax(MIN_SPEED, fmin(MAX_SPEED, motorSpeed));

    if (distanceRemaining < 100) {
      motorSpeed *= (distanceRemaining / 100);
    }

    motorSpeed = fmin(fmax(motorSpeed, -MAX_SPEED), MAX_SPEED);

    // Spin motors forward
    L1.spin(forward, motorSpeed, percent);
    L2.spin(forward, motorSpeed, percent);
    L3.spin(forward, motorSpeed, percent);
    R1.spin(forward, motorSpeed, percent);
    R2.spin(forward, motorSpeed, percent);
    R3.spin(forward, motorSpeed, percent);

    wait(20, msec);
  }

  // Stop all motors
  L1.stop(brake);
  L2.stop(brake);
  L3.stop(brake);
  R1.stop(brake);
  R2.stop(brake);
  R3.stop(brake);
}

void moveBackward(double distanceWanted) {
  double MAX_SPEED = 50;
  double MIN_SPEED = 0;
  double fP = 0.1;
  double error;
  double motorSpeed;

  Rotation.resetPosition();

  while (true) {
    double currentPosition = Rotation.position(degrees);
        
    error = distanceWanted + currentPosition; // Invert the error for backward movement

    // Break if within 5 degrees of the desired position
    if (fabs(error) <= 5.0) {
      break; 
    }

    motorSpeed = error * fP;

    double distanceRemaining = fabs(error);
    motorSpeed = fmax(MIN_SPEED, fmin(MAX_SPEED, motorSpeed));

    if (distanceRemaining < 100) {
      motorSpeed *= (distanceRemaining / 100);
    }

    motorSpeed = fmin(fmax(motorSpeed, -MAX_SPEED), MAX_SPEED);

    // Spin motors in reverse direction
    L1.spin(reverse, motorSpeed, percent);
    L2.spin(reverse, motorSpeed, percent);
    L3.spin(reverse, motorSpeed, percent);
    R1.spin(reverse, motorSpeed, percent);
    R2.spin(reverse, motorSpeed, percent);
    R3.spin(reverse, motorSpeed, percent);

    wait(20, msec);
  }

  // Stop all motors
  L1.stop(brake);
  L2.stop(brake);
  L3.stop(brake);
  R1.stop(brake);
  R2.stop(brake);
  R3.stop(brake);
}

void inertialTurn(double targetAngle) {
  const double pValue = 2;
  const double maxSpeed = 50.0;

  Inertial.resetHeading();
    
  double currentAngle = 0.0;
  double error = 0.0;
  double speed = 0.0;

  const double tolerance = 15.0;

  while (true) {
    currentAngle = Inertial.heading(degrees);
        
    error = targetAngle - currentAngle;

    if (fabs(error) < tolerance) {
      L1.stop();
      L2.stop();
      L3.stop();
      R1.stop();
      R2.stop();
      R3.stop();
      break;
    }

    speed = error * pValue;

    if (speed > maxSpeed) {
      speed = maxSpeed;
    } else if (speed < -maxSpeed) {
      speed = -maxSpeed;
    }

    L1.setVelocity(-speed, percent);
    L2.setVelocity(-speed, percent);
    L3.setVelocity(-speed, percent);
    R1.setVelocity(speed, percent); 
    R2.setVelocity(speed, percent);
    R3.setVelocity(speed, percent);

    L1.spin(forward);
    L2.spin(forward);
    L3.spin(forward);
    R1.spin(forward);
    R2.spin(forward);
    R3.spin(forward);

    wait(20, msec);
  }
}

const int position1 = 160; 
const int position2 = 142;
const int position3 = 10; 
const int position4 = -40;
int positionCounter = 0;

void moveArmTo(int targetPosition) {
  double aP = 2;
  while (true) {
    int currentAngle = RotationLB.position(degrees);
    int error = targetPosition - currentAngle;

    if (abs(error) < 5) {
      LadyBrown1.stop(brake);
      LadyBrown2.stop(brake);
      wait(10, msec);
      break;
    }

    double output = aP * error;

    if (output > 100) output = 100; 
    if (output < -100) output = -100; 

    LadyBrown1.spin(reverse, output, percent);
    LadyBrown2.spin(reverse, output, percent);

    wait(20, msec);
  }
}

void onButtonBPressed() {
    switch (positionCounter) {
        case 0:
            moveArmTo(position1);
            positionCounter++;
            break;
        case 1:
            moveArmTo(position2);
            positionCounter++;
            break;
        case 2:
            moveArmTo(position3);
            positionCounter++;
            break;
        case 3:
            moveArmTo(position2);
            positionCounter++;
            break;
        case 4:
            moveArmTo(position1);
            positionCounter = 0;
            break;
    }
}

void LadyBrown() {
  moveArmTo(position4);
  wait(1, sec);
}

void pre_auton(void) {
  Goal.set(false);
  Rotation.resetPosition();
  Inertial.resetHeading();
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  inertialTurn(-90);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

// Goal
bool airPress1 = false;
int counter1 = 0;
void down1() {
  if(counter1 == 0){
    Goal.set(airPress1);
     airPress1 = !airPress1;
  }
  counter1++;
}

// Doinker
bool airPress2 = false;
int counter2 = 0;
void down2() {
  if(counter2 == 0){
    Doinker.set(airPress2);
     airPress2 = !airPress2;
  }
  counter2++;
}

void usercontrol(void) {
  while (1) {
    // Drivetrain
    L1.spin(forward, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent) * TurnSpeed), percent);
    L2.spin(forward, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent) * TurnSpeed), percent);
    L3.spin(forward, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent) * TurnSpeed), percent);
    R1.spin(forward, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent) * TurnSpeed), percent);
    R2.spin(forward, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent) * TurnSpeed), percent);
    R3.spin(forward, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent) * TurnSpeed), percent);

    // Intake
    if (Controller1.ButtonR2.pressing()) {
      Intake.spin(forward, 100, percent);
    }
    else if (Controller1.ButtonR1.pressing()) {
      Intake.spin(reverse, 100, percent);
    }
    else {
      Intake.stop();
    }

    // Goal
    Controller1.ButtonL1.pressed(down1);
    counter1 = 0;

    // Doinker
    Controller1.ButtonA.pressed(down2);
    counter2 = 0;
    

    // Lady Brown
    if (Controller1.ButtonL2.pressing()) {
      onButtonBPressed();
      wait(200, msec); // Debounce delay to prevent multiple triggers
    }
  }
  wait(20, msec);
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
