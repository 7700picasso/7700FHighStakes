/*----------------------------------------------------------------------------*/
/*                                                                            */ 
/*    Module:       main.cpp                                                  */
/*    Author:       Team7700                                                  */
/*    Created:      10/14/2024, 5:27:25 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;
// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;
motor LF=motor(PORT4,ratio18_1,true);
motor LB=motor(PORT3,ratio18_1,true);
motor RF=motor(PORT11,ratio18_1,false);
motor RB=motor(PORT20,ratio18_1,false);
motor intake=motor(PORT5, ratio6_1, false);
motor con=motor(PORT1, ratio18_1, true);
motor arm=motor(PORT7, ratio18_1, true);
motor arm2=motor(PORT10,ratio18_1,false);
digital_out clamp (Brain.ThreeWirePort.A); 
inertial Gyro (PORT15);  
// A global instance of competition
float pi = 3.14;
float dia = 3.25;
float gearRatio = 1.6;
float armPositions[] = {0.0, 55, 530};
int currentPositionindex = 0;
float target = 0;

//SELECTING AUTON.
int AutonSelected = 1;
int AutonMin = 0;
int AutonMax = 4;

//Red Negative and Blue Positive is: #0
//Red Positive and Blue Negative is: #1

//competition Competition;

void selectAuton() {
		bool selectingAuton = true;
		
		int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
		int y = Brain.Screen.yPosition(); // get the y positiüon of last touch of the screen
		
		// check to see if buttons were pressed
		if (x >= 20 && x <= 120 && y >= 50 && y <= 150){ // select button pressed
				AutonSelected++;
				if (AutonSelected > AutonMax){
						AutonSelected = AutonMin; // rollover
				}
				Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
		}
		
		
		if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
				selectingAuton = false; // GO button pressed
				Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonSelected);
		}
		
		if (!selectingAuton) {
				Brain.Screen.setFillColor(green);
				Brain.Screen.drawCircle(300, 75, 25);
		} else {
				Brain.Screen.setFillColor(red);
				Brain.Screen.drawCircle(300, 75, 25);
		}
		
		wait(10, msec); // slow it down
		Brain.Screen.setFillColor(black);
}

void drawGUI() {
	// Draws 2 buttons to be used for selecting auto
	Brain.Screen.clearScreen();
	Brain.Screen.printAt(1, 40, "Select Auton then Press Go");
	Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
	Brain.Screen.setFillColor(red);
	Brain.Screen.drawRectangle(20, 50, 100, 100);
	Brain.Screen.drawCircle(300, 75, 25);
	Brain.Screen.printAt(25, 75, "Select");
	Brain.Screen.setFillColor(green);
	Brain.Screen.drawRectangle(170, 50, 100, 100);
	Brain.Screen.printAt(175, 75, "GO");
	Brain.Screen.setFillColor(black);
}

void drive(int lspeed, int rspeed, int wt){
  LF.spin(forward, lspeed, pct);
  LB.spin(forward, lspeed, pct);
  RF.spin(forward, rspeed, pct);
  RB.spin(forward, rspeed, pct);
  wait(wt, msec);
}
void driveBrake(){
  LF.stop(brake);
  RF.stop(brake);
  LB.stop(brake);
  RB.stop(brake);

}
void armbrake(){
  arm.stop(brake);
}



  void changeTarget(){
    currentPositionindex++;
    if(currentPositionindex > 3){
      currentPositionindex = 0;
    }
    target = armPositions[currentPositionindex];
  }
void armRotationcontrol(){
  float position = 0.0;
  float accuracy = 2.0;
  float error = target - position;
  float kp=0.3;
  float previousError = 0.0; //Store the previous error for derivative control
  float kd = 0.5; // Derivative Gain
  float speed=0;

//rotationSensor.resetPosition(); // strange

while (true) {
  target = armPositions[currentPositionindex]; //Keep updating target
  position = arm.position(deg);
  error = target - position;

//Speed term
float speed = kp * error;

  if (fabs(error) < accuracy){
    arm.stop(hold); 
    arm2.stop(hold); 
  }
else { 
  arm.spin(fwd, speed, percent); 
  arm2.spin(fwd, speed, percent); 
}
}
}
void gyroTurnwithP(float target) {
  Gyro.setRotation(0.0,deg);
  float accuracy = 1.5;
  float Kp = 0.65;
  float heading = Gyro.rotation();
  float  error = target - heading; 
  float speed=Kp*error;

  while (fabs(error) > accuracy) {
    speed = error * Kp;
    drive(speed*(error/error), -speed*(error/error), 10);
    heading = Gyro.rotation();
    error = target - heading;

  }
  driveBrake();
}




void inchDriveP(float target){
  Brain.Screen.printAt(20, 80, "inchDrive");
  LF.setPosition(0.0,rev);   //we are setting the senor to 0 rev
  float x =0.0;  //distance that robot travles
  float error = target - x;   //how far the robot is from the target
  float accuracy = 0.5 ; //its just to measure against 
  float kp=3.0;
    float speed =kp*error;
  while(fabs(error)>accuracy){
    drive (speed , speed, 10);
    x = LF.position(rev)*pi*dia*gearRatio;
    error = target-x;
    speed=kp*error;
  }

  driveBrake();
}

double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
if (curr < 1)
Brain.Screen.setFillColor(green);
else if (curr >= 1 && curr  <= 2.5)
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
if (temp < 45)
Brain.Screen.setFillColor(green);
else if (temp <= 50 && temp  >= 45)
// TRUE and TRUE --> True
// TRUE and FALSE --> False
// FALSE and FALSE --> False
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
Brain.Screen.setFillColor(transparent);
}

//Displays information on the brain
void Display()
{
double leftFrontCurr = LF.current(amp);
double leftFrontTemp = LF.temperature(celsius);
double leftBackCurr = LB.current(amp);
double leftBackTemp = LB.temperature(celsius);
double rightFrontCurr = RF.current(amp);
double rightFrontTemp = RF.temperature(celsius);
double rightBackCurr = RB.current(amp);
double rightBackTemp = RB.temperature(celsius);
double intakeFrontCurr = intake.current(amp);
double intakeFrontTemp = intake.temperature(celsius);
double conFrontCurr = con.current(amp);
double conFrontTemp = con.temperature(celsius);



if (LF.installed())
{
MotorDisplay(1, leftFrontCurr, leftFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 1, "LF");
}
else
Brain.Screen.printAt(5, YOFFSET + 1, "LF Problem");

if (LB.installed())
{
MotorDisplay(31, leftBackCurr, leftBackTemp);
Brain.Screen.printAt(300, YOFFSET + 31, "LB");
}
else
Brain.Screen.printAt(5, YOFFSET + 31, "LB Problem");

if (RF.installed())
{
MotorDisplay(61, rightFrontCurr, rightFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 61, "RF");
}
else
Brain.Screen.printAt(5, YOFFSET + 61, "RF Problem");

if (RB.installed())
{
MotorDisplay(91, rightBackCurr, rightBackTemp);
Brain.Screen.printAt(300, YOFFSET + 91, "RB");
}
else
Brain.Screen.printAt(5, YOFFSET + 91, "RB Problem");

if (intake.installed())
{
MotorDisplay(121, intakeFrontCurr, intakeFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 121, "intake");
}
else
Brain.Screen.printAt(5, YOFFSET + 121, "Intake Problem");


if (con.installed())
{
MotorDisplay(151, conFrontCurr, conFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 151, "Con");
}
else
Brain.Screen.printAt(5, YOFFSET + 151, "Con Problem");

}

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

   while(Gyro.isCalibrating())
   {
   wait(500, msec);
   }

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //clamp.set(true);
  //inchDriveP(-24);
  //gyroTurnwithP(90);
  //clamp.set(false);
  
  switch (AutonSelected) {
				case 0:
					//15 second auton red negative side (blue positive)
          clamp.set(true);
          inchDriveP(-26);
          clamp.set(false);
          con.spin(reverse, 68, pct);
          wait(1200,msec);
          gyroTurnwithP(60);
          intake.spin(reverse, 80, pct);
          inchDriveP(20);
          //gyroTurnwithP(-82);
          //inchDriveP(22);
          //wait(1000, msec);
          //clamp.set(true);
          gyroTurnwithP(-165);
           inchDriveP(45);
					break;
				
				case 1:
					//15 second auton red positive side (blue negative)
          clamp.set(true);
          inchDriveP(-26);
          clamp.set(false);
          con.spin(reverse, 60, pct);
          wait(1500,msec);
          gyroTurnwithP(-50);
          intake.spin(reverse, 65, pct);
          inchDriveP(20);
          wait(1000,msec);
          gyroTurnwithP(-165);
          inchDriveP(45);
					break;
				
				case 2:
					//code 2
					break;
				
				case 3:
					//code 3
					break;
        
        case 4:
		     //skills auton       
  
  // first ring onto aliance stake 
  con.spin(reverse, 25, pct);
  wait(700,msec);
  con.stop(); 

  //getting first mogo
  inchDriveP(16);
  wait(1000, msec);
  gyroTurnwithP(-87);
  clamp.set(true);
  inchDriveP(-22);
  clamp.set(false);
  gyroTurnwithP(165);
  intake.spin(reverse, 89, pct);
   con.spin(reverse, 60, pct);
  inchDriveP(22);
  wait(1000,msec);
  gyroTurnwithP(-9);
  inchDriveP(10);
  wait(1500,msec);

  //Places goal in corner
  intake.stop();
  gyroTurnwithP(-139);
 drive(-50, -50, 1000); 
 driveBrake(); 
clamp.set(true);
  wait(500, msec);
  con.spin(forward, 20, pct);
  wait(700,msec);
  con.stop(); 
 

 //Hits wall to straighten
  inchDriveP(13);
  gyroTurnwithP(132);
   drive(100, 100, 1500); 
 driveBrake(); 
 wait(1000, msec);

  //reverses to goal, clamps it.
  inchDriveP(-30);
  gyroTurnwithP(-10);
  inchDriveP(-20);
  drive(-50, -50, 1600);
  driveBrake();
clamp.set(false);
  gyroTurnwithP(105);
  inchDriveP(10);

  //starts to score
  intake.spin(reverse, 89, pct);
    con.spin(reverse, 60, pct);
    wait(1500, msec);
    con.stop();
    //goes to otherside of the field.
    gyroTurnwithP(195);
  inchDriveP(-15);
  //grabs mobile goal, places in corners
  clamp.set(true);
    gyroTurnwithP(-20);
  //goes across the field
    inchDriveP(129);
    gyroTurnwithP(45);
  //grabs the mobile goal 
  inchDriveP(-35);
  clamp.set(false);
  //turns around to start getting rings
  gyroTurnwithP(180);
  inchDriveP(45);
  intake.spin(reverse, 90, pct);
  con.spin(reverse, 60, pct);
  gyroTurnwithP(-145);
  inchDriveP(-30);
          break; }

  


  

   
   
 // drive(20, -20, 2000);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop


  Controller1.ButtonL2.pressed(changeTarget);
  thread  ArmThread (armRotationcontrol); 

  while (1) {

    Display(); 

    int lspeed=Controller1.Axis3.position(pct);
    int rspeed=Controller1.Axis2.position(pct);
    drive(lspeed, rspeed, 10);
    
    //use ButtonA and ButtonB to activate the Pneumatic 1
if(Controller1.ButtonA.pressing())
{
 
  clamp.set(true);
}
else if(Controller1.ButtonB.pressing())
{
  clamp.set(false);
}

motor;

if (Controller1.ButtonR1.pressing())
{
  intake.spin(forward, 89, pct);
  con.spin(forward, 75, pct);
}
else if(Controller1.ButtonR2.pressing())
{  
  intake.spin(reverse, 89, pct);
  con.spin(reverse, 75, pct);
}
//if (Controller1.ButtonL1.pressing())
//{
//  if (Controller1.ButtonR1.pressing())
//}
//else if(Controller1.ButtonL2.pressing())
//{
 // arm.spin(reverse, 60, pct);
//}
else {
intake.stop(); 
con.stop();
//arm.stop();
}

// if (Controller1.ButtonL1.pressing()){
// arm.spin(reverse, 89, pct);
// }
// else if(Controller1.ButtonL2.pressing()){
// arm.spin(forward, 89, pct);
// }
// else{
// arm.stop();
// }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources. 
  }

 
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
