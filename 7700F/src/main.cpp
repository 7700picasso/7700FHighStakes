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
digital_out clamp (Brain.ThreeWirePort.A); 
inertial Gyro (PORT15);  
// A global instance of competition
float pi = 3.14;
float dia = 3.25;
float gearRatio = 1.6;

int AutonSelected = 4;
int AutonMin = 0;
int AutonMax = 4;

//competition Competition;

void selectAuton() {
		bool selectingAuton = true;
		
		int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
		int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen
		
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

void gyroTurnwithP(float target) {
  Gyro.setRotation(0.0,deg);
  float accuracy = 1.5;
  float Kp = 0.75;
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
					//15 second auton
					break;
				
				case 1:
					//code 1
					break;
				
				case 2:
					//code 2
					break;
				
				case 3:
					//code 3
					break;
        
        case 4:
		     //skills auton       
  con.spin(reverse, 60, pct);
  wait(1500,msec);
  inchDriveP(13);
  wait(1000, msec);
  gyroTurnwithP(-90);
  clamp.set(true);
  inchDriveP(-22);
  clamp.set(false);
  gyroTurnwithP(175);
  intake.spin(reverse, 89, pct);
  wait(1000,msec);
  inchDriveP(18);
  wait(1000,msec);
  gyroTurnwithP(-3);
  inchDriveP(9);
  wait(1500,msec);
  gyroTurnwithP(90);
  inchDriveP(5);
  wait(1500,msec);
  inchDriveP(-10);
  gyroTurnwithP(120);
  inchDriveP(-7);
  clamp.set(true);
  inchDriveP(48);
  gyroTurnwithP(180);
  inchDriveP(-7);
  clamp.set(false);
  

          break; }

// kieran please set the robot and the field correctly >:(

  


  

   
   
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
  while (1) {

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
if(Controller1.ButtonL1.pressing())
{
 
  clamp.set(true);
}
else if(Controller1.ButtonL2.pressing())
{
  clamp.set(false);
}


motor;

if (Controller1.ButtonR1.pressing())
{
  intake.spin(forward, 89, pct);
  con.spin(forward, 57, pct);
}
else if(Controller1.ButtonR2.pressing())
{  
  intake.spin(reverse, 89, pct);
  con.spin(reverse, 57, pct);


}
else {
intake.stop(); 
con.stop();
}


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
