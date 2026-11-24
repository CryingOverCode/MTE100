#pragma region VEXcode Generated Robot Configuration

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();
motor HorizM = motor(PORT1, false);
motor PitchM = motor(PORT2, false);
motor leftMotor = motor(PORT3, false);
motor rightMotor = motor(PORT4, true);
distance Dist = distance(PORT6);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//
//    Module:       main.cpp
//    Author:       {author}
//    Created:      {date}
//    Description:  IQ project
//
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"
#include <string>

// Allows for easier use of the VEX Library
using namespace vex;

// States ------
//did we see enums in class? prob not lmfao 
enum struct State {
  Idle,
  Teleop,
  Scan2D,
  Scan3D,
  Calibration,
};
enum struct ScanState {
  Positioning,
  Sweeping,
};

struct robot_ctx {
  float x; //in mm
  float y; // in mm
  float heading;//which direction is the robot facing in degrees 
};


//Constants ------ 
const float PI_F = 3.14159265f;
const float DEG_TO_RAD = PI_F / 180.0f;

const float WHEEL_DIAMETER_MM      = 200.0f;
const float DRIVE_WHEEL_PER_MOTOR  = 1.0f;   
const float WHEEL_DIST_MM         = 200.0f; //measure wheel distance between left and right wheels in mm 
const float MM_PER_MOTOR_DEG       = PI_F * WHEEL_DIAMETER_MM / 360.0f * DRIVE_WHEEL_PER_MOTOR;

//
const float DISTANCE_INTERVAL_MS = 1000.0 / 30; // 30 Hz max
const int BUF_SIZE = 64; // 64 readings * 6 bytes -> 384 bytes per buffer. ~ 2.13s of data before an SD card flush
const float HORIZ_GEAR_RATIO = 4;
const float PITCH_GEAR_RATIO = 30;

const float PITCH_2D_ANGLE = 0;
const float PITCH_3D_ANGLE = -90;

const float PITCH_STOP_ANGLE = -90.0f;
const float HORIZ_STOP_ANGLE = -140.0f;

const float SWEEP_MAX_ANGLE = 140.0f;
const float SWEEP_SPEED = 20.0f;  // motor speed 
const float PITCH_SPEED = 50.0f;
const float PITCH_ARRIVE_TOLERANCE_DEG = 5.0f; // what counts as pitch has "arrived" to tune

//this is some experimentally determined offset from the 2d position, which is the proper (0,0,0). 
const float kx = 0; 
const float ky = 0;
const float kz = 0;


std::string c_file_name;
bool sd_ok = false;              

robot_ctx robot_context{0,0,0.0};
State robot_state = State::Idle;
bool calibrated = false;

double prev_left_deg = 0;
double prev_right_deg = 0;

//note: int16 allows us to store about +- 30 m of max distance which should be chill for now
struct Point { // 6 bytes per reading
  int16_t x_mm;
  int16_t y_mm;
  int16_t z_mm; // stays 0 in 2d
};

Point buffer[BUF_SIZE];
int buffer_count = 0;
ScanState scan_state = ScanState::Positioning;
bool sweep_positive = true; //true = sweep right, false = sweep left
float pitch_target = 0;

//Author: Jijia
//trick to debounce a button press. I learned this from CRC lmao
static bool deBounce(bool& lastState, bool currentState) {
  if (currentState && !lastState) {
    lastState = true;
    return true;
  } else if (!currentState) {
    lastState = false;
  }
  return false;
}

//Author: Jijia
//convert floats to int16_t with clamping to avoid overflow
static int16_t floatTOint16(float f) { 
  if (f > 32767.0f) return 32767;
  if (f < -32768.0f) return -32768;
  return (int16_t)(f);
}
//Author: Jijia
static void stopAllMotors() {
  leftMotor.stop();
  rightMotor.stop();
  HorizM.stop();
  PitchM.stop();
}
//Author: Jijia
void resetOdometry() {
  leftMotor.resetPosition();
  rightMotor.resetPosition();
  prev_left_deg = 0;
  prev_right_deg = 0;
  BrainInertial.resetRotation();
  robot_context = {0, 0, 0.0f};
}
//Author: Jijia
void updateOdometry() {
  double l = leftMotor.position(degrees);
  double r = rightMotor.position(degrees);
  float dl = (float)(l - prev_left_deg)  * MM_PER_MOTOR_DEG;
  float dr = (float)(r - prev_right_deg) * MM_PER_MOTOR_DEG;
  prev_left_deg  = l;
  prev_right_deg = r;
 
  float d = (dl + dr) / 2.0f;
 
  //ill just use the inertial sensor for heading for now cuz im lazy to write kalman or do math
  float new_heading = (float)BrainInertial.rotation(degrees);


  float mid = (robot_context.heading + new_heading) * 0.5f * DEG_TO_RAD; // average heading over this step
  robot_context.x += d * sinf(mid);
  robot_context.y += d * cosf(mid);
  robot_context.heading = new_heading;
}
//Author: Jijia
void teleopDrive() {
  int fwd  = Controller.AxisD.position();//which joystick is which
  int turn = Controller.AxisA.position();
  //joystick deadzone
  if (abs(fwd)  < 5) fwd  = 0;
  if (abs(turn) < 5) turn = 0;
 
  if (fwd == 0 && turn == 0) {
    leftMotor.stop(brake);
    rightMotor.stop(brake);
    return;
  }
 
  int left  = fwd + turn;  
  int right = fwd - turn;
  //clamping 
  if (left  >  100) left  =  100;
  if (left  < -100) left  = -100;
  if (right >  100) right =  100;
  if (right < -100) right = -100;
 
  leftMotor.spin(forward, left, percent);
  rightMotor.spin(forward, right, percent);
}

//Author: Aiden
//Assumptions: start of maze is (0,0). Theta is relative to direclty forward of the path. Left-right of init. is x-axis. Front-back of init. is y-axis.
void find2D(float DistTheta, float Dist, Point& output) {
  float actualHeading = (DistTheta + robot_context.heading) * DEG_TO_RAD; // convert heading to radians
	output.x_mm =  floatTOint16(robot_context.x + (sinf(actualHeading)*Dist));  
	output.y_mm = floatTOint16(robot_context.y + (cosf(actualHeading)*Dist)); 
  output.z_mm = 0; //set this just to write smt
}
//Assumptions: start of maze is (0,0,0). Directly downwards is 0 degrees and directly forwards is 0 degrees. Left-right of init. is x-axis. Front-back is y-axis. Up-down is z-axis.
void find3D(float DistTheta, float Dist, Point& output) { //theta2 and theta3 named for 2d and 3d respectively.
	output.x_mm = floatTOint16(kx + robot_context.x + (Dist*sinf(DistTheta * DEG_TO_RAD)*cosf(robot_context.heading * DEG_TO_RAD))); //scanning along the x plane.
	output.y_mm = floatTOint16(ky + robot_context.y + (Dist*sinf(DistTheta * DEG_TO_RAD)*sinf(robot_context.heading * DEG_TO_RAD))); //scanning along the y plane.
	output.z_mm = floatTOint16(kz -1*(Dist*cosf(DistTheta * DEG_TO_RAD ))); //*-1 since facing down is positive. Maybe double check the signs and how much each BotCoord is required, esp. z
}

//Author: Safeer
void sensorCalibration()
{
    robot_state = State::Calibration;
    timer timer1;
    // CALIBRATE TILT TO 0 DEGREES
    PitchM.spin(reverse, 10, percent);//check if reverse is moving up
    timer1.clear();
    int tiltStall = 0;

    while (timer1.time(msec) < 3000 && tiltStall < 5)
    {
        double oldPosition = PitchM.position(degrees);

        wait(100, msec);

        double newPosition = PitchM.position(degrees);
        double motorCurrent = PitchM.current(amp);

        double change = newPosition - oldPosition;

        if (change < 1 && change > -1 &&
            motorCurrent > 0.5)
        {
            tiltStall++;
        }
        else
        {
            tiltStall = 0;
        }
    }

    PitchM.stop(brake);

    if (tiltStall >= 5)
    {
        PitchM.setPosition(0, degrees); 
    }
    else
    {
        Brain.Screen.print("Tilt calibration failed");
        return;
    }


    // CALIBRATE SPIN TO -140 DEGREES

    HorizM.spin(reverse, 10, percent);

    timer1.clear();
    int spinStall = 0;

    while (timer1.time(msec) < 3000 && spinStall < 5)
    {
        double oldPosition = HorizM.position(degrees);

        wait(100, msec);

        double newPosition = HorizM.position(degrees);
        double motorCurrent = HorizM.current(amp);

        double change = newPosition - oldPosition;

        if (change < 1 && change > -1 &&
            motorCurrent > 0.5)
        {
            spinStall++;
        }
        else
        {
            spinStall = 0;
        }
    }

    HorizM.stop(brake);

    if (spinStall >= 5)
    {
        HorizM.setPosition(-HORIZ_STOP_ANGLE * HORIZ_GEAR_RATIO, degrees);
    }
    else
    {
        Brain.Screen.print("Spin calibration failed");
        return;
    }
    // CALIBRATE INERTIAL SENSOR

    BrainInertial.resetRotation();
    BrainInertial.calibrate();

    while (BrainInertial.isCalibrating())
    {
    }
    //move sensor to level after cali
    pitchTo(PITCH_2D_ANGLE);                                           
    HorizM.spinToPosition(0, degrees, 50, percent, true);    
    calibrated = true;
    Brain.Screen.print("Calibration complete");
    robot_state = State::Idle;
}

//Author:  SAFEER: Initialize teleop mode
bool initTeleopMode()
{
    // Check that controller is connected
    bool controllerReady = Controller.installed();
 
    // Check that required motors are online
    bool motorsReady = leftMotor.installed() && rightMotor.installed() && HorizM.installed() && PitchM.installed();
 
    // Check that required sensors are online
    bool sensorsReady = Dist.installed() && BrainInertial.installed();
 
    // Only allow teleop if everything is ready
    if (controllerReady && motorsReady && sensorsReady)
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Teleop Ready!");
 
        return true;
    }
    else //w checks
    {
        // Stop all motors
        stopAllMotors();
 
        // Display error message
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("SYSTEM ERROR!");
 
        if (!controllerReady)
        {
            Brain.Screen.newLine();
            Brain.Screen.print("Controller offline");
        }
 
        if (!motorsReady)
        {
            Brain.Screen.newLine();
            Brain.Screen.print("Motor offline");
        }
 
        if (!sensorsReady)
        {
            Brain.Screen.newLine();
            Brain.Screen.print("Sensor offline");
        }
        return false;
    }
}

//Author: Jijia
void enterIdle() {
  bool was_scanning = (robot_state == State::Scan2D || robot_state == State::Scan3D);
  stopAllMotors();                    
  if (was_scanning) flushToSD();      
 
  robot_state = State::Idle;
}

//Author: Jijia
//we need to be able to enter from idle to teleop
void armRobot() {
  if (!calibrated) {                 
    sensorCalibration();
    if (!calibrated) return;
  }
  if (initTeleopMode()) {
    robot_state = State::Teleop;
  }
}

//Author: Simon
bool initSD() {
  const char* header = (robot_state == State::Scan3D) ? "x_mm,y_mm,z_mm\n" : "x_mm,y_mm\n";
  printf("%s", header);

  if (!Brain.SDcard.isInserted()) {
    Brain.Screen.print("No SD card found");
    return false;
  }
  std::string base = "Point_Scan_V";
  std::string type = ".csv";
  int n = 1;
  std::string current_name = base + std::to_string(n) + type;
  while (Brain.SDcard.exists(current_name.c_str())) {
    n++;
    current_name = base + std::to_string(n) + type;
  }

  scan_context.c_file_name = current_name;

  Brain.SDcard.savefile(scan_context.c_file_name.c_str(), (uint8_t*)header, strlen(header));
  return true;
}

//Author: Simon
void flushToSD() {//might need to change to write everything at once instead of line by line.
  for (int i = 0; i < buffer_count; i++) {
    Point p = buffer[i];
    char line[24];   // longest is "-32768,-32768,-32768\n" = 21 chars
    int len;
    if (robot_state == State::Scan3D){
      len = snprintf(line, sizeof(line), "%d,%d,%d\n", p.x_mm, p.y_mm, p.z_mm);
    } else{
      len = snprintf(line, sizeof(line), "%d,%d\n", p.x_mm, p.y_mm);
    }
    printf("%s", line); 
    if (sd_ok){
      Brain.SDcard.appendfile(c_file_name.c_str(), (uint8_t*)line, len);//this calls sd card many times which might need to be changed, we want as little write operations as possible. 
    }
    if (n > 0 && n < room) len += n; //always fits with this sizing
  }

  printf("%s", flush_text);
  fflush(stdout);
  if (scan_context.sd_ok){
    Brain.SDcard.appendfile(scan_context.c_file_name.c_str(), (uint8_t*)flush_text, len);
  }
  scan_context.buffer_count = 0;
}
//Author: Simon
void bufferWrite(const Point& p) {
  buffer[buffer_count] = p;

  if (++buffer_count >= BUF_SIZE) {
    flushToSD();
  }
}

//Author: Simon
void pitchTo(float angle){
  PitchM.spinToPosition(angle * PITCH_GEAR_RATIO, degrees, PITCH_SPEED, rpm, false);
}
//Author: Simon
void scan(float maximum_angle, float speed){
  switch(scan_state) {
    case ScanState::Positioning:
      //we still moving to the starting position
      if (fabsf((float)PitchM.position(degrees) - pitch_target * PITCH_GEAR_RATIO) <= PITCH_ARRIVE_TOLERANCE_DEG) {
        scan_state = ScanState::Sweeping;
      }
      break;
    case ScanState::Sweeping:
      //we are sweeping
      static timer distance_reading_countdown;

      float limit = maximum_angle * HORIZ_GEAR_RATIO;
      float pos = (float)HorizM.position(degrees);

      if (sweep_positive && pos >= limit) {
        sweep_positive = false;
      } else if (!sweep_positive && pos <= -limit) {
        sweep_positive = true;
      }

      HorizM.spin(sweep_positive ? forward : reverse, speed, rpm);

      if (distance_reading_countdown.time(msec) >= DISTANCE_INTERVAL_MS){
        distance_reading_countdown.reset();
        if (Dist.isObjectDetected()){
          Point p = {};
          float theta = HorizM.position(degrees) / HORIZ_GEAR_RATIO;
          if (State::Scan2D == robot_state){
            find2D(theta, Dist.objectDistance(mm), p);
          } else if (State::Scan3D == robot_state){
            find3D(theta, Dist.objectDistance(mm), p);
          }
          bufferWrite(p);
        }
      }
      break;
  }
  PitchM.stop();
  HorizM.stop();
}
//Author: Jija
void startScan(State s) {
  robot_state = s; //should only be Scan2d or Scan3d
  initSD(); //every time we start a scan we want to create a new file to write to.
  buffer_count = 0;

  pitch_target = (s == State::Scan2D) ? PITCH_2D_ANGLE : PITCH_3D_ANGLE;
  pitchTo(pitch_target);
  sweep_positive = true;
  scan_state = ScanState::Positioning;
}
//Author: Jijia
void stopScan() {
  HorizM.stop(brake);
  PitchM.stop(brake);
  flushToSD();
  robot_state = State::Teleop;
  pitchTo(PITCH_2D_ANGLE);//return to starting position for 2d scan cuz why not looks nice
  HorizM.spinToPosition(0, degrees, 50, percent, false);
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code
  PithcM.setStopping(hold);//found this in the vex docs kinda cool
  HorizM.setStopping(brake);
  resetOdometry();
  
  sensorCalibration();   
  bool prev_estop = false, prev_arm = false, prev_2d = false, prev_3d = false;

  while (!Controller.ButtonL3.pressing() && State::Idle) { // exit button
    bool estop = deBounce(prev_estop, Controller.ButtonB.pressing());
    bool arm = deBounce(prev_arm, Controller.ButtonA.pressing());
    bool scan2d = deBounce(prev_2d, Controller.ButtonX.pressing());
    bool scan3d = deBounce(prev_3d, Controller.ButtonY.pressing());

    updateOdometry();
    if (estop) {
      enterIdle();
    }
    else {
      switch (robot_state) {
        case State::Idle:
          if (arm) armRobot();
          break;
        case State::Calibration:
          sensorCalibration(); //if it reaches here it means we failed calibration so well try again
          break; 
        case State::Teleop:
          teleopDrive();
          if (scan2d) {
            robot_state = State::Scan2D;
            startScan(State::Scan2D);
            //start scanning
          }
          else if (scan3d) {
            robot_state = State::Scan3D;
            startScan(State::Scan3D);
            //start scanning
          }
          break;
        case State::Scan2D:
          teleopDrive();
          scan(SWEEP_MAX_ANGLE, SWEEP_SPEED);
          if (scan2d){
            stopScan();
          }
          break;
        case State::Scan3D:
          teleopDrive();
          scan(SWEEP_MAX_ANGLE, SWEEP_SPEED);
          if (scan3d){
            stopScan();
          }
          break;
      }
    }
  }

  stopAllMotors();
  Brain.programStop();
  return 0;
}
