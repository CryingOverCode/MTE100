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
motor rightMotor = motor(PORT1, true);
motor leftMotor = motor(PORT6, false);
motor PitchM = motor(PORT2, true);
distance Dist = distance(PORT9);
motor HorizM = motor(PORT5, false);


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

controller Controller = controller();

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
  
  State robot_state;
  bool calibrated;
  double prev_left_deg;
  double prev_right_deg;
};


//Constants ------
const float PI_F = 3.14159265f;
const float DEG_TO_RAD = PI_F / 180.0f;

const float WHEEL_DIAMETER_MM      = 63.66f; //200mm is the circumference of the iq wheel not the diameter
const float DRIVE_WHEEL_PER_MOTOR  = 1.0f;
const float MM_PER_MOTOR_DEG       = PI_F * WHEEL_DIAMETER_MM / 360.0f * DRIVE_WHEEL_PER_MOTOR;

//
const float DISTANCE_INTERVAL_MS = 1000.0 / 30; // 30 Hz max
const int BUF_SIZE = 64; // 64 readings * 8 bytes -> 512 bytes per buffer. ~ 2.13s of data before an SD card flush
const float HORIZ_GEAR_RATIO = 4;
const float PITCH_GEAR_RATIO = 30;

const float PITCH_2D_ANGLE = 0;
const float PITCH_3D_ANGLE = 90; //90 forward from the stop, pointing down


const float SWEEP_MAX_ANGLE = 137.0f; //kept 3 deg short of the 140 deg hard stop
const float SWEEP_SPEED = 100.0f;  // motor speed
const float PITCH_CAL_FAST = 80.0f; //rpm at the motor when the reading is far from the floor value
const float PITCH_CAL_SLOW = 5.0f; //rpm at the motor when its right near the floor value. 5 is about 1 deg a second at the sensor
const float PITCH_CAL_SLOW_ZONE_MM = 50.0f; //within this many mm of the floor value it starts slowing down
const int PITCH_CAL_CHECKS = 3; //readings in a row that have to agree before we call it found
const float PITCH_FLOOR_MM = 57.8f; //what the distance sensor reads when it points straight down at the floor
const float PITCH_SPEED = 50.0f;
const float RETURN_SPEED = 60.0f; //rpm, used when going back to centre
const float SLOW_SPEED = 10.0f; //speed for the last bit before a target so we dont overshoot
const float SLOW_ZONE_DEG = 60.0f; //how close (motor deg) before we slow down
const float PITCH_ARRIVE_TOLERANCE_DEG = 5.0f; // what counts as pitch has "arrived" to tune
const float CREEP_SPEED = 4.5f; //percent. bot creeps forward at this speed during a 3d scan
const float CREEP_STEER = 10.0f; //percent. how much a full push of the right stick speeds up one wheel and slows the other while creeping

//this is some experimentally determined offset from the 2d position, which is the proper (0,0,0).
const float kx = 0;
const float ky = 0;
const float kz = 0;
const float SENSOR_OFFSET_MM = 100.0f; //how far the sensor sits in front of the robots rear axle (center of rotation)

const int MAX_LINE_CHARS = 21; // longest is "-32768,-32768,-32768\n" = 21 chars

const float WALL_MIN_ANGLE = 60.0f; //only readings between these two angles count as looking at a side wall
const float WALL_MAX_ANGLE = 120.0f;
const float KP_WALL = 0.02f; //percent of steer per mm of difference between the two walls aka centering sensitivity
const float WALL_STEER_MAX = CREEP_SPEED - 0.4 * CREEP_SPEED; // kept under CREEP_SPEED so neither wheel goes backwards when trying to turn

const float SENSOR_DELAY_MS = 43.0f;
const int HIST_SIZE = 64; //at about 10ms a loop this is about 640ms of history
const int OLD_READING_FREQUENCY = 2; //ms. the main loop is 10ms so this saves one every loop

//note: int16 allows us to store about +- 30 m of max distance which should be chill for now
struct Point { // 8 bytes per reading
  int16_t x_mm;
  int16_t y_mm;
  int16_t z_mm; // stays 0 in 2d
  int16_t horiz_deg; // sweep angle of the reading. 0 is straight ahead, positive is right, negative is left. only saved in 2d
};

//everything the scan has to remember between loops. these were all globals before
struct scan_ctx {
  std::string c_file_name;
  bool sd_ok;
  bool is_3d;
  Point buffer[BUF_SIZE];
  int buffer_count;
  ScanState scan_state;
  bool sweep_positive; //true = sweep right, false = sweep left
  float pitch_target;
  timer distance_reading_countdown;

  float lateral_left_avg;
  float lateral_right_avg;

  float left_sum;
  float right_sum;
  int left_count;
  int right_count;
  float steer;

  float hist_deg[HIST_SIZE];
  int hist_timings[HIST_SIZE];
  int hist_count;
  int hist_previous_reading;
};

void sensorCalibration(robot_ctx& robot_context);
void flushToSD(scan_ctx& scan_context);
bool pitchTo(float angle);
bool horizTo(float angle);
bool pitchCalibration();
void returnToStart();

void encoder_history(scan_ctx& scan_context){
  if (Brain.Timer.time(msec) - scan_context.hist_previous_reading >= OLD_READING_FREQUENCY){
    int c_index = scan_context.hist_count++ % HIST_SIZE;
    scan_context.hist_deg[c_index] = HorizM.position(degrees) / HORIZ_GEAR_RATIO;
    scan_context.hist_timings[c_index] = Brain.Timer.time(msec);
    scan_context.hist_previous_reading = Brain.Timer.time(msec);
  }
}

float getCorrectAngle(int data_time, const scan_ctx& scan_context){
  int wanted_time = data_time - SENSOR_DELAY_MS;
  int smallest_dif = 9999;
  int wanted_index = 0;

  for (int i = 0; i < HIST_SIZE; i++){
    int dif = abs(scan_context.hist_timings[i] - wanted_time);
    if(dif < smallest_dif){
      smallest_dif = dif;
      wanted_index = i;
    }
  }

  return scan_context.hist_deg[wanted_index];
}

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
void resetOdometry(robot_ctx& robot_context) {
  leftMotor.resetPosition();
  rightMotor.resetPosition();
  robot_context.prev_left_deg = 0;
  robot_context.prev_right_deg = 0;
  BrainInertial.resetRotation();
  robot_context.x = 0;
  robot_context.y = 0;
  robot_context.heading = 0.0f;
}
//Author: Jijia
void updateOdometry(robot_ctx& robot_context) {
  double l = leftMotor.position(degrees);
  double r = rightMotor.position(degrees);
  float dl = (float)(l - robot_context.prev_left_deg)  * MM_PER_MOTOR_DEG;
  float dr = (float)(r - robot_context.prev_right_deg) * MM_PER_MOTOR_DEG;
  robot_context.prev_left_deg  = l;
  robot_context.prev_right_deg = r;

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
  int fwd  = Controller.AxisA.position();//A is left stick up/down, C is right stick left/right
  int turn = Controller.AxisC.position();

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

//Author: Simon
void creepDrive(const scan_ctx& scan_context) {
  if (scan_context.scan_state != ScanState::Sweeping) {
    leftMotor.stop(brake);
    rightMotor.stop(brake);
    return;
  }
  int turn = Controller.AxisC.position();
  if (abs(turn) < 5) turn = 0; //joystick deadzone

  float steer = turn * CREEP_STEER / 100.0f + scan_context.steer;
  leftMotor.spin(forward, CREEP_SPEED + steer, percent);
  rightMotor.spin(forward, CREEP_SPEED - steer, percent);
}

//Author: Aiden
//Assumptions: start of maze is (0,0). Theta is relative to direclty forward of the path. Left-right of init. is x-axis. Front-back of init. is y-axis.
void find2D(float DistTheta, float Dist, const robot_ctx& robot_context, Point& output) {
  float actualHeading = (DistTheta + robot_context.heading) * DEG_TO_RAD; // convert heading to radians
  //the odometry tracks the center of rotation, but the sensor is SENSOR_OFFSET_MM ahead of it.
  //when the bot turns the sensor swings around that center, so start from where the sensor actually is
  float sensorX = robot_context.x + SENSOR_OFFSET_MM * sinf(robot_context.heading * DEG_TO_RAD);
  float sensorY = robot_context.y + SENSOR_OFFSET_MM * cosf(robot_context.heading * DEG_TO_RAD);
    output.x_mm =  floatTOint16(sensorX + (sinf(actualHeading)*Dist));
    output.y_mm = floatTOint16(sensorY + (cosf(actualHeading)*Dist));
  output.z_mm = 0; //set this just to write smt
}
//Assumptions: start of maze is (0,0,0). Directly downwards is 0 degrees and directly forwards is 0 degrees. Left-right of init. is x-axis. Front-back is y-axis. Up-down is z-axis.
void find3D(float DistTheta, float Dist, const robot_ctx& robot_context, Point& output) { //theta2 and theta3 named for 2d and 3d respectively.
    //same idea as find2D, start from where the sensor actually is and not from the center of rotation
    float sensorX = robot_context.x + SENSOR_OFFSET_MM * sinf(robot_context.heading * DEG_TO_RAD);
    float sensorY = robot_context.y + SENSOR_OFFSET_MM * cosf(robot_context.heading * DEG_TO_RAD);
    output.x_mm = floatTOint16(kx + sensorX + (Dist*sinf(DistTheta * DEG_TO_RAD)*cosf(robot_context.heading * DEG_TO_RAD))); //scanning along the x plane.
    output.y_mm = floatTOint16(ky + sensorY - (Dist*sinf(DistTheta * DEG_TO_RAD)*sinf(robot_context.heading * DEG_TO_RAD))); //scanning along the y plane. minus because heading is clockwise from +y, so the bots right side is (cos h, -sin h)
    output.z_mm = floatTOint16(kz -1*(Dist*cosf(DistTheta * DEG_TO_RAD ))); //*-1 since facing down is positive. Maybe double check the signs and how much each BotCoord is required, esp. z
}

//Author: Safeer
void sensorCalibration(robot_ctx& robot_context)
{
    robot_context.robot_state = State::Calibration;
    timer timer1;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibrating...");

    HorizM.spin(reverse, 40, percent);
    wait(500, msec);//let it get moving before we look for a stall

    timer1.clear();
    int spinStall = 0;

    while (timer1.time(msec) < 12000 && spinStall < 5)
    {
        double oldPosition = HorizM.position(degrees);

        wait(100, msec);

        double newPosition = HorizM.position(degrees);

        double change = newPosition - oldPosition;

        if (change < 3 && change > -3)
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
        HorizM.setPosition(0, degrees);
    }
    else
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Spin calibration failed");
        return;
    }

    HorizM.spin(forward, 40, percent);
    wait(500, msec);

    timer1.clear();
    spinStall = 0;

    while (timer1.time(msec) < 12000 && spinStall < 5)
    {
        double oldPosition = HorizM.position(degrees);

        wait(100, msec);

        double newPosition = HorizM.position(degrees);

        double change = newPosition - oldPosition;

        if (change < 3 && change > -3)
        {
            spinStall++;
        }
        else
        {
            spinStall = 0;
        }
    }

    HorizM.stop(brake);

    double range = HorizM.position(degrees);//motor degrees from one stop to the other

    if (spinStall >= 5 && range >= 2 * SWEEP_MAX_ANGLE * HORIZ_GEAR_RATIO)
    {
        // we are sitting on the forward stop, so center is half the range back
        HorizM.setPosition(range / 2, degrees);
    }
    else
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Spin calibration failed");
        return;
    }
    // center the spin before tilting, so the sensor looks at the floor in front and not at the bot
    timer1.clear();
    while (!horizTo(0) && timer1.time(msec) < 8000)
    {
        wait(10, msec);
    }
    HorizM.stop(brake);

    // CALIBRATE TILT
    if (!pitchCalibration())
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Tilt calibration failed");
        return;
    }
    // CALIBRATE INERTIAL SENSOR

    BrainInertial.resetRotation();
    BrainInertial.calibrate();

    while (BrainInertial.isCalibrating())
    {
        wait(20, msec);
    }
    //move sensor to level after cali
    returnToStart();
    robot_context.calibrated = true;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Calibration complete");
    robot_context.robot_state = State::Idle;
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
void enterIdle(robot_ctx& robot_context, scan_ctx& scan_context) {
  bool was_scanning = (robot_context.robot_state == State::Scan2D || robot_context.robot_state == State::Scan3D);
  stopAllMotors();
  if (was_scanning) flushToSD(scan_context);

  robot_context.robot_state = State::Idle;
}

//Author: Jijia
//we need to be able to enter from idle to teleop
void armRobot(robot_ctx& robot_context) {
  if (!robot_context.calibrated) {
    sensorCalibration(robot_context);
    if (!robot_context.calibrated) return;
  }
  if (initTeleopMode()) {
    robot_context.robot_state = State::Teleop;
  }
}

//Author: Simon
bool initSD(scan_ctx& scan_context) {
  const char* header = (scan_context.is_3d) ? "x_mm,y_mm,z_mm\n" : "x_mm,y_mm,horiz_deg\n";
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
//combines the whole buffer into one string then writes it once
void flushToSD(scan_ctx& scan_context) {
  char flush_text[BUF_SIZE * MAX_LINE_CHARS];
  if (scan_context.buffer_count == 0) return; 

  int len = 0; // how many chars are in flush_text so far
  for (int i = 0; i < scan_context.buffer_count; i++) {
    Point p = scan_context.buffer[i];
    int room = (int)sizeof(flush_text) - len;
    int size;
    if (scan_context.is_3d){
      size = snprintf(flush_text + len, room, "%d,%d,%d\n", p.x_mm, p.y_mm, p.z_mm);
    } else{
      size = snprintf(flush_text + len, room, "%d,%d,%d\n", p.x_mm, p.y_mm, p.horiz_deg);
    }
    if (size > 0 && size < room) len += size; //always fits with this sizing
  }

  printf("%s", flush_text);
  fflush(stdout);
  if (scan_context.sd_ok){
    Brain.SDcard.appendfile(scan_context.c_file_name.c_str(), (uint8_t*)flush_text, len);
  }
  scan_context.buffer_count = 0;
}

//Author: Simon
void bufferWrite(const Point& p, scan_ctx& scan_context) {
  scan_context.buffer[scan_context.buffer_count] = p;

  if (++scan_context.buffer_count >= BUF_SIZE) {
    flushToSD(scan_context);
  }
}

//Author: Simon
// rewrote the spin_to_pos so that it moves the motor a tad little bit, checks and goes again. None blocking
static bool spinTo(motor& m, float target_deg, float speed) {
  float error = target_deg - (float)m.position(degrees);
  if (fabsf(error) <= PITCH_ARRIVE_TOLERANCE_DEG) {
    m.stop();
    return true;
  }
  if (fabsf(error) < SLOW_ZONE_DEG) speed = SLOW_SPEED; //slow down for the last bit
  m.spin(error > 0 ? forward : reverse, speed, rpm);
  return false;
}
//Author: Simon
bool pitchTo(float angle){
  return spinTo(PitchM, angle * PITCH_GEAR_RATIO, PITCH_SPEED);
}
//Author: Simon
bool horizTo(float angle){
  return spinTo(HorizM, angle * HORIZ_GEAR_RATIO, RETURN_SPEED);
}
//Author: Simon
//return to starting position for 2d scan
void returnToStart() {
  float offset = 6.0; // for mecahnical slop :(
  float target_angle = PITCH_2D_ANGLE - offset;
  timer return_timer;
  return_timer.clear();
  bool pitch_done = false, horiz_done = false;
  while (!(pitch_done && horiz_done) && return_timer.time(msec) < 15000 && !Controller.ButtonEDown.pressing()) {
    pitch_done = pitchTo(target_angle);
    horiz_done = horizTo(0);
    wait(10, msec);
  }
  PitchM.stop();
  HorizM.stop();
}
//Author: Simon
bool pitchCalibration() {
  timer cal_timer;
  float start = (float)PitchM.position(degrees);
  float travelled = 0; //degrees tilted at the sensor since we started
  float d = 9999;
  int checks = 0; //how many readings in a row said we are there
  int show = 0;

  cal_timer.clear();
  while (checks < PITCH_CAL_CHECKS && travelled < 120 && cal_timer.time(msec) < 60000 && !Controller.ButtonEDown.pressing()) {
    wait(35, msec); //the sensor only has a new reading about every 32ms
    d = Dist.objectDistance(mm);
    travelled = ((float)PitchM.position(degrees) - start) / PITCH_GEAR_RATIO;

    if (d >= 20 && d <= PITCH_FLOOR_MM) {
      PitchM.stop(); //hold still while we double check
      checks++;
    } else {
      checks = 0;
      //speed depends on how far the reading is from the floor value
      float speed = PITCH_CAL_SLOW + (PITCH_CAL_FAST - PITCH_CAL_SLOW) * (d - PITCH_FLOOR_MM) / PITCH_CAL_SLOW_ZONE_MM;
      if (speed > PITCH_CAL_FAST) speed = PITCH_CAL_FAST;
      if (speed < PITCH_CAL_SLOW || d < 20) speed = PITCH_CAL_SLOW; //under 20 the sensor isnt reliable, so go slow
      PitchM.spin(forward, speed, rpm);
    }

    show++;
    if (show >= 6) { //only redraw every few readings so the screen doesnt flicker
      show = 0;
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Tilt cal");
      Brain.Screen.newLine();
      Brain.Screen.print("d=%.0f a=%.0f n=%d", d, travelled, checks);
    }
  }
  PitchM.stop();

  if (checks < PITCH_CAL_CHECKS) return false;
  PitchM.setPosition(PITCH_3D_ANGLE * PITCH_GEAR_RATIO, degrees);
  return true;
}

//Author: Simon
void selfCenter(float theta, float dist, scan_ctx& scan_context) {
  float lateral = dist * sinf(theta * DEG_TO_RAD); //sideways part of the reading. negative on the left
  if (theta >= WALL_MIN_ANGLE && theta <= WALL_MAX_ANGLE) {
    scan_context.right_sum += lateral;
    scan_context.right_count++;
  } else if (theta <= -WALL_MIN_ANGLE && theta >= -WALL_MAX_ANGLE) {
    scan_context.left_sum += -lateral; //flip the sign so both sides are positive distances
    scan_context.left_count++;
  }
}
//Author: Simon
void updateSteer(scan_ctx& scan_context) {
  if (scan_context.left_count > 0 && scan_context.right_count > 0) {
    float left_avg = scan_context.left_sum / scan_context.left_count;
    float right_avg = scan_context.right_sum / scan_context.right_count;
    float steer = KP_WALL * (right_avg - left_avg); //more room on the right means steer right
    if (steer > WALL_STEER_MAX) steer = WALL_STEER_MAX;
    if (steer < -WALL_STEER_MAX) steer = -WALL_STEER_MAX;
    scan_context.steer = steer;
  } else {
    scan_context.steer = 0; //one wall is missing so just go straight
  }
  scan_context.left_sum = 0;
  scan_context.right_sum = 0;
  scan_context.left_count = 0;
  scan_context.right_count = 0;
}


//Author: Simon
void scan(float maximum_angle, float speed, const robot_ctx& robot_context, scan_ctx& scan_context){
  //this was a switch but we cant use break
  if (scan_context.scan_state == ScanState::Positioning) {
      //we still moving to the starting position
      if (pitchTo(scan_context.pitch_target)) {
        scan_context.scan_state = ScanState::Sweeping;
      }
  } else if (scan_context.scan_state == ScanState::Sweeping) {
      //we are sweeping
      float limit = maximum_angle * HORIZ_GEAR_RATIO;
      float pos = (float)HorizM.position(degrees);
      //every time the sweep turns around we have seen both walls once we update the steer
      if (scan_context.sweep_positive && pos >= limit) {
        scan_context.sweep_positive = false;
        updateSteer(scan_context);
      } else if (!scan_context.sweep_positive && pos <= -limit) {
        scan_context.sweep_positive = true;
        updateSteer(scan_context);
      }
      HorizM.spin(scan_context.sweep_positive ? forward : reverse, speed, rpm);
      if (scan_context.distance_reading_countdown.time(msec) >= DISTANCE_INTERVAL_MS){
        scan_context.distance_reading_countdown.clear();
        if (Dist.isObjectDetected()){
          Point p = {};
          float theta = getCorrectAngle(Brain.Timer.time(msec), scan_context); // offset to unstale the stale data
          float d = Dist.objectDistance(mm); //read once so every function gets the same value
          if (State::Scan2D == robot_context.robot_state){
            find2D(theta, d, robot_context, p);
            p.horiz_deg = floatTOint16(roundf(theta)); //save the angle too so we know which wall the point is on
            selfCenter(theta, d, scan_context); //only in 2d. in 3d the sensor looks at the floor
          } else if (State::Scan3D == robot_context.robot_state){
            find3D(theta, d, robot_context, p);
          }
          bufferWrite(p, scan_context);
        }
      }
  }
}

//Author: Jija
void startScan(State s, robot_ctx& robot_context, scan_ctx& scan_context) {

  scan_context.left_sum = 0; //reset all scan_context in case this is the second time the robot has been put to scan in a single run
  scan_context.right_sum = 0;
  scan_context.left_count = 0;
  scan_context.right_count = 0;
  scan_context.steer = 0;

  robot_context.robot_state = s; //should only be Scan2d or Scan3d
  scan_context.is_3d = (s == State::Scan3D);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(scan_context.is_3d ? "3D scan" : "2D scan");
  Brain.Screen.newLine();
  scan_context.sd_ok = initSD(scan_context); //every time we start a scan we want to create a new file to write to.
  scan_context.buffer_count = 0;

  scan_context.pitch_target = (s == State::Scan2D) ? PITCH_2D_ANGLE : PITCH_3D_ANGLE;
  //scan() moves the pitch there while its in Positioning
  scan_context.sweep_positive = true;
  scan_context.scan_state = ScanState::Positioning;
}

//Author: Simon
//jump straight from one scan to the other
void switchScan(State s, robot_ctx& robot_context, scan_ctx& scan_context) {
  leftMotor.stop(brake);
  rightMotor.stop(brake);
  HorizM.stop(brake);
  flushToSD(scan_context);
  startScan(s, robot_context, scan_context);//this puts us back in Positioning so the pitch moves to the new angle
}
//Author: Jijia
void stopScan(robot_ctx& robot_context, scan_ctx& scan_context) {
  leftMotor.stop(brake);
  rightMotor.stop(brake);
  HorizM.stop(brake);
  PitchM.stop(brake);
  flushToSD(scan_context);
  robot_context.robot_state = State::Teleop;
  returnToStart();//return to starting position for 2d scan cuz why not looks nice
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code

  robot_ctx robot_context{0,0,0.0, State::Idle, false, 0, 0};
  scan_ctx scan_context;
  scan_context.sd_ok = false;
  scan_context.is_3d = false;
  scan_context.buffer_count = 0;
  scan_context.scan_state = ScanState::Positioning;
  scan_context.sweep_positive = true;
  scan_context.pitch_target = 0;
  scan_context.hist_count = 0;
  scan_context.hist_previous_reading = 0;
  for (int i = 0; i < HIST_SIZE; i++){
    scan_context.hist_deg[i] = 0;
    scan_context.hist_timings[i] = 0;
  }

  PitchM.setStopping(hold);//found this in the vex docs kinda cool
  HorizM.setStopping(brake);
  resetOdometry(robot_context);

  sensorCalibration(robot_context);
  bool prev_estop = false, prev_arm = false, prev_2d = false, prev_3d = false, prev_l3 = false;

  //E Up = arm, E Down = estop, F Up = 2d scan, F Down = 3d scan (both creep forward), L3 = stop scan -> all debug, we can remove later on. Was having issues w connectivity
  while (!Controller.ButtonR3.pressing()) { // exit button
    bool estop = deBounce(prev_estop, Controller.ButtonEDown.pressing());
    bool arm = deBounce(prev_arm, Controller.ButtonEUp.pressing());
    bool scan2d = deBounce(prev_2d, Controller.ButtonFUp.pressing());
    bool scan3d = deBounce(prev_3d, Controller.ButtonFDown.pressing());
    bool l3 = deBounce(prev_l3, Controller.ButtonL3.pressing());

    updateOdometry(robot_context);
    encoder_history(scan_context);
    if (estop) {
      enterIdle(robot_context, scan_context);
    }
    else {
      //chnaged to if statements to avoid breaks
      if (robot_context.robot_state == State::Idle) {
          if (arm) armRobot(robot_context);
      }
      else if (robot_context.robot_state == State::Calibration) {
          sensorCalibration(robot_context); //if it reaches here it means we failed calibration so well try again
      }
      else if (robot_context.robot_state == State::Teleop) {
          teleopDrive();
          if (scan2d) {
            robot_context.robot_state = State::Scan2D;
            startScan(State::Scan2D, robot_context, scan_context);
            //start scanning
          }
          else if (scan3d) {
            robot_context.robot_state = State::Scan3D;
            startScan(State::Scan3D, robot_context, scan_context);
            //start scanning
          }
      }
      else if (robot_context.robot_state == State::Scan2D) {
          creepDrive(scan_context);//2d creeps forward too
          scan(SWEEP_MAX_ANGLE, SWEEP_SPEED, robot_context, scan_context);
          if (scan2d || l3){
            stopScan(robot_context, scan_context);
          }
          else if (scan3d) {
            switchScan(State::Scan3D, robot_context, scan_context);
          }
      }
      else if (robot_context.robot_state == State::Scan3D) {
          creepDrive(scan_context);//bot drives itself in 3d
          scan(SWEEP_MAX_ANGLE, SWEEP_SPEED, robot_context, scan_context);
          if (scan3d || l3){
            stopScan(robot_context, scan_context);
          }
          else if (scan2d) {
            switchScan(State::Scan2D, robot_context, scan_context);//stops creeping and flips the pitch back to 0
          }
      }
    }
    wait(10, msec);
  }

  stopAllMotors();
  Brain.programStop();
  return 0;
}
