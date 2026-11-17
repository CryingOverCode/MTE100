#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "iq_cpp.h"
#include <math.h>
#include <cstdint>

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
motor Motor1 = motor(PORT1, false);
motor Motor6 = motor(PORT6, false);
distance Distance2 = distance(PORT2);


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


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

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

// Allows for easier use of the VEX Library
using namespace vex;

struct Point2D {
  float x;
  float y;
};
struct Point3D {
  float x;
  float y;
  float z;
}
enum struct State {//did we see enums in class? prob not lmfao 
  Idle,
  Teleop,
  Scan2D,
  Scan3D,
  Calibration,
}

std::string c_file_name;
bool sd_ok = false;              
bool is_3d = false;              
bool last_went_positive = false; 

//
const float DISTANCE_INTERVAL_MS = 1000.0 / 30; // 30 Hz max
const int BUF_SIZE = 64; // 64 readings * 6 bytes -> 384 bytes per buffer. ~ 2.13s of data before an SD card flush
const float HORIZ_GEAR_RATIO = 4;
const float PITCH_GEAR_RATIO = 30;
const float PITCH_STOP_ANGLE = -90;


struct Point { // 6 bytes per reading
  int16_t x_mm;
  int16_t y_mm;
  int16_t z_mm; // stays 0 in 2d
};

Point buffer[BUF_SIZE];
int buffer_count = 0;

bool initSD();
void flushToSD();
void bufferWrite(const Point& p);
void getCoords(float horiz_enc_deg, float pitch_enc_deg, float dist_mm, Point& p);
void homePitch();
void pitchTo(float angle);
void sweep(float maximum_angle, float speed);
void scanPoint2D(float maximum_angle, float speed);
void scanPoint3D(float maximum_angle, float speed);

//Assumptions: start of maze is (0,0). Theta is relative to direclty forward of the path. Left-right of init. is x-axis. Front-back of init. is y-axis.
void find2D(Point2D BotCoord, float DistTheta, float BotTheta, float Dist) {
	float xVal = BotCoord.x + (sin(DistTheta + BotTheta)*Dist);  
	float yVal = BotCoord.y + (cos(DistTheta + BotTheta)*Dist); 
	//figure out a way to account for error and write to some database?
}
//Assumptions: start of maze is (0,0,0). Directly downwards is 0 degrees and directly forwards is 0 degrees. Left-right of init. is x-axis. Front-back is y-axis. Up-down is z-axis.
void find3D(Point3D BotCoord, float DistTheta, float Dist) { //theta2 and theta3 named for 2d and 3d respectively.
	const float kx = 0; //this is some experimentally determined offset from the 2d position, which is the proper (0,0,0). 
	const float ky = 0;
	const float kz = 0;
	float xVal = kx + BotCoord.x + (Dist*sin(DistTheta)*cos(BotTheta));
	float yVal = ky + BotCoord.y + (Dist*sin(DistTheta)*sin(BotTheta)); //scanning along the y plane.
	float zVal = kz + BotCoord.z -1*(Dist*cos(DistTheta)); //*-1 since facing down is positive. Maybe double check the signs and how much each BotCoord is required, esp. z
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code

  homePitch();

//   while (true){
//     Brain.Screen.clearScreen();
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print("Up = 2D scan");
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.print("Down = 3D scan");

//     while (!Brain.buttonUp.pressing() && !Brain.buttonDown.pressing()){ wait(10, msec); }
//     is_3d = Brain.buttonDown.pressing();
//     while (Brain.buttonUp.pressing() || Brain.buttonDown.pressing()){ wait(10, msec); }

//     Brain.Screen.clearScreen();
//     Brain.Screen.setCursor(1, 1);
//     sd_ok = initSD(); // new file for every scan
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.print(is_3d ? "Hold check = stop" : "Scanning");

//     if (is_3d){
//       scanPoint3D(140, 20); // random values that passed the vibes test
//     } else{
//       scanPoint2D(140, 20);
//     }
  }
}

bool initSD() {
  const char* header = is_3d ? "x_mm,y_mm,z_mm\n" : "x_mm,y_mm\n";
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

  c_file_name = current_name;

  Brain.SDcard.savefile(c_file_name.c_str(), (uint8_t*)header, strlen(header));
  return true;
}

void flushToSD() {
  for (int i = 0; i < buffer_count; i++) {
    Point p = buffer[i];
    char line[24];   // longest is "-32768,-32768,-32768\n" = 21 chars
    int len;
    if (is_3d){
      len = snprintf(line, sizeof(line), "%d,%d,%d\n", p.x_mm, p.y_mm, p.z_mm);
    } else{
      len = snprintf(line, sizeof(line), "%d,%d\n", p.x_mm, p.y_mm);
    }
    printf("%s", line); 
    if (sd_ok){
      Brain.SDcard.appendfile(c_file_name.c_str(), (uint8_t*)line, len);
    }
  }
  fflush(stdout);
  buffer_count = 0;
}

void bufferWrite(const Point& p) {
  buffer[buffer_count] = p;

  if (++buffer_count >= BUF_SIZE) {
    flushToSD();
  }
}

void getCoords(float horiz_enc_deg, float pitch_enc_deg, float dist_mm, Point& p){

}

void pitchTo(float angle){
  PitchM.spinToPosition(angle * PITCH_GEAR_RATIO, degrees, 50, rpm, true);
}

void sweep(float maximum_angle, float speed){

  float max_motor_pos = maximum_angle * HORIZ_GEAR_RATIO;

  // start at the end the last sweep finished at, then go to the other end
  float start = last_went_positive ? max_motor_pos : -max_motor_pos;
  float finish = -start;
  bool going_positive = finish > start;

  HorizM.spinToPosition(start, degrees, 100, rpm, true); // does nothing if already there

  if (going_positive){
    HorizM.spin(forward, speed, rpm);
  } else{
    HorizM.spin(reverse, speed, rpm);
  }

  timer distance_reading_countdown;

  while ((going_positive && HorizM.position(degrees) < finish) || (!going_positive && HorizM.position(degrees) > finish)){
    if (distance_reading_countdown.time(msec) >= DISTANCE_INTERVAL_MS){

      distance_reading_countdown.reset();

      if (Dist.isObjectDetected()){ // skip readings with nothing in range
        Point p = {};
        getCoords(HorizM.position(degrees), PitchM.position(degrees), Dist.objectDistance(mm), p);
        bufferWrite(p);
      }
    }
    wait(1, msec);
  }
  HorizM.stop(brake);

  last_went_positive = going_positive;
}

void scanPoint2D(float maximum_angle, float speed){
  pitchTo(0);
  sweep(maximum_angle, speed);

  flushToSD(); // write the partial buffer at the end
}

void scanPoint3D(float maximum_angle, float speed){
  pitchTo(90);
  while (!Brain.buttonCheck.pressing()){ // bot moves forward slowly while this runs
    sweep(maximum_angle, speed);
  }
  while (Brain.buttonCheck.pressing()){ wait(10, msec); }
  pitchTo(0);

  flushToSD(); // write the partial buffer at the end
}

