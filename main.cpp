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
motor HorizM = motor(PORT5, false);
motor PitchM = motor(PORT2, true);
distance Dist = distance(PORT9);
motor Motor1 = motor(PORT1, false);


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
#include <math.h>
#include <cstdint>
#include <string>

using namespace vex;
// no "using namespace std", std::forward and vex::forward clash and it won't compile

//Global
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
const int MAX_LINE_CHARS = 21; // longest line is "-32768,-32768,-32768\n" = 21 chars

struct Point { // 6 bytes per reading
  int16_t x_mm;
  int16_t y_mm;
  int16_t z_mm; // stays 0 in 2d
};

Point buffer[BUF_SIZE];
int buffer_count sensorCalibration();
void flushToSD();
void bufferWrite(const Point& p);
void getCoords(float horiz_enc_deg, float pitch_enc_deg, float dist_mm, Point& p);
void homePitch();
void pitchTo(float angle);
void sweep(float maximum_angle, float speed);
void scanPoint2D(float maximum_angle, float speed);
void scanPoint3D(angle, float seed);

timer timer1;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexdeInit();
  // B
  homePitch();
egin project code
  
  while (true){
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Up = 2D scan");
    Brain.Screen.setCursor(2, 1);
    Brain.Seen.print("ensorCalibration();

    while (!Brain.buttonUp.pressing() && !Brain.buttonDown.pressing()){ wait(10, msec); }
    is_3d = Brain.buttonDown.pressing();
    while (Brain.buttonUp.pressing() || Brain.buttonDown.pressing()){ wait(10, msec); }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    sd_ok = initSD(); // new file for every scan
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print(is_3d ? "Hold check = stop" : "Scanning");

    if (is_3d){
      scanPoint3D(140, 20); // random values that passed the vibes test
    } else{
      void sensoraliation complete");
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
  static char flush_text[BUF_SIZE * MAX_LINE_CHARS + 1]; // static so that it reserves the memory the entire time and we dont suddenly get a crash when we suddenly run out. You can decide whether the static is necessary

  if (buffer_count == 0) return;

  int len = 0;
  for (int i = 0; i < buffer_count; i++) {
    Point p = buffer[i];
    int room = (int)sizeof(flush_text) - len;
    int n;
    if (is_3d) {
      n = snprintf(flush_text + len, room, "%d,%d,%d\n", p.x_mm, p.y_mm, p.z_mm);
    } else {
      n = snprintf(flush_text + len, room, "%d,%d\n", p.x_mm, p.y_mm);
    }
  }

  fwrite(flush_text, 1, len, stdout);
  fflush(stdout);

  if (sd_ok) {
    Brain.SDcard.appendfile(c_file_name.c_str(), (uint8_t*)flush_text, len);
  }
  buffer_count = 0;
}

void bufferWrite(const Point& p) {
  buffer[buffer_count] = p;

  if (++buffer_count >= BUF_SIZE) {
    flushToSD();
  }
}

void getCoords(float horiz_enc_deg, float pitch_e


void pitchTo(float angle){
  PitchM.spinToPosition(angle * PITCH_GEAR_RATIO, degrees, 50, rpm, true);
}

void sweep(float maximum_angle, float speed){
  static timer distance_reading_countdown;

  // Flip direction at the endstops 
  if (fabs(robot.horizM_enc - robot.horizM_left_endstop) < 1 || fabs(robot.horizM_enc - robot.horizM_right_endstop) < 1){
    robot.horizM_dir = !robot.horizM_dir;
  }
  HorizM.spin(robot.horizM_dir ? forward : reverse, speed, rpm);

  if (distance_reading_countdown.time(msec) >= DISTANCE_INTERVAL_MS){
    distance_reading_countdown.reset();
    if (Dist.isObjectDetected()){
      Point p = {};
      getCoords(HorizM.position(degrees), PitchM.position(degrees), Dist.objectDistance(mm), p);
      bufferWrite(p);
    }
  }
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
