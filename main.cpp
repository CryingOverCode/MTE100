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
  
}
