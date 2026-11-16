#include <cmath>
#include <iostream>

//Assumptions: start of maze is (0,0). Theta is relative to direclty forward of the path. Left-right of init. is x-axis. Front-back of init. is y-axis.
void find2D(float botX, float botY, float disttheta, float bottheta, float dist) {
	float xVal = botX + (sin(disttheta + bottheta)*dist);  
	float yVal = botY + (cos(disttheta + bottheta)*dist); 
	//figure out a way to account for error and write to some database?
}

//Assumptions: start of maze is (0,0,0). Directly downwards is 0 degrees and directly forwards is 0 degrees. Left-right of init. is x-axis. Front-back is y-axis. Up-down is z-axis.
void find3D(float botX, float botY, float bottheta, float disttheta, float dist) { //theta2 and theta3 named for 2d and 3d respectively.
	const float kx = 0; //this is some experimentally determined offset from the 2d position, which is the proper (0,0,0). 
	const float ky = 0;
	const float kz = 0;
	float xVal = kx + botX + (dist*sin(disttheta)*cos(bottheta));
	float yVal = ky + botY + (dist*sin(disttheta)*sin(bottheta)); //scanning along the y plane.
	float zVal = kz + -1*(dist*cos(theta3)); //*-1 since facing down is positive.
}