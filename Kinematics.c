#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "Kinematics.h"

# define PI 3.1415

float LF = 0.0;   // Ankle to foot offset default = 89.0;
float L0 = 139.0;  // Hip Roll Axis offset
float L1 = 198; 
float L2 = 900;    // Femur
float L3 = 900;    // Tibia

float R_1 = 0.0;
float R_2 = 0.0;
float R_3 = 0.0;
float R_4 = 0.0;

float th1_1 = 0.0;
float th2_1 = 0.0;
float th3_1 = 0.0;
float th4_1 = 0.0;
float th7_1 = 0.0;
float th8_1 = 0.0;
float th9_1 = 0.0;

float th1_2 = 0.0;
float th2_2 = 0.0;
float th3_2 = 0.0;
float th4_2 = 0.0;
float th7_2 = 0.0;
float th8_2 = 0.0;
float th9_2 = 0.0;




long int* IK(float x, float y, float z, float roll, float pitch, float yaw){

static long int motor_angle[] = {0, 0, 0, 0, 0, 0};
// printf("x: %f, y: %f, z: %f         ", x, y, z);

// End Effector Orientation and Position

float P0_19_1[] = 
{L0/2.0+z-(L0*cos(roll)*cos(yaw))/2.0+LF*sin(pitch)*sin(yaw)+LF*cos(pitch)*cos(yaw)*sin(roll)+(L0*cos(PI/2.0)*sin(pitch)*sin(yaw))/2.0+(L0*pow(cos(PI/2.0),2.0)*cos(roll)*cos(yaw))/2.0+(L0*cos(PI/2.0)*cos(pitch)*cos(yaw)*sin(roll))/2.0-(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*sin(yaw))/2.0+(L0*cos(PI/2.0)*sin(PI/2.0)*cos(yaw)*sin(pitch)*sin(roll))/2.0,
x*cos(PI/2.0)-y*sin(PI/2.0)-(L0*sin(PI/2.0)*sin(roll))/2.0+(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(roll))/2.0-LF*sin(PI/2.0)*cos(pitch)*cos(roll)-LF*cos(PI/2.0)*cos(yaw)*sin(pitch)-(L0*cos(PI/2.0)*cos(roll)*sin(yaw))/2.0-(L0*pow(cos(PI/2.0),2.0)*cos(yaw)*sin(pitch))/2.0+(L0*pow(cos(PI/2.0),3.0)*cos(roll)*sin(yaw))/2.0+LF*cos(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw)-(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(roll))/2.0+(L0*pow(cos(PI/2.0),2.0)*cos(pitch)*sin(roll)*sin(yaw))/2.0+(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(pitch)*cos(yaw))/2.0-(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(roll)*sin(pitch))/2.0+(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(pitch)*sin(roll)*sin(yaw))/2.0,
y*cos(PI/2.0)+x*sin(PI/2.0)-(L0*pow(cos(PI/2.0),3.0)*sin(roll))/2.0+(L0*cos(PI/2.0)*sin(roll))/2.0+LF*cos(PI/2.0)*cos(pitch)*cos(roll)-LF*sin(PI/2.0)*cos(yaw)*sin(pitch)-(L0*sin(PI/2.0)*cos(roll)*sin(yaw))/2.0+(L0*pow(cos(PI/2.0),2.0)*cos(pitch)*cos(roll))/2.0+(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(roll)*sin(yaw))/2.0+LF*sin(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw)-(L0*cos(PI/2.0)*sin(PI/2.0)*cos(yaw)*sin(pitch))/2.0+(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(pitch)*cos(yaw))/2.0+(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(roll)*sin(pitch))/2.0+(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw))/2.0+(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(pitch)*sin(roll)*sin(yaw))/2.0};
 

float P0_29_2[] = 
{L0*(-1.0/2.0)+z+(L0*cos(roll)*cos(yaw))/2.0+LF*sin(pitch)*sin(yaw)+LF*cos(pitch)*cos(yaw)*sin(roll)-(L0*cos(PI/2.0)*sin(pitch)*sin(yaw))/2.0-(L0*pow(cos(PI/2.0),2.0)*cos(roll)*cos(yaw))/2.0-(L0*cos(PI/2.0)*cos(pitch)*cos(yaw)*sin(roll))/2.0+(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*sin(yaw))/2.0-(L0*cos(PI/2.0)*sin(PI/2.0)*cos(yaw)*sin(pitch)*sin(roll))/2.0,
x*cos(PI/2.0)-y*sin(PI/2.0)+(L0*sin(PI/2.0)*sin(roll))/2.0-(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(roll))/2.0-LF*sin(PI/2.0)*cos(pitch)*cos(roll)-LF*cos(PI/2.0)*cos(yaw)*sin(pitch)+(L0*cos(PI/2.0)*cos(roll)*sin(yaw))/2.0+(L0*pow(cos(PI/2.0),2.0)*cos(yaw)*sin(pitch))/2.0-(L0*pow(cos(PI/2.0),3.0)*cos(roll)*sin(yaw))/2.0+LF*cos(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw)+(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(roll))/2.0-(L0*pow(cos(PI/2.0),2.0)*cos(pitch)*sin(roll)*sin(yaw))/2.0-(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(pitch)*cos(yaw))/2.0+(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(roll)*sin(pitch))/2.0-(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(pitch)*sin(roll)*sin(yaw))/2.0,
y*cos(PI/2.0)+x*sin(PI/2.0)+(L0*pow(cos(PI/2.0),3.0)*sin(roll))/2.0-(L0*cos(PI/2.0)*sin(roll))/2.0+LF*cos(PI/2.0)*cos(pitch)*cos(roll)-LF*sin(PI/2.0)*cos(yaw)*sin(pitch)+(L0*sin(PI/2.0)*cos(roll)*sin(yaw))/2.0-(L0*pow(cos(PI/2.0),2.0)*cos(pitch)*cos(roll))/2.0-(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(roll)*sin(yaw))/2.0+LF*sin(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw)+(L0*cos(PI/2.0)*sin(PI/2.0)*cos(yaw)*sin(pitch))/2.0-(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(pitch)*cos(yaw))/2.0-(L0*pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(roll)*sin(pitch))/2.0-(L0*cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*sin(roll)*sin(yaw))/2.0-(L0*cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(pitch)*sin(roll)*sin(yaw))/2.0};
 
R_1 = sqrt(P0_19_1[0]*P0_19_1[0] + P0_19_1[1]*P0_19_1[1] + P0_19_1[2]*P0_19_1[2]);
R_2 = sqrt(P0_29_2[0]*P0_29_2[0] + P0_29_2[1]*P0_29_2[1] + P0_29_2[2]*P0_29_2[2]);
R_3 = sqrt(P0_19_1[0]*P0_19_1[0] + P0_19_1[1]*P0_19_1[1]);
R_4 = sqrt(P0_29_2[0]*P0_29_2[0] + P0_29_2[1]*P0_29_2[1]);



if (R_3 > 1800.0){
    R_3 = 1800.0;
}

if (R_4 > 1800.0){
    R_4 = 1800.0;
}


th1_1 = -atan(P0_19_1[0]/P0_19_1[1]);
th3_1 = (PI - acos((L2*L2 + L3*L3 - R_1*R_1)/(2*L2*L3)));
th2_1 = -(acos((L2*L2 + R_1*R_1 - L3*L3)/(2*L2*R_1)) + (P0_19_1[2]/(abs(P0_19_1[2])+0.000001))*(acos(R_3/R_1)));


th1_2 = (-atan(P0_29_2[0]/P0_29_2[0]));
th3_2 = (PI -acos((L2*L2 + L3*L3 - R_2*R_2)/(2*L2*L3)));
th2_2 = -(acos((L2*L2 + R_2*R_2 - L3*L3)/(2*L2*R_2)) + (P0_29_2[2]/(abs(P0_29_2[2])+0.000001))*(acos(R_4/R_2)));

// printf("       %f        %f          ", th2_2, th3_2);

th8_1 = asin(cos(PI/2.0)*sin(th2_1+th3_1)*sin(roll)-pow(cos(PI/2.0),3.0)*sin(th2_1+th3_1)*sin(roll)-sin(PI/2.0)*sin(th2_1+th3_1)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(roll)-cos(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)+sin(PI/2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(roll)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(th1_1)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)-pow(cos(PI/2.0),3.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(roll)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(roll)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(roll)*sin(yaw)+cos(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)+cos(PI/2.0)*cos(th2_1+th3_1)*sin(pitch)*sin(th1_1)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(th1_1)*cos(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*sin(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_1)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(roll)*cos(th1_1)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*sin(th1_1)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(th1_1)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_1));
th8_2 = asin(cos(PI/2.0)*sin(th2_2+th3_2)*sin(roll)-pow(cos(PI/2.0),3.0)*sin(th2_2+th3_2)*sin(roll)-sin(PI/2.0)*sin(th2_2+th3_2)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(roll)-cos(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)+sin(PI/2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(roll)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(th1_2)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)-pow(cos(PI/2.0),3.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(roll)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(roll)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(roll)*sin(yaw)+cos(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)+cos(PI/2.0)*cos(th2_2+th3_2)*sin(pitch)*sin(th1_2)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(th1_2)*cos(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*sin(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_2)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(roll)*cos(th1_2)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*sin(th1_2)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(th1_2)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_2));

th7_1 = atan((cos(PI/2.0)*cos(th2_1+th3_1)*sin(roll)-pow(cos(PI/2.0),3.0)*cos(th2_1+th3_1)*sin(roll)+pow(cos(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(roll)+sin(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)-sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*sin(yaw)-sin(PI/2.0)*sin(th2_1+th3_1)*cos(th1_1)*sin(roll)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(yaw)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(th1_1)*cos(yaw)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)+pow(cos(PI/2.0),3.0)*sin(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(th1_1)*sin(roll)-cos(PI/2.0)*sin(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)-cos(PI/2.0)*sin(th2_1+th3_1)*sin(pitch)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(th1_1)*sin(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(th1_1)*cos(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_1)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(roll)*cos(th1_1)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(th1_1)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_1))/(-sin(PI/2.0)*sin(roll)*sin(th1_1)-cos(roll)*cos(th1_1)*cos(yaw)+cos(PI/2.0)*cos(th1_1)*sin(pitch)*sin(yaw)-cos(PI/2.0)*cos(roll)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*cos(roll)*cos(th1_1)*cos(yaw)-pow(cos(PI/2.0),2.0)*cos(yaw)*sin(pitch)*sin(th1_1)+pow(cos(PI/2.0),3.0)*cos(roll)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(roll)*sin(th1_1)-cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(roll)*sin(th1_1)-cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*cos(pitch)*sin(roll)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(pitch)*cos(yaw)*sin(th1_1)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(roll)*sin(pitch)*sin(th1_1)+cos(PI/2.0)*cos(pitch)*cos(th1_1)*cos(yaw)*sin(roll)+cos(PI/2.0)*sin(PI/2.0)*cos(th1_1)*cos(yaw)*sin(pitch)*sin(roll)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(pitch)*sin(roll)*sin(th1_1)*sin(yaw)));
th7_2 = atan((cos(PI/2.0)*cos(th2_2+th3_2)*sin(roll)-pow(cos(PI/2.0),3.0)*cos(th2_2+th3_2)*sin(roll)+pow(cos(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(roll)+sin(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)-sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*sin(yaw)-sin(PI/2.0)*sin(th2_2+th3_2)*cos(th1_2)*sin(roll)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(yaw)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(th1_2)*cos(yaw)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)+pow(cos(PI/2.0),3.0)*sin(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(th1_2)*sin(roll)-cos(PI/2.0)*sin(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)-cos(PI/2.0)*sin(th2_2+th3_2)*sin(pitch)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(th1_2)*sin(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(th1_2)*cos(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_2)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(roll)*cos(th1_2)+cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*sin(roll)*sin(yaw)+cos(PI/2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(th1_2)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_2))/(-sin(PI/2.0)*sin(roll)*sin(th1_2)-cos(roll)*cos(th1_2)*cos(yaw)+cos(PI/2.0)*cos(th1_2)*sin(pitch)*sin(yaw)-cos(PI/2.0)*cos(roll)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*cos(roll)*cos(th1_2)*cos(yaw)-pow(cos(PI/2.0),2.0)*cos(yaw)*sin(pitch)*sin(th1_2)+pow(cos(PI/2.0),3.0)*cos(roll)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(roll)*sin(th1_2)-cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(roll)*sin(th1_2)-cos(PI/2.0)*sin(PI/2.0)*cos(pitch)*cos(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*cos(pitch)*sin(roll)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(pitch)*cos(yaw)*sin(th1_2)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(roll)*sin(pitch)*sin(th1_2)+cos(PI/2.0)*cos(pitch)*cos(th1_2)*cos(yaw)*sin(roll)+cos(PI/2.0)*sin(PI/2.0)*cos(th1_2)*cos(yaw)*sin(pitch)*sin(roll)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(pitch)*sin(roll)*sin(th1_2)*sin(yaw)));

th9_1 = asin((pow(cos(PI/2.0),4.0)*sin(th2_1+th3_1)*cos(roll)*sin(pitch)-pow(sin(PI/2.0),3.0)*sin(th2_1+th3_1)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*sin(roll)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_1+th3_1)*sin(roll)-pow(cos(PI/2.0),4.0)*cos(th2_1+th3_1)*cos(pitch)*cos(th1_1)*cos(yaw)-pow(sin(PI/2.0),3.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(pitch)-pow(cos(PI/2.0),3.0)*cos(th2_1+th3_1)*cos(pitch)*sin(th1_1)*sin(yaw)+pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*sin(th1_1)*sin(yaw)-pow(sin(PI/2.0),3.0)*sin(th2_1+th3_1)*sin(pitch)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(roll)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(roll)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_1+th3_1)*cos(pitch)*cos(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(roll)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(yaw)*sin(pitch)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(roll)-pow(cos(PI/2.0),2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),3.0)*cos(th2_1+th3_1)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_1)-pow(cos(PI/2.0),4.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(pitch)*sin(roll)*sin(yaw)-pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_1)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(roll)*cos(th1_1)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(th1_1)*cos(yaw)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(th1_1)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(th1_1)*sin(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_1+th3_1)*cos(pitch)*sin(roll)*sin(yaw)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_1+th3_1)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(roll)*cos(yaw)*sin(th1_1)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*sin(pitch)*sin(th1_1)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(th1_1)*sin(roll)*sin(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_1+th3_1)*cos(th1_1)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_1+th3_1)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_1))/cos(th8_1));
th9_2 = asin((pow(cos(PI/2.0),4.0)*sin(th2_2+th3_2)*cos(roll)*sin(pitch)-pow(sin(PI/2.0),3.0)*sin(th2_2+th3_2)*cos(pitch)*cos(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*sin(roll)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_2+th3_2)*sin(roll)-pow(cos(PI/2.0),4.0)*cos(th2_2+th3_2)*cos(pitch)*cos(th1_2)*cos(yaw)-pow(sin(PI/2.0),3.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(pitch)-pow(cos(PI/2.0),3.0)*cos(th2_2+th3_2)*cos(pitch)*sin(th1_2)*sin(yaw)+pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*sin(th1_2)*sin(yaw)-pow(sin(PI/2.0),3.0)*sin(th2_2+th3_2)*sin(pitch)*sin(roll)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(roll)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(roll)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_2+th3_2)*cos(pitch)*cos(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(roll)*sin(pitch)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(yaw)*sin(pitch)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(roll)-pow(cos(PI/2.0),2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(roll)*sin(yaw)+pow(cos(PI/2.0),3.0)*cos(th2_2+th3_2)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_2)-pow(cos(PI/2.0),4.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(pitch)*sin(roll)*sin(yaw)-pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(yaw)*sin(pitch)*sin(roll)*sin(th1_2)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(roll)*cos(th1_2)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(th1_2)*cos(yaw)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(pitch)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(th1_2)*cos(yaw)*sin(pitch)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)-pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(th1_2)*sin(yaw)-cos(PI/2.0)*pow(sin(PI/2.0),2.0)*sin(th2_2+th3_2)*cos(pitch)*sin(roll)*sin(yaw)+pow(cos(PI/2.0),3.0)*sin(PI/2.0)*sin(th2_2+th3_2)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(roll)*cos(yaw)*sin(th1_2)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*sin(pitch)*sin(th1_2)*sin(yaw)+pow(cos(PI/2.0),2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(th1_2)*sin(roll)*sin(yaw)+cos(PI/2.0)*pow(sin(PI/2.0),2.0)*cos(th2_2+th3_2)*cos(th1_2)*sin(pitch)*sin(roll)*sin(yaw)-cos(PI/2.0)*sin(PI/2.0)*cos(th2_2+th3_2)*cos(pitch)*cos(yaw)*sin(roll)*sin(th1_2))/cos(th8_2));

th4_1 = knee2act(th2_1,th3_1);
th4_2 = knee2act(th2_2,th3_2);

// printf("th1_1: %f, th2_1: %f, th2_2: %f, th4_1: %f, th4_2: %f     ", th1_1, th2_1, th2_2, th4_1, th4_2);

motor_angle[0] = (long)(th1_1*159767.053003);
motor_angle[1] = (long)(-th2_1*159767.053003);
motor_angle[2] = (long)(-th2_2*159767.053003);
motor_angle[3] = (long)(th4_1*159767.053003);
motor_angle[4] = (long)(th4_2*159767.053003);
motor_angle[5] = (long)(th9_1*159767.053003);


return motor_angle;


}

float knee2act(float th2, float th3){

float L4 = 382; // 210 - (118-290)
float L5 = 953; // 975 - (922-953)
float L6_1 = 251.4615; // 131.4625 + 120
float L6_2 = 63.5375;
float L6 = sqrt(L6_1*L6_1+L6_2*L6_2); 
float offset_3 = atan(L6_2/L6_1);

float P02_1 = L2 * sin(th2); 
float P02_2 = -L2 * cos(th2);
float P03_1 = L1;
float P03_2 = 0;
float P06_1 = L2 * sin(th2) + L6 * sin(offset_3 + th2 + th3);
float P06_2 = - L2 * cos(th2) - L6 * cos(offset_3 + th2 + th3);

float v1_1 = P03_1 - P02_1;
float v1_2 = P03_2 - P02_2;
float v2_1 = P06_1 - P03_1;
float v2_2 = P06_2 - P03_2;

float v1_norm = sqrt(v1_1*v1_1 + v1_2*v1_2);
float v2_norm = sqrt(v2_1*v2_1 + v2_2*v2_2);

float beta1 = acos((L4*L4 + v2_norm*v2_norm - L5*L5) / (2 * L4 * v2_norm));
float beta2 = acos((v1_norm*v1_norm + v2_norm*v2_norm - L6*L6) / (2 * v1_norm * v2_norm));
float beta3 = acos((v1_norm*v1_norm + L1*L1 - L2*L2) / (2 * v1_norm * L1));

float beta = -(beta1 + beta2 + beta3);
float th4 = beta + PI/2;

return th4;
}