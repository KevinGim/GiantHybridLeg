#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "Kinematics.h"

#define PI 3.141592

double LF = 0.0;    // Ankle to foot offset
double L0_1 = 69.5; // Hip Roll Axis offset
double L0_2 = 45.4;
double L0 = 115.0;
double L2 = 900.0; // Femur
double L3 = 910.0; // Tibia
double R1_1 = 0.0;
double R2_1 = 0.0;
double R1_2 = 0.0;
double R2_2 = 0.0;
double th1_1 = 0.0;
double th2_1 = 0.0;
double th3_1 = 0.0;
double th4_1 = 0.0;
double th7_1 = 0.0;
double th8_1 = 0.0;
double th9_1 = 0.0;
double th1_2 = 0.0;
double th2_2 = 0.0;
double th3_2 = 0.0;
double th4_2 = 0.0;
double th7_2 = 0.0;
double th8_2 = 0.0;
double th9_2 = 0.0;

long int *IK(double x, double y, double z, double roll, double pitch, double yaw)
{

    static long int motor_angle[] = {0, 0, 0, 0, 0, 0};
    // printf("x: %f, y: %f, z: %f         ", x, y, z);

    // End Effector Orientation and Position

    double P0_19_1[] =
        {x - L0 * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + LF * (sin(pitch) * sin(yaw) + cos(pitch) * cos(yaw) * sin(roll)),
         y - L0_1 + L0 * (cos(pitch) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - LF * (cos(yaw) * sin(pitch) - cos(pitch) * sin(roll) * sin(yaw)),
         z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch)};

    double P0_29_2[] =
        {x + L0 * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + LF * (sin(pitch) * sin(yaw) + cos(pitch) * cos(yaw) * sin(roll)),
         L0_1 + y - L0 * (cos(pitch) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - LF * (cos(yaw) * sin(pitch) - cos(pitch) * sin(roll) * sin(yaw)),
         z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch)};

    R1_1 = sqrt(P0_19_1[1] * P0_19_1[1] + P0_19_1[2] * P0_19_1[2]);
    R1_2 = sqrt(P0_29_2[1] * P0_29_2[1] + P0_29_2[2] * P0_29_2[2]);

    th1_1 = (PI / 2.0 - acos(L0_2 / R1_1) - asin(P0_19_1[1] / R1_1));
    th1_2 = (-PI / 2.0 + acos(L0_2 / R1_2) - asin(P0_29_2[1] / R1_2));

    double P1_19_1[] =
        {x - L0 * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + LF * (sin(pitch) * sin(yaw) + cos(pitch) * cos(yaw) * sin(roll)),
         -L0_1 + y - L0_2 * sin(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw),
         z + L0_2 * cos(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch)};

    double P1_29_2[] =
        {x + L0 * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + LF * (sin(pitch) * sin(yaw) + cos(pitch) * cos(yaw) * sin(roll)),
         L0_1 + y - L0_2 * sin(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)))) - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw),
         z + L0_2 * cos(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)))) + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch)};

    R2_1 = sqrt(P1_19_1[0] * P1_19_1[0] + P1_19_1[1] * P1_19_1[1] + P1_19_1[2] * P1_19_1[2]);
    R2_2 = sqrt(P1_29_2[0] * P1_29_2[0] + P1_29_2[1] * P1_29_2[1] + P1_29_2[2] * P1_29_2[2]);

    if (R2_1 > 1810.0)
    {
        R2_1 = 1810.0;
    }
    if (R2_2 > 1810.0)
    {
        R2_2 = 1810.0;
    }

    th3_1 = PI - acos((L2 * L2 + L3 * L3 - R2_1 * R2_1) / (2 * L2 * L3));
    th3_2 = PI - acos((L2 * L2 + L3 * L3 - R2_2 * R2_2) / (2 * L2 * L3));

    th2_1 = -(asin(P0_19_1[0] / R2_1) + acos((L2 * L2 + R2_1 * R2_1 - L3 * L3) / (2 * L2 * R2_1)));
    th2_2 = -(asin(P0_29_2[0] / R2_2) + acos((L2 * L2 + R2_2 * R2_1 - L3 * L3) / (2 * L2 * R2_2)));

    th9_1 = -asin(1.0 / sqrt(-pow(-sin(th2_1 + th3_1) * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(th2_1 + th3_1) * cos(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) * (cos(pitch) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) + cos(th2_1 + th3_1) * sin(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) * cos(roll) * sin(pitch), 2.0) + 1.0) * (-cos(th2_1 + th3_1) * sin(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) * sin(roll) + sin(th2_1 + th3_1) * cos(roll) * cos(yaw) + cos(th2_1 + th3_1) * cos(acos(L0_2 * 1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0))) + asin(1.0 / sqrt(pow(-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) + L0 * cos(roll) * sin(pitch), 2.0)) * (-L0_1 + y + L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) + L0 * sin(pitch) * sin(roll) * sin(yaw)))) * cos(roll) * sin(yaw)));
    th9_2 = -asin(1.0 / sqrt(-pow(sin(th2_2 + th3_2) * (cos(pitch) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(th2_2 + th3_2) * cos(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)))) * (cos(pitch) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) + sin(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)))) * cos(th2_2 + th3_2) * cos(roll) * sin(pitch), 2.0) + 1.0) * (sin(th2_2 + th3_2) * cos(roll) * cos(yaw) + sin(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)))) * cos(th2_2 + th3_2) * sin(roll) - cos(th2_2 + th3_2) * cos(roll) * sin(yaw) * cos(asin(1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0)) * (L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw))) - acos(L0_2 * 1.0 / sqrt(pow(L0_1 + y - L0 * cos(pitch) * cos(yaw) - LF * cos(yaw) * sin(pitch) + LF * cos(pitch) * sin(roll) * sin(yaw) - L0 * sin(pitch) * sin(roll) * sin(yaw), 2.0) + pow(z + LF * cos(pitch) * cos(roll) - L0 * cos(roll) * sin(pitch), 2.0))))));

    th4_1 = knee2act(th2_1, th3_1);
    th4_2 = knee2act(th2_2, th3_2);

    // printf("th1_1: %3.3f, th2_1: %3.3f, th2_2: %3.3f, th4_1: %3.3f, th4_2: %3.3f, th9_1: %3.3f", th1_1, th2_1, th2_2, th4_1, th4_2, th9_1);

    motor_angle[0] = (long)(th1_1 * 159767.053003);
    motor_angle[1] = (long)(-th2_1 * 159767.053003);
    motor_angle[2] = (long)(-th2_2 * 159767.053003) - 2000;
    motor_angle[3] = (long)(th4_1 * 159767.053003);
    motor_angle[4] = (long)(th4_2 * 159767.053003) - 3000;
    motor_angle[5] = (long)(th9_1 * 159767.053003);

    return motor_angle;
}

double knee2act(double th2, double th3)
{
    double L1 = 198.0;
    double L4 = 382.0;
    double L5 = 960.0;
    double L6_1 = 261.4625;
    double L6_2 = 62.8;
    double L6 = sqrt(L6_1 * L6_1 + L6_2 * L6_2);
    double offset_3 = atan(L6_2 / L6_1);
    double P02_1 = L2 * sin(th2);
    double P02_2 = -L2 * cos(th2);
    double P03_1 = L1;
    double P03_2 = 0;
    double P06_1 = L2 * sin(th2) + L6 * sin(offset_3 + th2 + th3);
    double P06_2 = -L2 * cos(th2) - L6 * cos(offset_3 + th2 + th3);
    double v1_1 = P03_1 - P02_1;
    double v1_2 = P03_2 - P02_2;
    double v2_1 = P06_1 - P03_1;
    double v2_2 = P06_2 - P03_2;
    double v1_norm = sqrt(v1_1 * v1_1 + v1_2 * v1_2);
    double v2_norm = sqrt(v2_1 * v2_1 + v2_2 * v2_2);
    double beta1 = acos((L4 * L4 + v2_norm * v2_norm - L5 * L5) / (2 * L4 * v2_norm));
    double beta2 = acos((v1_norm * v1_norm + v2_norm * v2_norm - L6 * L6) / (2 * v1_norm * v2_norm));
    double beta3 = acos((v1_norm * v1_norm + L1 * L1 - L2 * L2) / (2 * v1_norm * L1));
    double beta = -(beta1 + beta2 + beta3);
    double th4 = beta + PI / 2;

    return th4;
}