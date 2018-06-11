//  Implementation of two data fusion methods, based on Long Tran's work,
//  "Data Fusion with 9 Degrees of Freedom Inertial Measurement Unit To Determine Object's Orientation"
//  The documentation can be found at: http://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1422&context=eesp
#ifndef __FILTERS_H__
#define __FILTERS_H__

#include "LSM6DSL_functions.h"
#define PI  (3.14159265359)

double Pitch_ComplementaryFilter(int elapsed_ticks, int tickrate, double *acc, double *gyro)
{
    static double dt;
    static double pitch_angle;
    static double alpha = 0.98;                                                 // Gyroscope/Accelerometer ratio

    dt = (double)elapsed_ticks/tickrate;
    double angle_s = atan2(acc[0],sqrt(acc[1]*acc[1]+acc[2]*acc[2]))*180/PI;    // Pitch angle from the accelerometer [degrees]
    pitch_angle = alpha*(pitch_angle + gyro[1]*dt) + (1-alpha)*(angle_s);       // Complementary filtering with the gyro data

    return pitch_angle;
}

double Pitch_KalmanFilter (int elapsed_ticks, int tickrate, double *acc, double *gyro)
{
    static double q_bias      = 0.003;
    static double q_angle     = 0.001;
    static double r_measure   = 0.03;

    static double bias        = 0;
    static double pitch_angle = 0;

    static double P[2][2]     = {};
    static double K[2]        = {};

    double dt   = (double)elapsed_ticks/tickrate;
    double rate = gyro[1] - bias;
    pitch_angle += dt * rate;                                                   // Gyro integration
    double angle_s = atan2(-acc[0],sqrt(acc[1]*acc[1]+acc[2]*acc[2]))*180/PI;   // Pitch angle from the accelerometer [degrees]

    // Estimation error covariance matrix update

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += q_bias * dt;

    // Kalman gain

    double S = P[0][0] + r_measure;

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double Y = angle_s - pitch_angle;

    // Kalman angle and new bias

    pitch_angle += K[0] * Y;
    bias        += K[1] * Y;

    // Recompute estimation error covariance matrix

    double P_00_temp = P[0][0]; // temporary variables used to avoid loss of values
    double P_01_temp = P[0][1];

    P[0][0] -= K[0] * P_00_temp;
    P[0][1] -= K[0] * P_01_temp;
    P[1][0] -= K[1] * P_00_temp;
    P[1][1] -= K[1] * P_01_temp;

    return pitch_angle;
}
#endif