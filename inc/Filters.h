#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <math.h>
#include "LSM6DSL.h"

double Pitch_ComplementaryFilter(int elapsed_ticks, int tickrate, double *acc, double *gyro);
double Pitch_KalmanFilter (int elapsed_ticks, int tickrate, double *acc, double *gyro);

#endif
