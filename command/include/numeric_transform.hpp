#ifndef __NUMERICAL_TRANSFORM_HPP__
#define __NUMERICAL_TRANSFORM_HPP__

#define R2D(X) (X * 180 / M_PI)
#define D2R(X) (X * M_PI / 180)

double mapValues(double data, double in_min, double in_max, double out_min, double out_max);
double constrain(float data, float minVal, float maxVal);

#endif