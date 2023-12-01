#ifndef __GEOMETRY_UTILS_HPP__
#define __GEOMETRY_UTILS_HPP__

#include <cmath>
#include "controller_type.hpp"

#define F2P(X) (1000 / X)
#define R2D(X) (X * 180 / M_PI)
#define D2R(X) (X * M_PI / 180)

#define FILTER_OFFSET(X) (abs(X) > OFFSET ? X : 0.0)

double mapValues(double data, double in_min, double in_max, 
                                double out_min, double out_max);
double constrain(float data, float minVal, float maxVal);
void body2World(float* body_frame, float* world_frame, float yaw);
void world2Body(float* world_frame, float* body_frame, float yaw);
#endif