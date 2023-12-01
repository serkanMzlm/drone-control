#include "geometry_utils.hpp"

double mapValues(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

double constrain(float data, float minVal, float maxVal){
  return fminf(maxVal, fmaxf(minVal, data));
}

void body2World(float* body_frame, float* world_frame, float yaw ){
    world_frame[C_X] = body_frame[C_X] * cosf(yaw) - body_frame[C_Y] * sinf(yaw);
    world_frame[C_Y] = body_frame[C_X] * sinf(yaw) + body_frame[C_Y] * cosf(yaw);
}

void world2Body(float* world_frame, float* body_frame, float yaw){
    body_frame[C_X] = world_frame[C_X] * cosf(yaw) + world_frame[C_Y] * sinf(yaw);
    body_frame[C_Y] = -world_frame[C_X] * sinf(yaw) + world_frame[C_Y] * cosf(yaw);
}