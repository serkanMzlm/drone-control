#include "geometry_utils.hpp"

double mapValues(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

double constrain(float data, float minVal, float maxVal){
  return fminf(maxVal, fmaxf(minVal, data));
}