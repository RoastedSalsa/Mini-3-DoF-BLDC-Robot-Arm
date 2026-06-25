#include "ArmKinematics.h"
#include "config.h"   // L1, L2, L3

bool ik(float x, float y, float z, float& t1, float& t2, float& t3) {
  float r = sqrt(pow(x,2) + pow(y,2));
  t1 =  atan2(y,x);
  t3 =  atan2(-sqrt(1-pow(((pow(r,2)+pow((z-L1),2) -pow(L2,2) - pow(L3,2))/ (2 * L2 * L3)),2) ), (pow(r,2) + pow((z-L1),2) -pow(L2,2) -pow(L3,2))/(2 * L2 * L3));
  t2 =  ( atan2(z - L1, r ) - atan2(L3*sin(t3), L2 + L3*cos(t3)));
  return true;
}

bool fk(float& x, float& y, float& z, float t1, float t2, float t3) {
  x = cos(t1)*(l2*cos(t2)+l3*cos(t2+t3));
  y = sin(t1)*(l2*cos(t2)+l3*cos(t2+t3));
  z = l1 + (l2*sin(t2)+l3*sin(t2+t3));
  return true;
}