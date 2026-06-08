#include "ArmKinematics.h"
#include "config.h"   // L1, L2, L3

// Math copied verbatim from full.cpp (only l1/l2/l3 -> L1/L2/L3, and the local
// theta names -> the t1/t2/t3 out-params). Same float types, same operator
// precedence, same signs -> identical results.
bool ik(float x, float y, float z, float& t1, float& t2, float& t3) {
  float r = sqrt(pow(x,2) + pow(y,2));
  t1 =  atan2(x,y) - PI/2;
  t3 =  atan2(-sqrt(1-pow(((pow(r,2)+pow((-z+L1),2) -pow(L2,2) - pow(L3,2))/ (2 * L2 * L3)),2) ), (pow(r,2) + pow((-z+L1),2) -pow(L2,2) -pow(L3,2))/(2 * L2 * L3));
  t2 =  -( atan2(-z + L1, r ) + atan2(L3*sin(t3), L2 + L3*cos(t3)));
  return true;
}
