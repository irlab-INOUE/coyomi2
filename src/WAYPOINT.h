#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#include "Pose2X.h"

struct WAYPOINT {
	double x;
	double y;
	double a;
  int stop_check;

  WAYPOINT() { ; };
  WAYPOINT(double vx, double vy) {
    x = vx;
    y = vy;
    a = 0.0;
    stop_check = 0;
  };
  WAYPOINT& operator=(const Pose2d& rhs) {
    x = rhs.x;
    y = rhs.y;
    a = rhs.a;
    stop_check = 0;
    return *this;
  };
};
#endif
