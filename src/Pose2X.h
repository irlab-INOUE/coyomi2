#ifndef __POSE2X_H__
#define __POSE2X_H__

#include <iostream>
#include <cmath>

template<typename T>
class Pose2 {
	public:
		T x;
		T y;
		T a;
    double weight;

		Pose2() { ; };
		Pose2(T _x, T _y, T _a) {
			x = _x; y = _y; a = _a;
		};

		inline void print(std::string str) {
			std::cout << str << " \tx:" << x << " \ty:" << y << " \ta:" << a*180.0/M_PI << std::endl;
		};

		T getX() { return x; };
		T getY() { return y; };
};

using Pose2d = Pose2<double>;
using Pose2i = Pose2<int>;

#endif
