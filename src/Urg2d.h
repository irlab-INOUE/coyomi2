#pragma once

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <unistd.h>

#include "Connection_information.h"
#include "Urg_driver.h"

#include "Pose2X.h"

class LSP {
  public:
    long data;
    double r;
    double th;
    double x;
    double y;

    LSP() {;};
    LSP(long vd, double vr, double vth) {
      this->data = vd;  // [mm]
      this->r = vr;     // [m]
      this->th = vth;   // [rad]
      this->x = vr * cos(vth);  // [m]
      this->y = vr * sin(vth);  // [m]
    };
};

class Urg2d {
  private:
    std::vector<double> ang_list;
    std::vector<double> cos_list;
    std::vector<double> sin_list;
    qrk::Urg_driver urg;
    std::vector<LSP> store_data;

    const int IMG_ORIGIN_X = 300;
    const int IMG_ORIGIN_Y = 400;
    const double csize = 6.0/600;
    cv::Mat baseImg;
  public:
    Urg2d();
    ~Urg2d();
    std::vector<LSP> getData();
    std::vector<LSP> getData(const cv::Mat &imgMap, const Pose2d &pose,
        const int originX, const int originY, const double csize);
    int view(int wait_time);
};
