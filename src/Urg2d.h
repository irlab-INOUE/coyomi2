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
    LSP(long vd, double vr, double vth, double cs, double sn) {
      this->data = vd;  // [mm]
      this->r = vr;     // [m]
      this->th = vth;   // [rad]
      this->x = vr * cs;  // [m]
      this->y = vr * sn;  // [m]
    };
    LSP(const LSP &lp) {
      data = lp.data;
      r = lp.r;
      th = lp.th;
      x = lp.x;
      y = lp.y;
    };
};

class Urg2d {
  private:
    std::vector<double> ang_list;
    std::vector<double> cos_list;
    std::vector<double> sin_list;
    qrk::Urg_driver urg;
    std::vector<LSP> store_data;

    const int IMG_WIDTH = 400;
    const int IMG_HEIGHT = 400;
    const int IMG_ORIGIN_X = 200;
    const int IMG_ORIGIN_Y = 200;
    const double csize = 12.0/400;
    cv::Mat baseImg;

    bool isConnectionSuccessful = true;
  public:
    Urg2d(const std::string, double, double, double);
    ~Urg2d();
    std::vector<LSP> getData();
    std::vector<LSP> getData(const cv::Mat &imgMap, const Pose2d &pose,
        const int originX, const int originY, const double csize);
    int view(int wait_time);
    int view(const std::string lidar_select, int wait_time);
    void close();
    bool getConnectionSuccessfully();
};

