#ifndef __DE_LFM_H__
#define __DE_LFM_H__

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <random>

#include "yaml-cpp/yaml.h"

#include "Pose2X.h"
#include "Urg2d.h"

class DELFM {
  private:
    cv::Mat LFM;

    std::mt19937 gen;
    std::uniform_real_distribution<> dis;

    void read_lfm(std::string fname);

    int originX, originY, margin;
    int width, height;
    double csize;

    double dth;
    double Wxy, Wa;
    int population_size, generations;
    double F, CR;

  public:
    DELFM();
    DELFM(double Wxy, double Wa, int population_size, int generations, double F, double CR);

    double measurement_model(const std::vector<LSP> &lsp, const double x, const double y, const double a);
    void set_lfm(std::string path);
    void set_mapInfo(std::string path);

    std::tuple<double, double, double, double> optimize_de(std::vector<LSP> &lsp, double current_x, double current_y, double current_a);
};
#endif
