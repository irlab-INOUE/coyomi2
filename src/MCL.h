#ifndef __MCL_H__
#define __MCL_H__

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "Pose2X.h"
#include "Urg2d.h"

class MCL {
  private:
    std::vector<Pose2d> particle;
    int NUM;
    cv::RNG rng;
    cv::Mat LFM;

    void read_lfm(std::string fname);

    int originX, originY, margin;
    double csize;

    int draw_index_with_probability(const std::vector<Pose2d>& particle);
    Pose2d sample_motion_model(const Pose2d& particle,
        const Pose2d& curX, const Pose2d& prevX);
    double w_slow = 0;
    double w_fast = 0;
    const double alpha_slow = 0.01;
    const double alpha_fast = 0.1;

  public:
    MCL();
    MCL(const Pose2d pose);
    void set_currentPose(const Pose2d pose);
    void motion_update(double v, double w, double dt);
    void measurement_update(const std::vector<LSP> &lsp);
    double measurement_model(const std::vector<LSP> &lsp,
        const double x, const double y, const double a);
    std::vector<Pose2d> get_particle_set();
    void set_lfm(std::string path);
    void set_mapInfo(std::string path);
    void resampling();
    void KLD_sampling(const std::vector<LSP> &lsp,
        const Pose2d &curX, const Pose2d &prevX);

    Pose2d get_best_pose();
    std::tuple<double, double> get_w();
};
#endif
