#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <fstream>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "MapPath.h"
#include "WAYPOINT.h"
#include "Pose2X.h"
#include "Urg2d.h"

class Viewer {
  private:
    cv::Mat imgMap;
    cv::Mat AccMap;
    cv::Mat imgMap_original;
    cv::Mat AccMap_original;
    cv::Mat imgMap_hold;
    cv::Mat AccMap_hold;
    double csize;
    int originX, originY;

  public:
    Viewer(MapPath path);
    ~Viewer() {};
    void show();
    void show(int time);
    void show(double x, double y, int time);
    void plot_wp(std::vector<WAYPOINT> wp_list);
    void plot_current_wp(const WAYPOINT &wp);
    void plot_subGoal(std::vector<WAYPOINT> sub_goal);
    void plot_smoothPath(std::vector<WAYPOINT> smooth_path);
    void robot(Pose2d pose);
    void particle(const std::vector<Pose2d> &particle);
    void line(Pose2d pose, WAYPOINT wp);
    double get_csize();
    int get_originX();
    int get_originY();
    cv::Mat get_imgMap_original();
    void hold();
    void reset();
    void clear();
    void urg(const Pose2d pose, const std::vector<LSP> &lsp);
};
#endif
