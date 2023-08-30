#pragma once

#include "yaml-cpp/yaml.h"

#include "Urg2d.h"
#include "WAYPOINT.h"

// DWAのスコアを格納する
struct Score {
	double v;
	double w;
	double heading;
	double velocity;
	double distance;
	double score;
  double obx;
  double oby;
	std::vector<Pose2d> path;

  Score() {
    v = 0;
    w = 0;
  }
};

struct U {
  double v;
  double w;
  double heading;
  double velocity;
  double distance;
  double obstacle_distance;
  double score;
  double obx;   // 最小距離と判断した障害物のローカル座標
  double oby;
  U(double v_, double w_) {
    v = v_;
    w = w_;
    velocity = v;
  };
};

class DynamicWindowApproach {
  private:
    double limit_distance;
    double dv;
    double dw;
    double acceleration_limit;
    double w_acceleration_limit;
    double T;
    double v_max;
    double v_min;
    double w_max;
    double w_min;
    double path_divided_step;
    double alpha;
    double beta;
    double gamma;
    double obstacle_size;
    double deltaT;   //　予測ステップ時間幅

    // Viewer用の設定
    double REAL_WIDTH;
    double REAL_HEIGHT;
    int IMG_WIDTH;
    int IMG_HEIGHT;
    int IMG_ORIGIN_X;
    int IMG_ORIGIN_Y;
    double csize;
    cv::Mat baseImg;

    Score best_score;   // runの後で更新される
    std::vector<LSP> nearby_lsp; // urgの近傍点リストを作成
    std::vector<U> ng;  // 棄却した操作指令

  public:
    DynamicWindowApproach() {};
    DynamicWindowApproach(YAML::Node &coyomi_yaml);
    std::tuple<double, double, double, double> run(const std::vector<LSP> &lsp,
        const Pose2d pose, const double robot_v, const double robot_w,
        const WAYPOINT target);
    void view(const Pose2d pose, const WAYPOINT target, const std::vector<LSP> &lsp);
};
