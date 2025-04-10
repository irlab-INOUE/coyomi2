#ifndef COYOMI_SRC_SIMULATOR_STRATEGY_WAVE_FRONT_PLANNER_H
#define COYOMI_SRC_SIMULATOR_STRATEGY_WAVE_FRONT_PLANNER_H

#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

namespace wavefrontplanner {
class Config {
  public:
    std::string map_path;
    std::string map_info_path;
    double start_x_m;
    double start_y_m;
    double goal_x_m;
    double goal_y_m;

    int index_start_x;
    int index_start_y;
    int index_goal_x;
    int index_goal_y;

    void print() {
      std::cerr << "====================\n"
        << "map_path: " << map_path << "\n"
        << "map_info_path: " << map_info_path << "\n"
        << "index_start_x: " << index_start_x << "\n"
        << "index_start_y: " << index_start_y << "\n"
        << "index_goal_x: " << index_goal_x << "\n"
        << "index_goal_y: " << index_goal_y << "\n"
        << "====================\n";
    };
};

struct OperateIndex {
  int x;
  int y;
};

struct TargetInfo {
  int x;
  int y;
  int base_value;

  TargetInfo(int value_x, int value_y, int value) {
    x = value_x; y = value_y; base_value = value;
  };

  TargetInfo(int value_x, int value_y) {
    x = value_x; y = value_y; base_value = 0;
  };
};

struct MapInfo {
  int  originX;
  int originY;
  double csize;
  int margin;
};

enum class MapStatus {
  kObstacle,
  kUnknown,
  kFree,
  kStart,
  kGoal,
  kPath,
};

class MapState {
  public:
    double center_x_;
    double center_y_;
    double width_;
    double height_;
    MapStatus state_;
    int value_;

    MapState(){};
    MapState(double center_x, double center_y, double width, double height,
             MapStatus state) {
      center_x_ = center_x;
      center_y_ = center_y;
      width_ = width;
      height_ = height;
      state_ = state;
    };
};

struct Path {
  double x;
  double y;
  Path(double x_, double y_) {
    x = x_;
    y = y_;
  }
};


//============================================================================

class WaveFrontPlanner {
  private:
    Config cfg_;
    cv::Mat img_;
    MapInfo map_info_;

    double start_x_m_;
    double start_y_m_;
    double goal_x_m_;
    double goal_y_m_;

    int divided_pixel;

    std::vector<std::vector<MapState>> map_state_;

  public:
    WaveFrontPlanner();
    WaveFrontPlanner(Config &cfg);
    void Init(Config &cfg);
    void SetStartPosition(const double x, const double y);
    void SetGoalPosition(const double x, const double y);
    std::vector<Path> SearchGoal();
};

WaveFrontPlanner::WaveFrontPlanner() {
  // std::cerr << "Hello, wave front planner\n";
  ;
}

WaveFrontPlanner::WaveFrontPlanner(Config &cfg) {
  std::cerr << "Hello, wave front planner\n";
  img_ = cv::imread(cfg.map_path);

	YAML::Node config;
	try {
		config = YAML::LoadFile(cfg.map_info_path);
	} catch(YAML::BadFile &e) {
		std::cout << "read error!" << std::endl;
    exit(0);
	}
  map_info_.margin  = config["margin"].as<int>();
  map_info_.originX = config["originX"].as<int>() + map_info_.margin;
  map_info_.originY = config["originY"].as<int>() + map_info_.margin;
  map_info_.csize   = config["csize"].as<double>();

  cv::Mat occMap = cv::imread(cfg.map_path, cv::IMREAD_GRAYSCALE);
  // 地図をdivided_size (m)区画に分割し，領域の種別に分ける
  // 障害物がある(その区画内に1ピクセル以上) or
  // 未計測(その区画内がすべて未計測の場合)
  // フリースペース
  const double divided_size = 0.5;
  divided_pixel = floor(divided_size / map_info_.csize);
  // gridを確認
  cv::Mat display_grid;
  occMap.copyTo(display_grid);
  for (int ix = 0; ix < display_grid.cols; ix += divided_pixel) {
    cv::line(display_grid, cv::Point(ix, 0), cv::Point(ix, occMap.rows),
        cv::Scalar(100, 100, 100), 1);
  }
  for (int iy = 0; iy < display_grid.rows; iy += divided_pixel) {
    cv::line(display_grid, cv::Point(0, iy), cv::Point(occMap.cols, iy),
        cv::Scalar(100, 100, 100), 1);
  }
  //cv::imshow("GRID", display_grid);
  //cv::waitKey(0);

  int grid_num_width = floor(occMap.cols / divided_pixel) + 1;
  int grid_num_height = floor(occMap.rows / divided_pixel) + 1;
  std::vector<std::vector<MapState>> _state(grid_num_height, std::vector<MapState>(grid_num_width));
  map_state_ = _state;

  for (int iy = 0; iy < occMap.rows; iy += divided_pixel) {
    double center_y = (-iy + map_info_.originY) * map_info_.csize - divided_size/2;
    for (int ix = 0; ix < occMap.cols; ix += divided_pixel) {
      double center_x = (ix - map_info_.originX) * map_info_.csize + divided_size/2;
      bool finish = false;
      for (int sy = 0; sy < divided_pixel; sy++) {
        int check_pixel_y = iy + sy;
        if (check_pixel_y >= occMap.rows) check_pixel_y = occMap.rows - 1;

        for (int sx = 0; sx < divided_pixel; sx++) {
          int check_pixel_x = ix + sx;
          if (check_pixel_x >= occMap.cols) check_pixel_x = occMap.cols - 1;

          int color = occMap.at<uchar>(check_pixel_y, check_pixel_x);

          if (color < 220) {
            MapState state(center_x, center_y, divided_size, divided_size, MapStatus::kObstacle);
            map_state_[floor(static_cast<double>(iy)/divided_pixel)]
                     [floor(static_cast<double>(ix)/divided_pixel)] = state;
            finish = true;
            break;
          }
        }
        if (finish) break;

        // for free space
        MapState state(center_x, center_y, divided_size, divided_size, MapStatus::kFree);
        map_state_[iy/divided_pixel][ix/divided_pixel] = state;
      }
    }
  }
}

void WaveFrontPlanner::Init(Config &cfg) {
  start_x_m_ = cfg.start_x_m;
  start_y_m_ = cfg.start_y_m;

  goal_x_m_ = cfg.goal_x_m;
  goal_y_m_ = cfg.goal_y_m;
}

void WaveFrontPlanner::SetStartPosition(const double x, const double y) {
  start_x_m_ = x;
  start_y_m_ = y;
}

void WaveFrontPlanner::SetGoalPosition(const double x, const double y) {
  goal_x_m_ = x;
  goal_y_m_ = y;
}

std::vector<Path> WaveFrontPlanner::SearchGoal() {
  // start and goal をセット
  int index_start_x = (start_x_m_/map_info_.csize + map_info_.originX) / divided_pixel;
  int index_start_y = (-start_y_m_/map_info_.csize + map_info_.originY) / divided_pixel;
  int index_goal_x = (goal_x_m_/map_info_.csize + map_info_.originX) / divided_pixel;
  int index_goal_y = (-goal_y_m_/map_info_.csize + map_info_.originY) / divided_pixel;

  std::vector<std::vector<MapState>> map_state = map_state_;
  map_state[index_start_y][index_start_x].state_ = MapStatus::kStart;
  map_state[index_goal_y][index_goal_x].state_ = MapStatus::kGoal;

  // map_state に値を割り当てる
  for (int y = 0; y < map_state.size(); y++) {
    for (int x = 0; x < map_state[y].size(); x++) {
      switch (map_state[y][x].state_) {
        case (MapStatus::kObstacle):
          map_state[y][x].value_ = -1;
          break;
        case (MapStatus::kUnknown):
          map_state[y][x].value_ = -1;
          break;
        case (MapStatus::kFree):
          map_state[y][x].value_ = 0;
          break;
        case (MapStatus::kStart):
          map_state[y][x].value_ = 1e9;
          break;
        case (MapStatus::kGoal):
          map_state[y][x].value_ = 1;
          break;
        default:
          break;
      }
    }
  }

  // 探索するインデックスを指すオペレータ
  std::vector<OperateIndex> operator_index(4);
  operator_index[0].x =  0; operator_index[0].y = -1;  // 上
  operator_index[1].x = -1; operator_index[1].y =  0;  // 左
  operator_index[2].x =  0; operator_index[2].y =  1;  // 下
  operator_index[3].x =  1; operator_index[3].y =  0;  // 右

  // 探索開始
  // 最初のターゲットリストを作成する
  std::vector<TargetInfo> target_list;
  TargetInfo start_target(index_goal_x, index_goal_y,
                          map_state[index_goal_y][index_goal_x].value_);
  target_list.emplace_back(start_target);

  for (;;) {
    if (target_list.size() == 0) break;

    std::vector<TargetInfo> next_target_list;
    // 現時点のターゲットリストを上から順に走査する
    for (auto current_target: target_list) {
      // 上下左右を順にチェックする
      for (auto op: operator_index) {
        int search_x = current_target.x + op.x;
        int search_y = current_target.y + op.y;
        if (search_x < 0 || search_x > map_state[0].size() ||
            search_y < 0 || search_y > map_state.size()) continue;
        if (map_state[search_y][search_x].value_ == -1) continue;  // 障害物等
        if (map_state[search_y][search_x].value_ > 0) continue;  // 探索済みなので除去

        map_state[search_y][search_x].value_ = current_target.base_value + 1;

        TargetInfo search_target_state(search_x, search_y,
                                       current_target.base_value + 1);
        next_target_list.emplace_back(search_target_state);
      }
    }
    target_list.clear();
    if (next_target_list.size() > 0) {
      target_list = next_target_list;
    }
    //target_list_print(target_list);
    if (target_list.size() == 0) break;
  }
  //std::cerr << "finish\n";

  // スタートから順にスコアを辿る
  std::vector<TargetInfo> path;
  int current_x = index_start_x;
  int current_y = index_start_y;
  int min_value = 1e9;
  int previous_min_value = min_value;
  for (;;) {
    int min_index_x;
    int min_index_y;
    for (auto op: operator_index) {
        int search_x = current_x + op.x;
        int search_y = current_y + op.y;
        if (search_x < 0 || search_x >= map_state[0].size() ||
            search_y < 0 || search_y >= map_state.size()) continue;
        if (map_state[search_y][search_x].value_ == -1) continue;  // 障害物等

        if (min_value > map_state[search_y][search_x].value_) {
          min_value = map_state[search_y][search_x].value_;
          min_index_x = search_x;
          min_index_y = search_y;
        }
    }
    TargetInfo current_position(min_index_x, min_index_y, min_value);
    path.emplace_back(current_position);
    if ((current_position.x == index_goal_x) &&
        (current_position.y == index_goal_y)) break;

    if (previous_min_value == min_value) {
      //std::cerr << "no path\n";
      break;
    }
    previous_min_value = min_value;

    current_x = current_position.x;
    current_y = current_position.y;
  }
  // pathをstate_にkPath としてセットする
  for (auto pt: path) {
    map_state[pt.y][pt.x].state_ = MapStatus::kPath;
  }
  std::vector<Path> result_path;
  for (auto pt: path) {
    Path path(map_state[pt.y][pt.x].center_x_,
              map_state[pt.y][pt.x].center_y_);
    result_path.emplace_back(path);
  }

  return result_path;
}
}  // wavefrontplanner

#endif // COYOMI_SRC_SIMULATOR_STRATEGY_WAVE_FRONT_PLANNER_H
