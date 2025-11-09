/**
 * manual_loop_closure.cpp
 * 
 * このプログラムは、slam.cppによって生成された部分地図データを読み込み、
 * ユーザーが手動で地図を移動・回転させてループクロージャを行うためのツールです。
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iomanip> 
#include <string>
#include <algorithm>
#include <random>

namespace fs = std::filesystem;
using OccupancyGrid = std::vector<std::vector<double>>;

// slam.cppから移植した定義
#define GREEN cv::Scalar(118,209,173)
#define BACK_BLUE cv::Scalar(47,35,30)
#define BLUE_LIGHT cv::Scalar(255,218,0)
#define WHITE cv::Scalar(198,204,203)
#define RED cv::Scalar(0,31,245)

// slam.cppから移植した構造体
struct Point {
  double x, y;
  Point() : x(0), y(0) {};
  Point(double x, double y) : x(x), y(y) {}
};

struct Pose {
  long long ts;
  double x, y, a;
  Pose() : ts(0), x(0), y(0), a(0) {};
  Pose(long long ts, double x, double y, double a) : ts(ts), x(x), y(y), a(a) {}
};

struct SubMap {
  int submap_id;
  Pose start_pose;  // グローバル座標系での部分地図の基準姿勢
  OccupancyGrid local_gmap; // 部分地図内のローカルな占有格子地図

  // 地図の物理的・ピクセル的情報
  double min_x, max_x, min_y, max_y; // ローカル座標系でのバウンディングボックス
  double LOCAL_CSIZE;
  int LOCAL_WIDTH, LOCAL_HEIGHT, LOCAL_ORIGIN_X, LOCAL_ORIGIN_Y;

  // 読み込み成功フラグ
  bool is_valid = false;
};

// slam.cppから移植したユーティリティ関数
constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double normalize_th(double ra) {
  while(ra > M_PI) ra -= 2*M_PI;
  while(ra < -M_PI) ra += 2*M_PI;
  return ra;
}

// 関数のプロトタイプ宣言
std::vector<SubMap> load_all_submaps(const std::string& root_dir);
void visualize_maps(const std::vector<SubMap>& submaps, int selected_id, const double CSIZE, 
                    const bool id_selection_mode);

// slam.cppから移植したGaussianKernelクラス
class GaussianKernel {
private:
  std::vector<std::vector<double>> kernel;
  int radius;

public:
  GaussianKernel(double sigma, int kernel_radius) : radius(kernel_radius) {
    int size = 2 * radius + 1;
    kernel.resize(size, std::vector<double>(size));

    double sum = 0.0;
    double sigma2 = sigma * sigma;
    for (int y = -radius; y <= radius; y++) {
      for (int x = -radius; x <= radius; x++) {
        double distance_sq = x*x + y*y;
        double weight = exp(-distance_sq / (2.0 * sigma2));
        kernel[y + radius][x + radius] = weight;
        sum += weight;
      }
    }

    // 正規化
    for (int y = 0; y < size; y++) {
      for (int x = 0; x < size; x++) {
        kernel[y][x] /= sum;
      }
    }
  }

  double getWeight(int dx, int dy) const {
    if (abs(dx) > radius || abs(dy) > radius) return 0.0;
    return kernel[dy + radius][dx + radius];
  }

  int getRadius() const { return radius; }
};

// slam.cppから移植したxy2index関数
std::tuple<int, int> xy2index(double xd, double yd, double CSIZE, int ox, int oy) {
  int ix = static_cast<int>( xd / CSIZE) + ox;
  int iy = static_cast<int>(-yd / CSIZE) + oy;
  return std::make_tuple(ix, iy);
}

// slam.cppから移植したmatch_count関数
double match_count(OccupancyGrid &gmap,
                   std::vector<Point> &pt,
                   double cx, double cy, double ca,
                   double CSIZE, int originX, int originY, int width, int height) {
  double eval = 0;
  double cs = cos(ca);
  double sn = sin(ca);
  for (int ind = 0; ind < pt.size(); ind += 1) {
    double xd = pt[ind].x * cs - pt[ind].y * sn + cx;
    double yd = pt[ind].x * sn + pt[ind].y * cs + cy;
    auto [ix, iy] = xy2index(xd, yd, CSIZE, originX, originY);
    if (2 <= ix && ix < width-2 && 2 <= iy && iy < height-2) {
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          if (gmap[iy + dy][ix + dx] > 0.0)
            eval += (1.0/(1+abs(dx) + abs(dy)));
        }
      }
    }
  }
  const double MAX_SCORE_PER_POINT = 13.0/3.0;
  const double MAX_LIDAR_POINTS = 1081.0;
  const double THEORETICAL_MAX = MAX_LIDAR_POINTS * MAX_SCORE_PER_POINT;
  return eval / THEORETICAL_MAX;
}

// slam.cppから移植したgaussian_match_count関数
double gaussian_match_count(const OccupancyGrid& gmap,
                            const std::vector<Point>& pt,
                            double cx, double cy, double ca,
                            double CSIZE, int originX, int originY, 
                            int width, int height,
                            const GaussianKernel& kernel) {
  double total_score = 0.0;
  double cs = cos(ca);
  double sn = sin(ca);

  for (int idx = 0; idx < pt.size(); idx++) {
    const auto& p = pt[idx];
    double xd = p.x * cs - p.y * sn + cx;
    double yd = p.x * sn + p.y * cs + cy;
    auto [gx, gy] = xy2index(xd, yd, CSIZE, originX, originY);

    double point_score = 0.0;
    for (int dy = -kernel.getRadius(); dy <= kernel.getRadius(); dy++) {
      for (int dx = -kernel.getRadius(); dx <= kernel.getRadius(); dx++) {
        int map_x = gx + dx;
        int map_y = gy + dy;

        if (map_x >= 0 && map_x < width && map_y >= 0 && map_y < height) {
          if (gmap[map_y][map_x] > 0.0) {
            point_score += kernel.getWeight(dx, dy);
          }
        }
      }
    }
    total_score += point_score;
  }
  return total_score;
}

// slam.cppから移植したoptimize_de関数
std::random_device rd; 
std::mt19937 gen(rd()); 
std::uniform_real_distribution<> dis(-1.0, 1.0);

std::tuple<double, double, double, double> optimize_de(
  OccupancyGrid &gmap, std::vector<Point> &pt,
  double current_x, double current_y, double current_a,
  int originX, int originY, 
  double CSIZE, double dth,
  int width, int height, 
  double Wxy, double Wa, 
  int population_size, int generations, double F, double CR,
  const GaussianKernel* kernel = nullptr) {
  double prev_x = current_x;
  double prev_y = current_y;
  double prev_a = current_a;
  while (1) {
    std::vector<std::tuple<double, double, double>> population(population_size);
    for (int i = 0; i < population_size; i++) {
      double x = current_x + dis(gen) * Wxy;
      double y = current_y + dis(gen) * Wxy;
      double a = current_a + dis(gen) * Wa;
      population[i] = std::make_tuple(x, y, a);
    }

    double best_eval = -std::numeric_limits<double>::infinity();
    double best_x = current_x, best_y = current_y, best_a = current_a;

    std::uniform_int_distribution<> dist(0, population_size - 1);
    for (int generation = 0; generation < generations; generation++) {
      for (int i = 0; i < population_size; i++) {
        int r1, r2, r3;
        do r1 = dist(gen);
        while (r1 == i);
        do r2 = dist(gen);
        while (r2 == i || r2 == r1);
        do r3 = dist(gen);
        while (r3 == i || r3 == r1 || r3 == r2);

        auto [x1, y1, a1] = population[r1];
        auto [x2, y2, a2] = population[r2];
        auto [x3, y3, a3] = population[r3];

        double vx = x1 + F * (x2 - x3);
        double vy = y1 + F * (y2 - y3);

        double ax1 = cos(a1), ay1 = sin(a1);
        double ax2 = cos(a2), ay2 = sin(a2);
        double ax3 = cos(a3), ay3 = sin(a3);
        double vax = ax1 + F * (ax2 - ax3);
        double vay = ay1 + F * (ay2 - ay3);
        double va = atan2(vay, vax);

        double trial_x = vx;
        double trial_y = vy;
        double trial_a = va;
        for (int j = 0; j < 3; j++) {
          if (dis(gen) > CR) {
            if (j == 0) trial_x = std::get<0>(population[i]);
            if (j == 1) trial_y = std::get<1>(population[i]);
            if (j == 2) trial_a = std::get<2>(population[i]);
          }
        }

        double eval;
        if (kernel != nullptr) {
          eval = gaussian_match_count(gmap, pt, trial_x, trial_y, trial_a, CSIZE, originX, originY, width, height, *kernel);
        } else {
          eval = match_count(gmap, pt, trial_x, trial_y, trial_a, CSIZE, originX, originY, width, height);
        }

        if (eval > best_eval) {
          best_eval = eval;
          best_x = trial_x;
          best_y = trial_y;
          best_a = trial_a;
        }

        double current_eval;
        if (kernel != nullptr) {
          current_eval = gaussian_match_count(gmap, pt, std::get<0>(population[i]), std::get<1>(population[i]), std::get<2>(population[i]), CSIZE, originX, originY, width, height, *kernel);
        } else {
          current_eval = match_count(gmap, pt, std::get<0>(population[i]), std::get<1>(population[i]), std::get<2>(population[i]), CSIZE, originX, originY, width, height);
        }
        if (eval > current_eval) {
          population[i] = std::make_tuple(trial_x, trial_y, trial_a);
        }
      }
    }

    return std::make_tuple(best_x, best_y, best_a, best_eval);
  }
}


// OccupancyGridから点群を生成するヘルパー関数
std::vector<Point> create_point_cloud_from_grid(const SubMap& submap) {
  std::vector<Point> points;
  for (int y = 0; y < submap.LOCAL_HEIGHT; ++y) {
    for (int x = 0; x < submap.LOCAL_WIDTH; ++x) {
      if (submap.local_gmap[y][x] > 0.4) { // 占有されているセルを点として抽出
        // ピクセル座標を物理座標（サブマップのローカル座標系）に変換
        double px = (x - submap.LOCAL_ORIGIN_X) * submap.LOCAL_CSIZE;
        double py = -(y - submap.LOCAL_ORIGIN_Y) * submap.LOCAL_CSIZE; // y軸反転
        points.emplace_back(px, py);
      }
    }
  }
  return points;
}

// スキャンマッチングを実行する関数
Pose perform_scan_matching(const SubMap& fixed_submap, const SubMap& moving_submap) {
  // 1. fixed_submapのlocal_gmapをターゲット地図とする
  OccupancyGrid& target_gmap = const_cast<OccupancyGrid&>(fixed_submap.local_gmap);
  int target_width = fixed_submap.LOCAL_WIDTH;
  int target_height = fixed_submap.LOCAL_HEIGHT;
  int target_origin_x = fixed_submap.LOCAL_ORIGIN_X;
  int target_origin_y = fixed_submap.LOCAL_ORIGIN_Y;
  double target_csize = fixed_submap.LOCAL_CSIZE;

  // 2. moving_submapのlocal_gmapから点群を生成し、ソース点群とする
  std::vector<Point> source_points = create_point_cloud_from_grid(moving_submap);

  // 3. optimize_deの初期姿勢を計算
  //    fixed_submapのstart_poseを基準としたmoving_submapの相対姿勢を初期推定値とする
  double initial_x_rel = moving_submap.start_pose.x - fixed_submap.start_pose.x;
  double initial_y_rel = moving_submap.start_pose.y - fixed_submap.start_pose.y;
  double initial_a_rel = normalize_th(moving_submap.start_pose.a - fixed_submap.start_pose.a);

  //    fixed_submapのローカル座標系に変換
  double cos_fixed_a_inv = cos(-fixed_submap.start_pose.a);
  double sin_fixed_a_inv = sin(-fixed_submap.start_pose.a);
  double initial_x = initial_x_rel * cos_fixed_a_inv - initial_y_rel * sin_fixed_a_inv;
  double initial_y = initial_x_rel * sin_fixed_a_inv + initial_y_rel * cos_fixed_a_inv;
  double initial_a = initial_a_rel; // 角度はそのまま

  // optimize_deのパラメータ
  double Wxy = 1.0; // 探索範囲 [m]
  double Wa = deg2rad(10.0); // 探索角度 [rad]
  int population_size = 100;
  int generations = 50;
  double F = 0.5;
  double CR = 0.2;
  GaussianKernel gaussian_kernel(0.8, 2);

  double best_x_local, best_y_local, best_a_local, best_eval;
  std::tie(best_x_local, best_y_local, best_a_local, best_eval) = optimize_de(
    target_gmap, source_points,
    initial_x, initial_y, initial_a,
    target_origin_x, target_origin_y, target_csize,
    deg2rad(0.25), // dth (LiDAR resolution, can be fixed or passed)
    target_width, target_height,
    Wxy, Wa,
    population_size, generations, F, CR,
    &gaussian_kernel);

  // optimize_deが返した姿勢はfixed_submapのローカル座標系におけるsource_pointsの姿勢
  // これをグローバル座標系に戻す
  Pose result_pose;

  double cos_fixed_a = cos(fixed_submap.start_pose.a);
  double sin_fixed_a = sin(fixed_submap.start_pose.a);

  result_pose.x = best_x_local * cos_fixed_a - best_y_local * sin_fixed_a + fixed_submap.start_pose.x;
  result_pose.y = best_x_local * sin_fixed_a + best_y_local * cos_fixed_a + fixed_submap.start_pose.y;
  result_pose.a = normalize_th(best_a_local + fixed_submap.start_pose.a);

  std::cout << "スキャンマッチング結果 (グローバル): x=" << result_pose.x 
    << ", y=" << result_pose.y 
    << ", a=" << rad2deg(result_pose.a) << " deg" 
    << " (評価値: " << best_eval << ")" << std::endl;

  return result_pose;
}


// スキャンマッチング結果を個別に可視化する関数
void visualize_scan_match_result(const SubMap& fixed_submap, const SubMap& moving_submap, const Pose& matched_pose) {
  const double CSIZE = 0.05; // 解像度

  // 1. 描画範囲を計算 (fixed_submap と matched_pose で変換された moving_submap)
  double global_min_x = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_y = std::numeric_limits<double>::lowest();

  // fixed_submapの範囲
  double cos_fixed_a = cos(fixed_submap.start_pose.a);
  double sin_fixed_a = sin(fixed_submap.start_pose.a);
  std::vector<Point> fixed_corners = {
    {fixed_submap.min_x, fixed_submap.min_y}, {fixed_submap.max_x, fixed_submap.min_y},
    {fixed_submap.max_x, fixed_submap.max_y}, {fixed_submap.min_x, fixed_submap.max_y}
  };
  for (const auto& corner : fixed_corners) {
    double rot_x = corner.x * cos_fixed_a - corner.y * sin_fixed_a;
    double rot_y = corner.x * sin_fixed_a + corner.y * cos_fixed_a;
    double glob_x = fixed_submap.start_pose.x + rot_x;
    double glob_y = fixed_submap.start_pose.y + rot_y;
    if (glob_x < global_min_x) global_min_x = glob_x;
    if (glob_x > global_max_x) global_max_x = glob_x;
    if (glob_y < global_min_y) global_min_y = glob_y;
    if (glob_y > global_max_y) global_max_y = glob_y;
  }

  // matched_poseで変換されたmoving_submapの範囲
  double cos_matched_a = cos(matched_pose.a);
  double sin_matched_a = sin(matched_pose.a);
  std::vector<Point> moving_corners = {
    {moving_submap.min_x, moving_submap.min_y}, {moving_submap.max_x, moving_submap.min_y},
    {moving_submap.max_x, moving_submap.max_y}, {moving_submap.min_x, moving_submap.max_y}
  };
  for (const auto& corner : moving_corners) {
    double rot_x = corner.x * cos_matched_a - corner.y * sin_matched_a;
    double rot_y = corner.x * sin_matched_a + corner.y * cos_matched_a;
    double glob_x = matched_pose.x + rot_x;
    double glob_y = matched_pose.y + rot_y;
    if (glob_x < global_min_x) global_min_x = glob_x;
    if (glob_x > global_max_x) global_max_x = glob_x;
    if (glob_y < global_min_y) global_min_y = glob_y;
    if (glob_y > global_max_y) global_max_y = glob_y;
  }

  double margin = 5.0; // マージン
  global_min_x -= margin; global_max_x += margin;
  global_min_y -= margin; global_max_y += margin;

  // 2. 描画用イメージを作成
  int img_w = static_cast<int>((global_max_x - global_min_x) / CSIZE);
  int img_h = static_cast<int>((global_max_y - global_min_y) / CSIZE);
  cv::Mat result_img = cv::Mat::zeros(img_h, img_w, CV_8UC3);
  result_img.setTo(BACK_BLUE);

  // 3. fixed_submapを描画 (白)
  for (int y = 0; y < fixed_submap.LOCAL_HEIGHT; ++y) {
    for (int x = 0; x < fixed_submap.LOCAL_WIDTH; ++x) {
      if (fixed_submap.local_gmap[y][x] > 0.4) {
        double local_x = (x - fixed_submap.LOCAL_ORIGIN_X) * fixed_submap.LOCAL_CSIZE;
        double local_y = -(y - fixed_submap.LOCAL_ORIGIN_Y) * fixed_submap.LOCAL_CSIZE;
        double rot_x = local_x * cos_fixed_a - local_y * sin_fixed_a;
        double rot_y = local_x * sin_fixed_a + local_y * cos_fixed_a;
        double glob_x = fixed_submap.start_pose.x + rot_x;
        double glob_y = fixed_submap.start_pose.y + rot_y;
        int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
        int py = static_cast<int>((global_max_y - glob_y) / CSIZE);
        if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
          result_img.at<cv::Vec3b>(py, px) = cv::Vec3b(198, 204, 203);
        }
      }
    }
  }

  // 4. matched_poseで変換されたmoving_submapを描画 (緑)
  for (int y = 0; y < moving_submap.LOCAL_HEIGHT; ++y) {
    for (int x = 0; x < moving_submap.LOCAL_WIDTH; ++x) {
      if (moving_submap.local_gmap[y][x] > 0.4) {
        double local_x = (x - moving_submap.LOCAL_ORIGIN_X) * moving_submap.LOCAL_CSIZE;
        double local_y = -(y - moving_submap.LOCAL_ORIGIN_Y) * moving_submap.LOCAL_CSIZE;
        double rot_x = local_x * cos_matched_a - local_y * sin_matched_a;
        double rot_y = local_x * sin_matched_a + local_y * cos_matched_a;
        double glob_x = matched_pose.x + rot_x;
        double glob_y = matched_pose.y + rot_y;
        int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
        int py = static_cast<int>((global_max_y - glob_y) / CSIZE);
        if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
          result_img.at<cv::Vec3b>(py, px) = cv::Vec3b(118, 209, 173);
        }
      }
    }
  }

  // 5. ウィンドウに表示
  cv::imshow("Scan Match Result", result_img);
  cv::waitKey(0); // ユーザーが閉じるまで待機
}


/******************************************************
* MAIN
*******************************************************/
int main(int argc, char *argv[]) {
  const double CSIZE = 0.05;     // [m] 格子の解像度 slam.cppと合わせる

  if (argc < 2) {
    std::cerr << "使用法: " << argv[0] << " <slam_result_directory>" << std::endl;
    return 1;
  }
  std::string result_dir = argv[1];

  // 1. 全ての部分地図データをファイルから読み込む
  std::vector<SubMap> submaps = load_all_submaps(result_dir);
  if (submaps.empty()) {
    std::cerr << "部分地図を読み込めませんでした。ディレクトリを確認してください: " << result_dir << std::endl;
    return 1;
  }

  int selected_submap_id = 0; // 現在選択されている部分地図のID
  bool needs_redraw = true;   // 再描画が必要かどうかのフラグ
  bool id_selection_mode = false; // ID選択モードのフラグ
  int temp_highlighted_id = 0; // ID選択モード中にハイライトされるID

  bool scan_match_mode = false; // スキャンマッチングモードのフラグ
  int id_match_1 = -1;          // マッチング対象1つ目のID
  int id_match_2 = -1;          // マッチング対象2つ目のID

  std::cout << "操作方法:" << "\n";
  std::cout << "  -        i: ID選択モードに入る (n/pで移動, Enterで決定, Escでキャンセル)" << "\n";
  std::cout << "  -        m: スキャンマッチングモードに入る (1つ目選択 -> 2つ目選択 -> マッチング実行)" << "\n";
  std::cout << "  -     hjkl: 選択した地図以降を平行移動" << "\n";
  std::cout << "  -      r/t: 選択した地図以降を回転 (r:反時計回り, t:時計回り)" << "\n";
  std::cout << "  -        s: 現在の変更を保存（未実装）" << "\n";
  std::cout << "  -        q: 終了" << std::endl;

  // 2. インタラクションループ
  while (true) {
    int id_to_highlight = selected_submap_id; // 通常は選択中のIDをハイライト
    if (id_selection_mode || scan_match_mode) {
      id_to_highlight = temp_highlighted_id; // モード中はハイライト中のID
    }

    if (needs_redraw) {
      visualize_maps(submaps, id_to_highlight, CSIZE, id_selection_mode);
      needs_redraw = false;
    }

    int key = cv::waitKey(0); // キー入力を待つ

    double dx = 0, dy = 0, da = 0; // 移動量

    if (id_selection_mode) {
      switch (key) {
        case 'n': // Next ID
          temp_highlighted_id = (temp_highlighted_id + 1) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 'p': // Previous ID
          temp_highlighted_id = (temp_highlighted_id - 1 + submaps.size()) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 13: // Enter key (ASCII for CR)
          selected_submap_id = temp_highlighted_id;
          id_selection_mode = false;
          std::cout << "部分地図 " << selected_submap_id << " を選択しました。" << std::endl;
          needs_redraw = true;
          break;
        case 27: // Escape key
          id_selection_mode = false;
          std::cout << "ID選択モードをキャンセルしました。" << std::endl;
          needs_redraw = true;
          break;
        default:
          // Ignore other keys in selection mode
          break;
      }
    } else if (scan_match_mode) { // スキャンマッチングモード
      // プロンプト表示
      if (id_match_1 == -1) {
        std::cout << "スキャンマッチング: 1つ目の部分地図IDを選択してください (n/pで移動, Enterで決定, Escでキャンセル)。" << std::endl;
      } else {
        std::cout << "スキャンマッチング: 2つ目の部分地図IDを選択してください (n/pで移動, Enterで決定, Escでキャンセル)。" << std::endl;
      }

      switch (key) {
        case 'n': // Next ID
          temp_highlighted_id = (temp_highlighted_id + 1) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 'p': // Previous ID
          temp_highlighted_id = (temp_highlighted_id - 1 + submaps.size()) % submaps.size();
          std::cout << "ハイライトID: " << temp_highlighted_id << std::endl;
          needs_redraw = true;
          break;
        case 13: // Enter key
          if (id_match_1 == -1) {
            id_match_1 = temp_highlighted_id;
            std::cout << "1つ目の部分地図ID: " << id_match_1 << " を選択しました。" << std::endl;
            temp_highlighted_id = (id_match_1 + 1) % submaps.size(); // 2つ目は1つ目の次から開始
            if (temp_highlighted_id == id_match_1) temp_highlighted_id = (temp_highlighted_id + 1) % submaps.size(); // 自身は避ける
            needs_redraw = true;
          } else {
            id_match_2 = temp_highlighted_id;
            std::cout << "2つ目の部分地図ID: " << id_match_2 << " を選択しました。" << std::endl;

            if (id_match_1 == id_match_2) {
              std::cerr << "エラー: 同じ部分地図を選択することはできません。" << std::endl;
              id_match_1 = -1; // リセット
              id_match_2 = -1;
              scan_match_mode = false;
              needs_redraw = true;
              break;
            }

            // 固定側と移動側を決定
            const SubMap* fixed_map_ptr = nullptr;
            SubMap* moving_map_ptr = nullptr;
            int actual_moving_id = -1;

            if (id_match_1 == 0) { // 1つ目が0なら、0を固定、2つ目を移動
              fixed_map_ptr = &submaps[id_match_1];
              moving_map_ptr = &submaps[id_match_2];
              actual_moving_id = id_match_2;
            } else if (id_match_2 == 0) { // 2つ目が0なら、0を固定、1つ目を移動
              fixed_map_ptr = &submaps[id_match_2];
              moving_map_ptr = &submaps[id_match_1];
              actual_moving_id = id_match_1;
            } else { // どちらも0以外なら、1つ目を固定、2つ目を移動
              fixed_map_ptr = &submaps[id_match_1];
              moving_map_ptr = &submaps[id_match_2];
              actual_moving_id = id_match_2;
            }

            if (fixed_map_ptr && moving_map_ptr) {
              std::cout << "スキャンマッチング: 固定側ID=" << fixed_map_ptr->submap_id
                << ", 移動側ID=" << moving_map_ptr->submap_id << " を実行中..." << std::endl;
              Pose matched_pose = perform_scan_matching(*fixed_map_ptr, *moving_map_ptr);
              moving_map_ptr->start_pose = matched_pose;
              std::cout << "部分地図 " << actual_moving_id << " をマッチング結果に移動しました。" << std::endl;

              // スキャンマッチング結果を個別に表示
              visualize_scan_match_result(*fixed_map_ptr, *moving_map_ptr, matched_pose);
            } else {
              std::cerr << "エラー: スキャンマッチングの対象が不正です。" << std::endl;
            }

            id_match_1 = -1; // リセット
            id_match_2 = -1;
            scan_match_mode = false;
            needs_redraw = true;
          }
          break;
        case 27: // Escape key
          id_match_1 = -1; // リセット
          id_match_2 = -1;
          scan_match_mode = false;
          std::cout << "スキャンマッチングモードをキャンセルしました。" << std::endl;
          needs_redraw = true;
          break;
        default:
          // Ignore other keys in selection mode
          break;
      }
    } else { // Normal mode
      switch (key) {
        case 'i': // Enter ID selection mode
          id_selection_mode = true;
          temp_highlighted_id = selected_submap_id; // Start highlighting from current selection
          std::cout << "ID選択モードに入りました (n/pで移動, Enterで決定, Escでキャンセル)。" << std::endl;
          needs_redraw = true;
          break;

        case 'm': // Enter Scan Matching mode
          scan_match_mode = true;
          id_match_1 = -1; // Reset selections
          id_match_2 = -1;
          temp_highlighted_id = selected_submap_id; // Start highlighting from current selection
          std::cout << "スキャンマッチングモードに入りました (1つ目選択 -> 2つ目選択 -> マッチング実行)。" << std::endl;
          needs_redraw = true;
          break;

        // --- 移動・回転 ---
        case 'k': dy = -0.5; break; // Up
        case 'K': dy = -1.0; break; // Up
        case 'j': dy =  0.5; break;  // Down
        case 'J': dy =  1.0; break;  // Down
        case 'h': dx = -0.5; break; // Left
        case 'H': dx = -1.0; break; // Left
        case 'l': dx =  0.5; break;  // Right
        case 'L': dx =  1.0; break;  // Right
        case 'r': da = deg2rad(1.0); break; // 1度回転
        case 't': da = -deg2rad(1.0); break; // -1度回転

        // --- 終了 ---
        case 'q':
          std::cout << "終了します。" << std::endl;
          return 0;

        // --- 保存（未実装） ---
        case 's':
          std::cout << "保存機能は未実装です。" << std::endl;
          break;
      }
    }

    // 3. 連動ロジックの適用
    if (dx != 0 || dy != 0 || da != 0) {
      // 回転の中心を選択されたサブマップの原点に設定
      const double center_x = submaps[selected_submap_id].start_pose.x;
      const double center_y = submaps[selected_submap_id].start_pose.y;
      const double c = cos(da);
      const double s = sin(da);

      for (int i = std::max(selected_submap_id, 1); i < submaps.size(); ++i) {
        // 回転処理 (選択された地図の中心周り)
        double translated_x = submaps[i].start_pose.x - center_x;
        double translated_y = submaps[i].start_pose.y - center_y;

        double rotated_x = translated_x * c - translated_y * s;
        double rotated_y = translated_x * s + translated_y * c;

        submaps[i].start_pose.x = rotated_x + center_x;
        submaps[i].start_pose.y = rotated_y + center_y;

        // 平行移動処理
        submaps[i].start_pose.x += dx;
        submaps[i].start_pose.y += dy;

        // 角度の更新
        submaps[i].start_pose.a = normalize_th(submaps[i].start_pose.a + da);
      }
      needs_redraw = true;
    }
  }

  return 0;
}
// End of MAIN


// YAMLファイルから特定のキーの値を取得する、より堅牢なヘルパー関数
double get_yaml_value(const std::string& path, const std::string& key_to_find) {
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string key_part;
    ss >> key_part;
    if (key_part == key_to_find + ":") {
      double value;
      ss >> value;
      if (!ss.fail()) {
        return value;
      }
    }
  }
  return 0.0; // 見つからなかった、またはパース失敗
}

// 単一の部分地図を読み込むヘルパー関数
SubMap load_single_submap(const std::string& submap_dir) {
  SubMap submap;
  std::string map_info_path = submap_dir + "/mapInfo.yaml";
  std::string gmap_path = submap_dir + "/local_gmap.yml";

  if (!fs::exists(map_info_path) || !fs::exists(gmap_path)) {
    return submap; // 必要なファイルがない
  }

  // 1. mapInfo.yamlからメタデータを読み込む
  submap.submap_id = static_cast<int>(get_yaml_value(map_info_path, "submap_id"));
  submap.start_pose.x = get_yaml_value(map_info_path, "x");
  submap.start_pose.y = get_yaml_value(map_info_path, "y");
  submap.start_pose.a = get_yaml_value(map_info_path, "a");
  submap.min_x = get_yaml_value(map_info_path, "min_x");
  submap.max_x = get_yaml_value(map_info_path, "max_x");
  submap.min_y = get_yaml_value(map_info_path, "min_y");
  submap.max_y = get_yaml_value(map_info_path, "max_y");

  // 2. local_gmap.ymlから地図データを読み込む
  cv::FileStorage fs(gmap_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return submap;
  }
  cv::Mat gmap_mat;
  fs["local_gmap"] >> gmap_mat;
  fs["width"] >> submap.LOCAL_WIDTH;
  fs["height"] >> submap.LOCAL_HEIGHT;
  fs["origin_x"] >> submap.LOCAL_ORIGIN_X;
  fs["origin_y"] >> submap.LOCAL_ORIGIN_Y;
  fs["cell_size"] >> submap.LOCAL_CSIZE;
  fs.release();

  // cv::MatをOccupancyGridに変換
  submap.local_gmap.resize(gmap_mat.rows, std::vector<double>(gmap_mat.cols));
  for (int r = 0; r < gmap_mat.rows; ++r) {
    for (int c = 0; c < gmap_mat.cols; ++c) {
      submap.local_gmap[r][c] = gmap_mat.at<double>(r, c);
    }
  }

  submap.is_valid = true;
  return submap;
}

// 全ての部分地図を読み込むメイン関数
std::vector<SubMap> load_all_submaps(const std::string& root_dir) {
  std::string submaps_path = root_dir + "/submaps";
  std::vector<SubMap> submaps;

  if (!fs::exists(submaps_path) || !fs::is_directory(submaps_path)) {
    return submaps;
  }

  // ディレクトリ名をソートするために一時的なベクターに格納
  std::vector<std::string> dir_paths;
  for (const auto& entry : fs::directory_iterator(submaps_path)) {
    if (entry.is_directory()) {
      dir_paths.push_back(entry.path().string());
    }
  }
  std::sort(dir_paths.begin(), dir_paths.end());

  // ソートされたパスからサブマップを読み込む
  for (const auto& path : dir_paths) {
    std::cout << path << " を読み込み中..." << std::endl;
    SubMap submap = load_single_submap(path);
    if (submap.is_valid) {
      submaps.push_back(submap);
    }
  }

  return submaps;
}

void visualize_maps(const std::vector<SubMap>& submaps, int selected_id, double CSIZE,
                    const bool id_selection_mode) {
  if (submaps.empty()) return;

  // 1. 全体地図の描画範囲を計算
  double global_min_x = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_y = std::numeric_limits<double>::lowest();

  for (const auto& submap : submaps) {
    double cos_a = cos(submap.start_pose.a);
    double sin_a = sin(submap.start_pose.a);
    std::vector<Point> corners = {
      {submap.min_x, submap.min_y}, {submap.max_x, submap.min_y},
      {submap.max_x, submap.max_y}, {submap.min_x, submap.max_y}
    };
    for (const auto& corner : corners) {
      double rot_x = corner.x * cos_a - corner.y * sin_a;
      double rot_y = corner.x * sin_a + corner.y * cos_a;
      double glob_x = submap.start_pose.x + rot_x;
      double glob_y = submap.start_pose.y + rot_y;
      if (glob_x < global_min_x) global_min_x = glob_x;
      if (glob_x > global_max_x) global_max_x = glob_x;
      if (glob_y < global_min_y) global_min_y = glob_y;
      if (glob_y > global_max_y) global_max_y = glob_y;
    }
  }
  double margin = 10.0; // 10mマージン
  global_min_x -= margin; global_max_x += margin;
  global_min_y -= margin; global_max_y += margin;

  // 2. 描画用イメージを作成
  int img_w = static_cast<int>((global_max_x - global_min_x) / CSIZE);
  int img_h = static_cast<int>((global_max_y - global_min_y) / CSIZE);
  cv::Mat integrated_img = cv::Mat::zeros(img_h, img_w, CV_8UC3);
  integrated_img.setTo(BACK_BLUE);

  // 3. 各部分地図を描画
  for (const auto& submap : submaps) {
    double cos_a = cos(submap.start_pose.a);
    double sin_a = sin(submap.start_pose.a);
    for (int y = 0; y < submap.LOCAL_HEIGHT; ++y) {
      for (int x = 0; x < submap.LOCAL_WIDTH; ++x) {
        if (submap.local_gmap[y][x] > 0.4) { // 占有確率が高いグリッドのみ描画
          double local_x = (x - submap.LOCAL_ORIGIN_X) * submap.LOCAL_CSIZE;
          double local_y = -(y - submap.LOCAL_ORIGIN_Y) * submap.LOCAL_CSIZE;
          double rot_x = local_x * cos_a - local_y * sin_a;
          double rot_y = local_x * sin_a + local_y * cos_a;
          double glob_x = submap.start_pose.x + rot_x;
          double glob_y = submap.start_pose.y + rot_y;
          int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
          int py = static_cast<int>((global_max_y - glob_y) / CSIZE);
          if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
            integrated_img.at<cv::Vec3b>(py, px) = cv::Vec3b(198, 204, 203);
          }
        }
      }
    }
  }

  // 3.1 idセレクトモードで，ハイライトされている部分地図を描画
  if (id_selection_mode) {
    double cos_a = cos(submaps[selected_id].start_pose.a);
    double sin_a = sin(submaps[selected_id].start_pose.a);
    for (int y = 0; y < submaps[selected_id].LOCAL_HEIGHT; ++y) {
      for (int x = 0; x < submaps[selected_id].LOCAL_WIDTH; ++x) {
        if (submaps[selected_id].local_gmap[y][x] > 0.4) { // 占有確率が高いグリッドのみ描画
          double local_x = (x - submaps[selected_id].LOCAL_ORIGIN_X) * submaps[selected_id].LOCAL_CSIZE;
          double local_y = -(y - submaps[selected_id].LOCAL_ORIGIN_Y) * submaps[selected_id].LOCAL_CSIZE;
          double rot_x = local_x * cos_a - local_y * sin_a;
          double rot_y = local_x * sin_a + local_y * cos_a;
          double glob_x = submaps[selected_id].start_pose.x + rot_x;
          double glob_y = submaps[selected_id].start_pose.y + rot_y;
          int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
          int py = static_cast<int>((global_max_y - glob_y) / CSIZE);
          if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
            cv::circle(integrated_img, cv::Point(px, py), 5, GREEN, -1); // 塗りつぶし
          }
        }
      }
    }
  }

  // 4. 各部分地図の目印（ID付きの円）を描画
  for (const auto& submap : submaps) {
    double glob_x = submap.start_pose.x;
    double glob_y = submap.start_pose.y;
    int px = static_cast<int>((glob_x - global_min_x) / CSIZE);
    int py = static_cast<int>((global_max_y - glob_y) / CSIZE);

    cv::Scalar color = (submap.submap_id == selected_id) ? RED : BLUE_LIGHT;
    int radius = (submap.submap_id == selected_id) ? 10 : 5;

    if (px >= 0 && px < img_w && py >= 0 && py < img_h) {
      cv::circle(integrated_img, cv::Point(px, py), radius, color, -1); // 塗りつぶし
      cv::putText(integrated_img, std::to_string(submap.submap_id), cv::Point(px + 10, py + 10), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
    }
  }

  // 5. ウィンドウに表示
  cv::imshow("Manual Loop Closure", integrated_img);
}
