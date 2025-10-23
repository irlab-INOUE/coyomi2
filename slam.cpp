/*
 * 全体地図: 一枚の地図として軌跡推定をしたもの
 * 部分地図: 一定の累積走行距離ごとに地図を分割したもの
 * 統合地図: 部分地図を貼り合わせたもの．見た目は全体地図だが，作り方が違う
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iomanip>  // std::setprecision, std::setw, std::fixed
#include <tuple>
#include <random>
#include <sstream>  // std::stringstream
#include <cstdlib>  // popen, pclose
#include <lua.hpp>  // Lua

namespace fs = std::filesystem;

// Color definitions for visualization
#define GREEN cv::Scalar(118,209,173)
#define BACK_BLUE cv::Scalar(47,35,30)
#define BLUE_LIGHT cv::Scalar(255,218,0)
#define GRID_DARK cv::Scalar(68,68,68)
#define WHITE cv::Scalar(198,204,203)
#define RED cv::Scalar(0,31,245)

// センサー配置の座標
const double SENSOR_OFFSET_X = 0.19; // urglog_bのX軸オフセット [m]

// LiDAR照射方向テーブル（グローバル変数）
std::vector<double> lidar_cos_table;
std::vector<double> lidar_sin_table;
std::vector<double> lidar_angle_table;

struct Point {
  double x;
  double y;

  Point() {};
  Point(double x, double y) {
    this->x = x;
    this->y = y;
  }
};

// ロボット姿勢
struct Pose {
  long long ts;
  double x, y, a;
  Pose() {};
  Pose(long long ts, double x, double y, double a) {
    this->ts = ts;
    this->x = x;
    this->y = y;
    this->a = a;
  }
};

// LiDARのレイキャストの逆引きのための曲座標情報;
struct PolarInfo {
  double angle;
  double distance;
  PolarInfo() : angle(0.0), distance(0.0) {}
  PolarInfo(double a, double d) : angle(a), distance(d) {}
};

// LiDARの1スキャンデータ
struct LaserData {
  long long timestamp;
  std::vector<Point> points;
  std::vector<double> angles; // 各点の角度情報
  std::vector<double> ranges; // 各点の距離情報
  long max_r;
  bool valid;
  char sensor_type; // 't' for urglog_t, 'b' for urglog_b
  double start_angle, end_angle, delta_th; // 照射角情報
};

// 部分地図の構造体
struct SubMap {
  int submap_id;
  double global_start_distance;
  double global_end_distance;
  Pose start_pose;  // 物理座標の原点（基準点）

  std::vector<LaserData> laser_data_sequence;
  std::vector<Pose> trajectory; // 部分地図内での相対座標による軌跡
  std::vector<std::vector<double>> local_gmap;

  // 点群の実際の範囲（部分地図相対座標）
  double min_x, max_x;
  double min_y, max_y;
  bool bounds_initialized;
  double LOCAL_CSIZE; // 元の全体地図のcsizeに相当する

  // 地図パラメータ（インスタンス変数）
  int LOCAL_WIDTH;
  int LOCAL_HEIGHT;
  int LOCAL_ORIGIN_X;
  int LOCAL_ORIGIN_Y;

  SubMap() : submap_id(-1), global_start_distance(0.0), global_end_distance(0.0),
    min_x(0.0), max_x(0.0), min_y(0.0), max_y(0.0), bounds_initialized(false), LOCAL_CSIZE(0.05),
    LOCAL_WIDTH(2000), LOCAL_HEIGHT(2000), LOCAL_ORIGIN_X(1000), LOCAL_ORIGIN_Y(1000) {
    local_gmap.resize(LOCAL_HEIGHT, std::vector<double>(LOCAL_WIDTH, 0.0));
  }

  SubMap(int id, double start_dist, const Pose& start_p, double csize) 
    : submap_id(id), global_start_distance(start_dist), global_end_distance(start_dist), start_pose(start_p),
    min_x(0.0), max_x(0.0), min_y(0.0), max_y(0.0), bounds_initialized(false), LOCAL_CSIZE(csize),
    LOCAL_WIDTH(2000), LOCAL_HEIGHT(2000), LOCAL_ORIGIN_X(1000), LOCAL_ORIGIN_Y(1000) {
    local_gmap.resize(LOCAL_HEIGHT, std::vector<double>(LOCAL_WIDTH, 0.0));
  }

  // 点の範囲を更新（部分地図相対座標で）
  void update_bounds_point(double rel_x, double rel_y) {
    if (!bounds_initialized) {
      min_x = max_x = rel_x;
      min_y = max_y = rel_y;
      bounds_initialized = true;
    } else {
      if (rel_x < min_x) min_x = rel_x;
      if (rel_x > max_x) max_x = rel_x;
      if (rel_y < min_y) min_y = rel_y;
      if (rel_y > max_y) max_y = rel_y;
    }
  }

  // 部分地図での姿勢推定と地図構築
  void build_submap();

  // 部分地図構築進捗の可視化
  void show_submap_progress(size_t current_frame);

  // 部分地図データの保存
  void save_submap_data(const std::string& base_dir);
};

// 極座標テーブル用のグローバル変数
std::vector<std::vector<PolarInfo>> polar_lookup_table;
bool polar_table_initialized = false;
int table_size = 0;
int table_origin = 0;
double table_csize = 0.0;

// 対数オッズ値の定数（CreateOccMap.cppより）
const double log04  = log(0.40/(1.0 - 0.40));   // 約-0.4055 (低い占有確率)
const double log045 = log(0.45/(1.0 - 0.45)); // 約-0.2007 (わずかに低い確率)  
const double log06  = log(0.60/(1.0 - 0.60));   // 約+0.4055 (高い占有確率)

// ガウシアンカーネルクラス
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


// 統合地図作成・表示関数の前方宣言
std::vector<std::vector<double>> create_and_show_integrated_map(
  const std::vector<SubMap>& completed_submaps);

// 関数の前方宣言
std::vector<std::vector<double>> update_map(
  std::vector<std::vector<double>> &gmap, 
  std::vector<Point> &pt1, 
  double current_x, double current_y, double current_a, 
  int width, int height, int ox, int oy, double CSIZE);

std::vector<std::vector<double>> remove_moving_objects(
  std::vector<std::vector<double>> &gmap,
  const LaserData &scan_data,
  double current_x, double current_y, double current_a,
  int width, int height, int ox, int oy, double CSIZE);

std::tuple<double, double, double, double> optimize_de(
  std::vector<std::vector<double>> &gmap, std::vector<Point> &pt,
  double current_x, double current_y, double current_a,
  int originX, int originY, 
  double CSIZE, double dth,
  int width, int height, 
  double Wxy, double Wa, 
  int population_size, int generations, double F, double CR,
  const GaussianKernel* kernel);


std::tuple<int, int> xy2index(double xd, double yd, double CSIZE, int ox, int oy) {
  int ix = static_cast<int>( xd / CSIZE) + ox;
  int iy = static_cast<int>(-yd / CSIZE) + oy;
  return std::make_tuple(ix, iy);
}

std::vector<std::vector<double>> update_map(std::vector<std::vector<double>> &gmap, 
                                            std::vector<Point> &pt1, 
                                            double current_x, double current_y, double current_a, 
                                            int width, int height, int ox, int oy, double CSIZE) {
  int ix, iy;
  double cs = cos(current_a);
  double sn = sin(current_a);
  for (size_t i = 0; i < pt1.size(); i++) {
    // ロボット中心からの距離を計算
    double distance = sqrt(pt1[i].x * pt1[i].x + pt1[i].y * pt1[i].y);

    // 500mm以下の近距離データは除去（ロボット自身や近傍ノイズ）
    if (distance <= 0.5) {
      continue;
    }

    double xd = pt1[i].x * cs - pt1[i].y * sn + current_x;
    double yd = pt1[i].x * sn + pt1[i].y * cs + current_y;
    auto [ix, iy] = xy2index(xd, yd, CSIZE, ox, oy);
    if (0 <= ix && ix < width && 0 <= iy && iy < height) {
      // 観測された点は高い占有確率を与える（強化）
      gmap[iy][ix] += log06 * 1.5; // 1.5倍の加算で静的障害物を強固に
    }
  }

  return gmap;
}

// 極座標ルックアップテーブルの初期化
void initialize_polar_table(int size, int origin, double csize) {
  if (polar_table_initialized && table_size == size && 
    table_origin == origin && table_csize == csize) {
    return; // 既に同じパラメータで初期化済み
  }

  table_size = size;
  table_origin = origin;
  table_csize = csize;

  polar_lookup_table.resize(size, std::vector<PolarInfo>(size));

  // 各ピクセルの角度と距離を事前計算
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      double dx = (x - origin) * csize;
      double dy = -(y - origin) * csize; // y軸反転

      double angle = atan2(dy, dx);
      double distance = sqrt(dx * dx + dy * dy);
      polar_lookup_table[y][x] = PolarInfo(angle, distance);
    }
  }

  polar_table_initialized = true;
  std::cout << "極座標ルックアップテーブル初期化完了 (" << size << "x" << size << ")" << std::endl;
}

// 移動体除去処理
std::vector<std::vector<double>> remove_moving_objects(std::vector<std::vector<double>> &gmap,
                                                       const LaserData &scan_data,
                                                       double current_x, double current_y, double current_a,
                                                       int width, int height, int ox, int oy, double CSIZE) {
  // std::cout << "  [移動体除去処理] スキャン点数: " << scan_data.points.size() << std::endl;

  // レーザー到達距離設定（ロボット近傍の移動体のみ対象）
  const double LASER_RANGE = 5.0; 

  // ローカル座標系の2次元配列サイズ計算（解像度はgmapと同一）
  int local_size = static_cast<int>(2 * LASER_RANGE / CSIZE); 
  int local_origin = local_size / 2; // 中央を原点とする

  // ローカル座標系の2次元配列を初期化（実数値対応）
  std::vector<std::vector<double>> local_map(local_size, std::vector<double>(local_size, 0.0));

  // scan_dataの点をローカル座標系にバイナリで格納
  for (const auto& point : scan_data.points) {
    // ローカル座標系でのインデックス計算
    int ix = static_cast<int>( point.x / CSIZE) + local_origin;
    int iy = static_cast<int>(-point.y / CSIZE) + local_origin; // y軸反転

    // 範囲チェック
    if (ix >= 0 && ix < local_size && iy >= 0 && iy < local_size) {
      local_map[iy][ix] = 1.0; // バイナリで格納
    }
  }

  // cv::Matに変換して可視化
  /*
  cv::Mat local_img(local_size, local_size, CV_64F);
  for (int y = 0; y < local_size; y++) {
    for (int x = 0; x < local_size; x++) {
      local_img.at<double>(y, x) = local_map[y][x];
    }
  }

  // 可視化用に0-255に正規化
  cv::Mat display_img;
  local_img.convertTo(display_img, CV_8U, 255.0);

  // 十字線を描画（原点表示）
  cv::line(display_img, cv::Point(local_origin, 0), cv::Point(local_origin, local_size-1), cv::Scalar(128), 1);
  cv::line(display_img, cv::Point(0, local_origin), cv::Point(local_size-1, local_origin), cv::Scalar(128), 1);

  cv::imshow("Local Scan (Bottom)", display_img);
  cv::waitKey(10);
  */

  // 極座標テーブルの初期化（初回のみ）
  initialize_polar_table(local_size, local_origin, CSIZE);

  // 自由空間判定用配列
  std::vector<std::vector<double>> free_space_map(local_size, std::vector<double>(local_size, 0.0));

  // 各ピクセルについて自由空間判定（ルックアップテーブル使用）
  for (int y = 0; y < local_size; y++) {
    for (int x = 0; x < local_size; x++) {
      const PolarInfo& pixel_polar = polar_lookup_table[y][x];

      // 原点はロボット位置なので確実に自由空間として登録
      if (x == local_origin && y == local_origin) {
        free_space_map[y][x] = -1.0;  // 自由空間として登録
        continue;
      }

      // 該当角度に最も近いLiDARデータを探す（角度情報を直接使用）
      double min_angle_diff = M_PI;
      double closest_scan_distance = LASER_RANGE;

      // LiDARの角度データを直接参照
      for (size_t i = 0; i < scan_data.angles.size(); i++) {
        double angle_diff = fabs(scan_data.angles[i] - pixel_polar.angle);
        // 角度の周期性を考慮（-π〜πの範囲）
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

        if (angle_diff < min_angle_diff) {
          min_angle_diff = angle_diff;
          closest_scan_distance = scan_data.ranges[i];
        }
      }

      // 角度の許容範囲内で、スキャン点より近い場合は自由空間
      const double ANGLE_TOLERANCE = scan_data.delta_th * M_PI / 180.0 * 2; // 角度刻みの2倍を許容範囲
      if (min_angle_diff < ANGLE_TOLERANCE && pixel_polar.distance < closest_scan_distance) {
        free_space_map[y][x] = -1.0;
      }
    }
  }

  // 自由空間マップをcv::Matに変換して可視化
  /*
  cv::Mat free_space_img(local_size, local_size, CV_8U);
  for (int y = 0; y < local_size; y++) {
    for (int x = 0; x < local_size; x++) {
      if (free_space_map[y][x] == -1.0) {
        free_space_img.at<uchar>(y, x) = 255; // -1の部分を白
      } else {
        free_space_img.at<uchar>(y, x) = 0;   // それ以外は黒
      }
    }
  }

  // 十字線を描画（原点表示）
  cv::line(free_space_img, cv::Point(local_origin, 0), cv::Point(local_origin, local_size-1), cv::Scalar(128), 1);
  cv::line(free_space_img, cv::Point(0, local_origin), cv::Point(local_size-1, local_origin), cv::Scalar(128), 1);

  cv::imshow("Free Space Map", free_space_img);
  cv::waitKey(10);
  */

  // 自由空間ピクセルをgmapに対応付けて移動体除去
  int removed_count = 0;

  double cs = cos(current_a);
  double sn = sin(current_a);
  for (int y = 0; y < local_size; y++) {
    for (int x = 0; x < local_size; x++) {
      if (free_space_map[y][x] == -1.0) { // 自由空間ピクセル
        // ローカル座標系からセンサー座標系への変換
        double local_x =  (x - local_origin) * CSIZE;
        double local_y = -(y - local_origin) * CSIZE; // y軸反転

        // センサー座標系から車両座標系への変換（オフセット考慮）
        double sensor_x = local_x - SENSOR_OFFSET_X; // センサーオフセット補正
        double sensor_y = local_y;

        // 車両座標系から世界座標系への変換
        double world_x = sensor_x * cs - sensor_y * sn + current_x;
        double world_y = sensor_x * sn + sensor_y * cs + current_y;

        // 世界座標系からgmapインデックスへの変換
        auto [gmap_ix, gmap_iy] = xy2index(world_x, world_y, CSIZE, ox, oy);

        // gmap範囲内で、障害物の占有確率が高い場合は尤度を減算
        if (gmap_ix >= 0 && gmap_ix < width && gmap_iy >= 0 && gmap_iy < height) {
          // 占有確率が30%以上の場合を障害物とみなす（閾値を下げて除去を強化）
          if (gmap[gmap_iy][gmap_ix] > -0.8) {  // log(0.3/0.7) ≈ -0.847
            // より強い減算で確実に除去
            gmap[gmap_iy][gmap_ix] -= log06 * 2.0; // 2倍の減算で強力除去
            removed_count++;
          }
        }
      }
    }
  }

  if (removed_count > 0) {
    // std::cout << "  [移動体除去] 除去したグリッド数: " << removed_count << std::endl;
  }

  return gmap;
}

// 占有地図から尤度場地図を生成する関数
void create_likelihood_field_map(const std::vector<std::vector<double>>& gmap, 
                                 int width, int height, double CSIZE,
                                 const std::string& output_dir) {
  std::cout << "尤度場地図生成開始..." << std::endl;

  // gmapの内容確認用表示
  cv::Mat gmap_check = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(50, 50, 50));
  const double HIGH_THRESHOLD = 0.405;   // 60%確率に相当

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      double log_odds = gmap[j][i];

      if (log_odds > HIGH_THRESHOLD) {  // 高い占有確率：白色
        gmap_check.at<cv::Vec3b>(j, i)[0] = 255;
        gmap_check.at<cv::Vec3b>(j, i)[1] = 255;
        gmap_check.at<cv::Vec3b>(j, i)[2] = 255;
      }
      // その他は背景色のまま
    }
  }

  // パラメータ設定（MakeLFM.cppより）
  double sgm2 = 0.4 * 0.4;  // σ² = 0.16
  double mu = 1.0/sqrt(2*M_PI*sgm2);
  int distance = static_cast<int>(2.0 / CSIZE);  // 2m範囲で最近傍探索

  // 尤度場地図用の配列
  cv::Mat LFM(height, width, CV_64FC1);
  cv::Mat imgLFM(height, width, CV_8UC1);

  // 占有確率閾値（対数オッズから判定）

  for (int iy = 0; iy < height; iy++) {
    if (iy % 100 == 0) {
      std::cout << "\r尤度場生成進捗: " << iy << "/" << height << std::flush;
    }

    for (int ix = 0; ix < width; ix++) {
      // 全範囲で尤度場を計算（未観測領域も含む）

      // 最近傍の障害物を探索
      double min_dist2 = 1e9;
      int startX = ix - distance; if (startX < 0) startX = 0;
      int endX = ix + distance; if (endX >= width) endX = width - 1;
      int startY = iy - distance; if (startY < 0) startY = 0;
      int endY = iy + distance; if (endY >= height) endY = height - 1;

      for (int cy = startY; cy < endY; cy++) {
        for (int cx = startX; cx < endX; cx++) {
          // 高い占有確率（障害物）の場合
          if (gmap[cy][cx] > 0.405) {
            double dist2 = ((ix - cx) * (ix - cx) + (iy - cy) * (iy - cy)) * CSIZE * CSIZE;
            if (min_dist2 > dist2) {
              min_dist2 = dist2;
            }
          }
        }
      }

      // ガウシアン尤度値を計算
      LFM.at<double>(iy, ix) = sgm2 * exp(-0.5 * min_dist2/sgm2);
    }
  }

  // OpenCV形式で実数データを出力（ナビゲーション用）
  cv::FileStorage fs(output_dir + "/lfm.yml", cv::FileStorage::WRITE);
  fs << "height" << height;
  fs << "width" << width;
  fs << "CSIZE" << CSIZE;
  fs << "sigma" << sqrt(sgm2);  // パラメータも保存
  fs << "likelihood_field" << LFM;
  fs.release();

  std::cout << "尤度場地図保存: " << output_dir << "/lfm.yml" << std::endl;

  // 可視化用PNG出力
  double min_val = 1e9;
  double max_val = -1e9;
  for (int iy = 0; iy < LFM.rows; iy++) {
    for (int ix = 0; ix < LFM.cols; ix++) {
      if (LFM.at<double>(iy, ix) < 0) continue;
      if (min_val > LFM.at<double>(iy, ix)) min_val = LFM.at<double>(iy, ix);
      if (max_val < LFM.at<double>(iy, ix)) max_val = LFM.at<double>(iy, ix);
    }
  }

  for (int iy = 0; iy < imgLFM.rows; iy++) {
    for (int ix = 0; ix < imgLFM.cols; ix++) {
      if (LFM.at<double>(iy, ix) < 0) {
        imgLFM.at<uchar>(iy, ix) = 128;  // 未観測領域
      } else {
        unsigned char val;
        if (LFM.at<double>(iy, ix) < 1e-8)
          val = 0;
        else
          val = static_cast<unsigned char>((LFM.at<double>(iy, ix) - min_val) / (max_val - min_val) * 255);
        imgLFM.at<uchar>(iy, ix) = val;
      }
    }
  }

  cv::imwrite(output_dir + "/lfm.png", imgLFM);
  std::cout << "\r尤度場地図生成完了: " << output_dir << "/lfm.yml (実数データ), lfm.png (可視化)" << std::endl;
}

cv::Mat gmap_show(std::vector<std::vector<double>> &gmap, double width, double height,
                  const std::vector<Pose>* pose_trajectory = nullptr,
                  double CSIZE = 0.05, int originX = 0, int originY = 0, double minX = 0, double minY = 0) {
  cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC3, BACK_BLUE);

  // 対数オッズ直接比較で高速化（exp計算を回避）
  // log(0.6/0.4) ≈ 0.405, log(0.4/0.6) ≈ -0.405
  const double HIGH_THRESHOLD = 0.405;   // 60%確率に相当
  const double LOW_THRESHOLD = -0.405;   // 40%確率に相当

  for (int j = 0; j < gmap.size(); j++) {
    for (int i = 0; i < gmap[0].size(); i++) {
      double log_odds = gmap[j][i];

      if (log_odds > HIGH_THRESHOLD) {  // 高い占有確率：白色
        img.at<cv::Vec3b>(j, i)[0] = WHITE[0];
        img.at<cv::Vec3b>(j, i)[1] = WHITE[1];
        img.at<cv::Vec3b>(j, i)[2] = WHITE[2];
      }
      // 中間確率は背景色のまま
    }
  }

  // Draw estimated trajectory and uncertainty ellipses
  if (pose_trajectory != nullptr && pose_trajectory->size() > 1) {
    // Draw trajectory path
    for (size_t i = 1; i < pose_trajectory->size(); i++) {
      int prev_x = static_cast<int>((*pose_trajectory)[i-1].x / CSIZE) + originX;
      int prev_y = static_cast<int>(-(*pose_trajectory)[i-1].y / CSIZE) + originY;
      int curr_x = static_cast<int>((*pose_trajectory)[i].x / CSIZE) + originX;
      int curr_y = static_cast<int>(-(*pose_trajectory)[i].y / CSIZE) + originY;

      if (prev_x >= 0 && prev_x < width && prev_y >= 0 && prev_y < height &&
        curr_x >= 0 && curr_x < width && curr_y >= 0 && curr_y < height) {
        cv::line(img, cv::Point(prev_x, prev_y), cv::Point(curr_x, curr_y), GREEN, 2);
      }
    }

    // Draw pose trajectory points
    for (size_t i = 0; i < pose_trajectory->size(); i += 5) {
      const auto& pose = (*pose_trajectory)[i];
      int pose_x = static_cast<int>(pose.x / CSIZE) + originX;
      int pose_y = static_cast<int>(-pose.y / CSIZE) + originY;

      if (pose_x >= 0 && pose_x < width && pose_y >= 0 && pose_y < height) {
        cv::circle(img, cv::Point(pose_x, pose_y), 3, BLUE_LIGHT, -1);
      }
    }
  }

  cv::imshow("slam", img);
  int key = cv::waitKey(10);
  return img;
}

double normalize_th(double ra) {
  while(1) {
    if (ra > M_PI) {
      ra -= 2*M_PI;
    } else if (ra < -M_PI) {
      ra += 2*M_PI;
    } else {
      break;
    }
  }
  return ra;
}


double match_simple_count(std::vector<std::vector<double>> &gmap,
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
    if (0 <= ix && ix < width && 0 <= iy && iy < height) {
      // 対数オッズ直接比較（0.0が50%確率に相当）
      if (gmap[iy][ix] > 0.0)
        eval += 1.0; // バイナリカウント（高速化）
    }
  }
  return eval;
}

double match_count(std::vector<std::vector<double>> &gmap,
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
          // 対数オッズ直接比較（0.0が50%確率）
          if (gmap[iy + dy][ix + dx] > 0.0)
            eval += (1.0/(1+abs(dx) + abs(dy)));
        }
      }
    }
  }

  // 0～1に正規化
  // 理論的最大値: 1点あたり最大 13/3, 最大LiDAR点数 1081
  const double MAX_SCORE_PER_POINT = 13.0/3.0;  // 4.33...
  const double MAX_LIDAR_POINTS = 1081.0;
  const double THEORETICAL_MAX = MAX_LIDAR_POINTS * MAX_SCORE_PER_POINT;

  return eval / THEORETICAL_MAX;
}

// ガウシアンカーネルを使用した評価関数
double gaussian_match_count(const std::vector<std::vector<double>>& gmap,
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

    // 点を変換してグリッド座標に変換
    double xd = p.x * cs - p.y * sn + cx;
    double yd = p.x * sn + p.y * cs + cy;
    auto [gx, gy] = xy2index(xd, yd, CSIZE, originX, originY);

    // カーネル範囲内でガウシアン重み付きスコア計算
    double point_score = 0.0;
    for (int dy = -kernel.getRadius(); dy <= kernel.getRadius(); dy++) {
      for (int dx = -kernel.getRadius(); dx <= kernel.getRadius(); dx++) {
        int map_x = gx + dx;
        int map_y = gy + dy;

        // グリッド範囲チェック
        if (map_x >= 0 && map_x < width && map_y >= 0 && map_y < height) {
          // 対数オッズ直接比較（高速化）
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

std::tuple<double, double, double, double> optimize_greedy(
  std::vector<std::vector<double>> &gmap, std::vector<Point> &pt,
  double current_x, double current_y, double current_a, 
  int originX, int originY, 
  double CSIZE, double dth,
  int width, int height, 
  double Wxy, double Wa) {
  double best_x, best_y, best_a;
  double best_eval = 0;
  for (double dx = -Wxy; dx < Wxy; dx += CSIZE) {
    double cx = current_x + dx;
    for (double dy = -Wxy; dy < Wxy; dy += CSIZE) {
      double cy = current_y + dy;
      for (double da = -Wa; da < Wa; da += dth) {
        double ca = current_a + da;
        int eval = match_count(gmap, pt, cx, cy, ca, CSIZE, originX, originY, width, height);
        if (best_eval < eval) {
          best_eval = eval;
          best_x = cx;
          best_y = cy;
          best_a = ca;
        }
      }
    }
  }
  return std::make_tuple(best_x, best_y, best_a, best_eval);
}

// 乱数生成エンジンの準備
std::random_device rd; // ハードウェア乱数生成器
std::mt19937 gen(rd()); // メルセンヌ・ツイスタ法の乱数生成器
// -1から1の間の一様分布を定義
std::uniform_real_distribution<> dis(-1.0, 1.0);


std::tuple<double, double, double, double> optimize_de(
  std::vector<std::vector<double>> &gmap, std::vector<Point> &pt,
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
    // Initialize population
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
      // 各世代の最初に，ベストな個体を見つけるは，悪くなった

      for (int i = 0; i < population_size; i++) {
        // Mutation: select three distinct individuals (r1, r2, r3)
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

        // Mutation: v = x_r1 + F * (x_r2 - x_r3)
        double vx = x1 + F * (x2 - x3);
        double vy = y1 + F * (y2 - y3);
        //double va = a1 + F * (a2 - a3);

        double ax1 = cos(a1), ay1 = sin(a1);
        double ax2 = cos(a2), ay2 = sin(a2);
        double ax3 = cos(a3), ay3 = sin(a3);
        double vax = ax1 + F * (ax2 - ax3);
        double vay = ay1 + F * (ay2 - ay3);
        double va = atan2(vay, vax);

        // Crossover: generate trial vector u by mixing the mutant vector with the target vector
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

        // Evaluate the trial vector
        double eval;
        if (kernel != nullptr) {
          eval = gaussian_match_count(gmap, pt, trial_x, trial_y, trial_a, CSIZE, originX, originY, width, height, *kernel);
        } else {
          eval = match_count(gmap, pt, trial_x, trial_y, trial_a, CSIZE, originX, originY, width, height);
        }

        // Selection: if trial is better, replace the target individual
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

// LASERSCANRTの総行数をカウントする関数
int count_laserscanrt_lines(const std::string& filename) {
  std::string command = "grep -c 'LASERSCANRT' " + filename;
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) return -1;

  char buffer[128];
  std::string result = "";
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  pclose(pipe);

  try {
    return std::stoi(result);
  } catch (const std::exception&) {
    return -1;
  }
}

// 画面クリアと上部固定表示用の関数
void update_status_display(int loop, const std::string& sensor_type, 
                           long long timestamp, double eval, 
                           double x, double y, double angle_deg, int total_count, double distance = 0.0) {
  static std::vector<std::string> recent_lines;
  const int MAX_LINES = 20;

  // 色分け設定
  std::string color_code;
  if (sensor_type.find("Top") != std::string::npos) {
    color_code = "\033[31m"; // 赤色
  } else if (sensor_type.find("Bottom") != std::string::npos) {
    color_code = "\033[34m"; // 青色
  } else {
    color_code = "\033[37m"; // 白色（その他）
  }

  // 新しい行を作成（色付き・桁揃え）
  std::stringstream ss;
  ss << color_code 
    << "[" << std::setw(12) << std::left << sensor_type << "] "
    << "ts:" << std::setw(13) << timestamp 
    << " eval:" << std::fixed << std::setprecision(3) << std::setw(7) << eval
    << " pose:(" << std::setprecision(3) << std::setw(8) << x 
    << "," << std::setw(8) << y;

  // 角度を手動でフォーマット
  ss << ",";
  if (angle_deg >= 0) ss << " ";  // 正の数の場合は先頭にスペース
  ss << std::fixed << std::setprecision(1) << std::setw(4) << angle_deg << "°)";

  // 累積距離を追加
  if (distance > 0.0) {
    ss << " dist:" << std::setprecision(2) << std::setw(6) << distance << "m";
  }

  ss << "\033[0m"; // 色リセット

  // 最新行を追加
  recent_lines.push_back(ss.str());
  if (recent_lines.size() > MAX_LINES) {
    recent_lines.erase(recent_lines.begin());
  }

  // 画面クリアして再表示
  std::cout << "\033[2J\033[H"; // 画面クリア + カーソル移動

  // プログレス表示（色付き）
  int progress = (total_count > 0) ? (loop * 100) / total_count : 0;
  progress = std::min(100, progress);  // 100%で上限
  std::cout << "\033[32mSLAM Progress: \033[33m" << std::setw(4) << loop 
    << "\033[32m/" << total_count << " (\033[33m" << std::setw(3) << progress << "%\033[32m)\033[0m" 
    << std::endl;
  std::cout << "\033[36m" << std::string(50, '=') << "\033[0m" << std::endl;

  for (const auto& line : recent_lines) {
    std::cout << line << std::endl;
  }
  std::cout << std::flush;
}

// LiDAR角度テーブルの初期化関数
void initialize_lidar_tables() {
  // LiDAR設定（coyomi.yamlから読み込みも可能だが，ここでは固定値）
  const double START_ANGLE = -135.0;
  const double END_ANGLE = 135.0;
  const double DELTA_TH = 0.25;
  const int MAX_ECHO_SIZE = 3;

  int num_points = static_cast<int>((END_ANGLE - START_ANGLE) / DELTA_TH + 1);
  lidar_cos_table.resize(num_points);
  lidar_sin_table.resize(num_points);
  lidar_angle_table.resize(num_points);

  double th = START_ANGLE * M_PI/180.0;
  for (int i = 0; i < num_points; i++) {
    lidar_cos_table[i] = cos(th);
    lidar_sin_table[i] = sin(th);
    lidar_angle_table[i] = th; // 角度値を事前計算
    th += DELTA_TH * M_PI/180.0;
  }

  std::cout << "LiDAR角度テーブル初期化完了: " << num_points << "点" << std::endl;
}

// SubMapクラスのbuild_submap()メソッドの実装
void SubMap::build_submap() {
  std::cout << "SubMap " << submap_id << " の構築完了 (メインループの結果を使用)" << std::endl;

  // local_gmapとtrajectoryはメインループで既に構築済みなので、ここでは再構築しない。
  // 代わりに、最終的なboundsを計算する。

  // LiDAR点群の範囲を更新（部分地図相対座標で）
  for (size_t i = 0; i < laser_data_sequence.size(); i++) {
    const LaserData& laser_data = laser_data_sequence[i];
    const Pose& pose = trajectory[i]; // メインループで推定された姿勢を使用

    if (!laser_data.valid || laser_data.points.empty()) continue;

    double cos_a = cos(pose.a);
    double sin_a = sin(pose.a);
    for (const Point& pt : laser_data.points) {
      // LiDAR点をロボット座標系から部分地図座標系に変換
      double global_x = pt.x * cos_a - pt.y * sin_a + pose.x;
      double global_y = pt.x * sin_a + pt.y * cos_a + pose.y;
      update_bounds_point(global_x, global_y);
    }

    // ロボット位置自体も範囲に含める
    // 例えば，視野が前方しかない場合に後退すると，点群は範囲でも自身が領域をはみ出すことがある
    update_bounds_point(pose.x, pose.y);
  }

  std::cout << "SubMap " << submap_id << " bounds: X[" << min_x << "," << max_x 
    << "] Y[" << min_y << "," << max_y << "]" << std::endl;
}

// SubMapクラスのshow_submap_progress()メソッドの実装
void SubMap::show_submap_progress(size_t current_frame) {
  // local_gmapを可視化用画像に変換
  cv::Mat submap_img = cv::Mat(cv::Size(LOCAL_WIDTH, LOCAL_HEIGHT), CV_8UC3, cv::Scalar(50, 50, 50));

  // local_gmapの内容を描画（対数オッズを確率に変換）
  for (int j = 0; j < LOCAL_HEIGHT; j++) {
    for (int i = 0; i < LOCAL_WIDTH; i++) {
      double log_odds = local_gmap[j][i];

      if (log_odds > 0.2) {  // 高い占有確率：白色（障害物）
        submap_img.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 255, 255);
      } else if (log_odds < -0.2) {  // 低い占有確率：黒色（自由空間）
        submap_img.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
      }
      // 未観測領域は背景色のまま
    }
  }

  // 軌跡を描画（現在のフレームまで）
  for (size_t i = 0; i <= current_frame && i < trajectory.size(); i++) {
    int px = static_cast<int>(trajectory[i].x / LOCAL_CSIZE) + LOCAL_ORIGIN_X;
    int py = static_cast<int>(-trajectory[i].y / LOCAL_CSIZE) + LOCAL_ORIGIN_Y;

    if (px >= 0 && px < LOCAL_WIDTH && py >= 0 && py < LOCAL_HEIGHT) {
      if (i == current_frame) {
        // 現在位置：赤色（大きめ）
        cv::circle(submap_img, cv::Point(px, py), 3, cv::Scalar(0, 0, 255), -1);

        // 方向表示
        double ax = cos(trajectory[i].a) * 10;
        double ay = sin(trajectory[i].a) * 10;
        cv::line(submap_img, cv::Point(px, py), 
                 cv::Point(px + ax, py - ay), cv::Scalar(0, 0, 255), 2);
      } else {
        // 過去の軌跡：緑色（小さめ）
        cv::circle(submap_img, cv::Point(px, py), 1, cv::Scalar(0, 255, 0), -1);
      }
    }
  }

  // 現在のLiDARデータを描画
  if (current_frame < laser_data_sequence.size()) {
    const LaserData& current_laser = laser_data_sequence[current_frame];
    const Pose& current_pose = trajectory[current_frame];

    for (const Point& pt : current_laser.points) {
      // LiDAR点をグローバル座標に変換
      double cs = cos(current_pose.a);
      double sn = sin(current_pose.a);
      double x_global = pt.x * cs - pt.y * sn + current_pose.x;
      double y_global = pt.x * sn + pt.y * cs + current_pose.y;

      int px = static_cast<int>(x_global / LOCAL_CSIZE) + LOCAL_ORIGIN_X;
      int py = static_cast<int>(-y_global / LOCAL_CSIZE) + LOCAL_ORIGIN_Y;

      if (px >= 0 && px < LOCAL_WIDTH && py >= 0 && py < LOCAL_HEIGHT) {
        // LiDAR点：青色
        submap_img.at<cv::Vec3b>(py, px) = cv::Vec3b(255, 100, 0);
      }
    }
  }

  // 画像のリサイズ（見やすくするため）
  cv::Mat display_img;
  cv::resize(submap_img, display_img, cv::Size(600, 600));

  // ウィンドウ名を動的に設定
  std::string window_name = "SubMap " + std::to_string(submap_id) + " 構築進捗";
  cv::imshow(window_name, display_img);

  // 短時間待機（構築過程を観察するため）
  int key = cv::waitKey(100);
  if (key == ' ') {  // スペースキーで一時停止
    cv::waitKey(0);
  }
}

// 統合地図作成・表示関数の実装
std::vector<std::vector<double>> create_and_show_integrated_map(const std::vector<SubMap>& completed_submaps) {
  if (completed_submaps.empty()) return {};

  // 統合地図の範囲を計算
  double global_min_x = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_y = std::numeric_limits<double>::lowest();

  for (const auto& submap : completed_submaps) {
    // 各部分地図の範囲をグローバル座標系に変換（回転考慮）
    double cos_a = cos(submap.start_pose.a);
    double sin_a = sin(submap.start_pose.a);

    // 部分地図の4つの角を回転変換
    std::vector<std::pair<double, double>> corners = {
      {submap.min_x, submap.min_y},
      {submap.max_x, submap.min_y},
      {submap.min_x, submap.max_y},
      {submap.max_x, submap.max_y}
    };

    double submap_min_x = std::numeric_limits<double>::max();
    double submap_max_x = std::numeric_limits<double>::lowest();
    double submap_min_y = std::numeric_limits<double>::max();
    double submap_max_y = std::numeric_limits<double>::lowest();

    for (const auto& corner : corners) {
      double rotated_x = corner.first * cos_a - corner.second * sin_a;
      double rotated_y = corner.first * sin_a + corner.second * cos_a;
      double global_x = submap.start_pose.x + rotated_x;
      double global_y = submap.start_pose.y + rotated_y;

      submap_min_x = std::min(submap_min_x, global_x);
      submap_max_x = std::max(submap_max_x, global_x);
      submap_min_y = std::min(submap_min_y, global_y);
      submap_max_y = std::max(submap_max_y, global_y);
    }

    global_min_x = std::min(global_min_x, submap_min_x);
    global_max_x = std::max(global_max_x, submap_max_x);
    global_min_y = std::min(global_min_y, submap_min_y);
    global_max_y = std::max(global_max_y, submap_max_y);
  }

  // マージンを追加
  double margin = 5.0;
  global_min_x -= margin;
  global_max_x += margin;
  global_min_y -= margin;
  global_max_y += margin;

  // 統合地図のサイズを計算
  const double INTEGRATED_CSIZE = 0.05;
  int integrated_width = static_cast<int>((global_max_x - global_min_x) / INTEGRATED_CSIZE);
  int integrated_height = static_cast<int>((global_max_y - global_min_y) / INTEGRATED_CSIZE);

  // デバッグ出力：統合地図の範囲とサイズ
  std::cout << "=== 統合地図範囲デバッグ ===" << std::endl;
  std::cout << "統合地図範囲: X[" << global_min_x << "," << global_max_x 
    << "] Y[" << global_min_y << "," << global_max_y << "]" << std::endl;
  std::cout << "統合地図サイズ: " << integrated_width << "x" << integrated_height << " (解像度:" << INTEGRATED_CSIZE << "m)" << std::endl;
  std::cout << "実際の範囲: X=" << (global_max_x - global_min_x) << "m, Y=" << (global_max_y - global_min_y) << "m" << std::endl;

  // 各部分地図の範囲も出力
  for (size_t i = 0; i < completed_submaps.size(); i++) {
    const auto& submap = completed_submaps[i];
    std::cout << "SubMap" << i << " start_pose:(" << submap.start_pose.x << "," << submap.start_pose.y << "," << submap.start_pose.a << ")" << std::endl;
    std::cout << "SubMap" << i << " bounds:X[" << submap.min_x << "," << submap.max_x 
      << "] Y[" << submap.min_y << "," << submap.max_y << "]" << std::endl;
  }
  std::cout << "=========================" << std::endl;

  // 統合地図の初期化
  std::vector<std::vector<double>> integrated_map(integrated_height, 
                                                  std::vector<double>(integrated_width, 0.0));

  std::cout << "統合地図作成中... (サイズ: " << integrated_width << "x" << integrated_height << ")" << std::endl;

  // 各部分地図を統合地図に投影
  for (const auto& submap : completed_submaps) {
    for (int local_y = 0; local_y < submap.LOCAL_HEIGHT; local_y++) {
      for (int local_x = 0; local_x < submap.LOCAL_WIDTH; local_x++) {
        double log_odds = submap.local_gmap[local_y][local_x];

        if (std::abs(log_odds) < 1e-6) continue; // 未観測グリッドはスキップ

        // 部分地図座標をグローバル座標に変換（回転考慮）
        double local_world_x = (local_x - submap.LOCAL_ORIGIN_X) * submap.LOCAL_CSIZE;
        double local_world_y = -(local_y - submap.LOCAL_ORIGIN_Y) * submap.LOCAL_CSIZE;

        // start_poseの回転を考慮した座標変換
        double cos_a = cos(submap.start_pose.a);
        double sin_a = sin(submap.start_pose.a);
        double rotated_x = local_world_x * cos_a - local_world_y * sin_a;
        double rotated_y = local_world_x * sin_a + local_world_y * cos_a;

        double global_x = submap.start_pose.x + rotated_x;
        double global_y = submap.start_pose.y + rotated_y;

        // 統合地図のグリッドインデックスに変換
        int integrated_x = static_cast<int>((global_x - global_min_x) / INTEGRATED_CSIZE);
        int integrated_y = static_cast<int>((global_max_y - global_y) / INTEGRATED_CSIZE);

        if (integrated_x >= 0 && integrated_x < integrated_width &&
          integrated_y >= 0 && integrated_y < integrated_height) {
          // 対数オッズを累積（重複領域では統合）
          integrated_map[integrated_y][integrated_x] += log_odds;
        }
      }
    }
  }

  // 可視化用画像を作成
  cv::Mat integrated_img = cv::Mat(cv::Size(integrated_width, integrated_height), CV_8UC3, 
                                   cv::Scalar(50, 50, 50));

  for (int y = 0; y < integrated_height; y++) {
    for (int x = 0; x < integrated_width; x++) {
      double log_odds = integrated_map[y][x];

      if (log_odds > 0.2) {  // 障害物
        integrated_img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
      } else if (log_odds < -0.2) {  // 自由空間
        integrated_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      }
    }
  }

  // 各部分地図の境界と軌跡を描画
  for (size_t i = 0; i < completed_submaps.size(); i++) {
    const auto& submap = completed_submaps[i];

    // 各部分地図の色を決定（異なる色で区別）
    cv::Scalar color;
    switch (i % 6) {
      case 0: color = cv::Scalar(0, 0, 255); break;    // 赤
      case 1: color = cv::Scalar(0, 255, 0); break;    // 緑
      case 2: color = cv::Scalar(255, 0, 0); break;    // 青
      case 3: color = cv::Scalar(0, 255, 255); break;  // 黄
      case 4: color = cv::Scalar(255, 0, 255); break;  // マゼンタ
      case 5: color = cv::Scalar(255, 255, 0); break;  // シアン
    }

    // 軌跡を描画
    for (const auto& pose : submap.trajectory) {
      // start_poseの回転を考慮した座標変換
      double cos_a = cos(submap.start_pose.a);
      double sin_a = sin(submap.start_pose.a);
      double rotated_x = pose.x * cos_a - pose.y * sin_a;
      double rotated_y = pose.x * sin_a + pose.y * cos_a;

      double global_x = submap.start_pose.x + rotated_x;
      double global_y = submap.start_pose.y + rotated_y;

      int px = static_cast<int>((global_x - global_min_x) / INTEGRATED_CSIZE);
      int py = static_cast<int>((global_max_y - global_y) / INTEGRATED_CSIZE);

      if (px >= 0 && px < integrated_width && py >= 0 && py < integrated_height) {
        cv::circle(integrated_img, cv::Point(px, py), 1, color, -1);
      }
    }

    // スタート地点を大きく表示
    double start_x = submap.start_pose.x;
    double start_y = submap.start_pose.y;
    int start_px = static_cast<int>((start_x - global_min_x) / INTEGRATED_CSIZE);
    int start_py = static_cast<int>((global_max_y - start_y) / INTEGRATED_CSIZE);

    if (start_px >= 0 && start_px < integrated_width && start_py >= 0 && start_py < integrated_height) {
      cv::circle(integrated_img, cv::Point(start_px, start_py), 5, color, 2);
      // 部分地図IDを表示
      cv::putText(integrated_img, std::to_string(submap.submap_id), 
                  cv::Point(start_px + 10, start_py), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
  }

  // 画像をリサイズして表示
  cv::Mat display_img;
  double scale = std::min(800.0 / integrated_width, 600.0 / integrated_height);
  scale = 1;
  if (scale < 1.0) {
    cv::resize(integrated_img, display_img, cv::Size(integrated_width * scale, integrated_height * scale));
  } else {
    display_img = integrated_img;
  }

  std::string window_name = "統合地図 (部分地図数: " + std::to_string(completed_submaps.size()) + ")";
  cv::imshow(window_name, display_img);
  cv::waitKey(1000);  // 1秒間表示

  std::cout << "統合地図表示完了 (部分地図数: " << completed_submaps.size() << ")" << std::endl;

  return integrated_map;
}

class LaserLogReader {
public:
  LaserLogReader(const std::string& filepath, char sensor_type, int lidar_skip)
    : sensor_type(sensor_type), lidar_direction_skip(lidar_skip), line_count(0) {
    file.open(filepath);
  }

  ~LaserLogReader() {
    if (file.is_open()) {
      file.close();
    }
  }

  bool is_open() const {
    return file.is_open();
  }

  LaserData readNextScan() {
    LaserData data;
    data.valid = false;
    data.sensor_type = this->sensor_type;

    line_count++;

    std::string type;
    while (file >> type && !file.eof()) {
      if (type == "LASERSCANRT") {
        long long timestamp_end;
        int count;
        double START_ANGLE, END_ANGLE, deltaTH;
        int max_echo_size;
        long intensity;

        file >> data.timestamp >> count >> START_ANGLE >> END_ANGLE >> deltaTH >> max_echo_size;

        if (file.fail()) {
          std::cout << "[デバッグ] センサ" << this->sensor_type << "のヘッダー読み込み失敗" << std::endl;
          data.valid = false;
          break;
        }

        data.start_angle = START_ANGLE;
        data.end_angle = END_ANGLE;
        data.delta_th = deltaTH;

        data.max_r = 0;
        long r;

        int loop_count = count / max_echo_size;
        if (this->sensor_type == 'b') {
          std::cout << "[デバッグ] センサb (行" << this->line_count << "): count=" << count << ", max_echo_size=" << max_echo_size << ", loop_count=" << loop_count << std::endl;
        }

        for (int i = 0; i < loop_count; i += this->lidar_direction_skip) {
          file >> r;

          if (file.fail()) {
            std::cout << "[デバッグ] センサ" << this->sensor_type << " (行" << this->line_count << ") 距離データ読み込み失敗 at i=" << i << " (部分的に有効として継続)" << std::endl;
            file.clear();
            break;
          }

          if (data.max_r < r) {
            data.max_r = r;
          }
          if (r > 200) {
            double x = (double)r * lidar_cos_table[i] / 1000.0;
            double y = (double)r * lidar_sin_table[i] / 1000.0;
            if (this->sensor_type == 't') {
              y = -y;
            } else if (this->sensor_type == 'b') {
              x += SENSOR_OFFSET_X;
            }
            data.points.emplace_back(x, y);
            data.angles.push_back(lidar_angle_table[i]);
            data.ranges.push_back(r / 1000.0);
          }
          file >> r >> r;

          if (file.fail()) {
            std::cout << "[デバッグ] センサ" << this->sensor_type << " マルチエコー読み込み失敗 at i=" << i << " (部分的に有効として継続)" << std::endl;
            file.clear();
            break;
          }
        }
        file >> timestamp_end;

        if (file.fail()) {
          std::cout << "[デバッグ] センサ" << this->sensor_type << " 最終タイムスタンプ読み込み失敗" << std::endl;
          data.valid = false;
          return data;
        }

        data.valid = true;
        break;
      }
    }

    if (!data.valid) {
      std::cout << "[デバッグ] センサ" << this->sensor_type << "のファイル読み込み失敗" << std::endl;
      std::cout << "  ファイル状態: eof=" << file.eof() << ", fail=" << file.fail() << ", bad=" << file.bad() << std::endl;
      std::cout << "  ファイル位置: " << file.tellg() << std::endl;

      file.clear();
      std::string debug_line;
      if (std::getline(file, debug_line)) {
        std::cout << "  次の行内容(先頭50文字): " << debug_line.substr(0, 50) << std::endl;
      } else {
        std::cout << "  次の行の読み取りも失敗" << std::endl;
      }
    }

    return data;
  }

private:
  std::ifstream file;
  char sensor_type;
  int lidar_direction_skip;
  int line_count;
};

class MergedLaserStream {
public:
  MergedLaserStream(const std::string& path_t, const std::string& path_b, int lidar_skip)
    : reader_t(path_t, 't', lidar_skip), reader_b(path_b, 'b', lidar_skip) {
    if (!reader_t.is_open()) {
      std::cerr << "urglog_t ファイルを開けませんでした: " << path_t << std::endl;
      next_scan_t.valid = false;
    } else {
      next_scan_t = reader_t.readNextScan();
    }

    if (!reader_b.is_open()) {
      std::cerr << "urglog_b ファイルを開けませんでした: " << path_b << std::endl;
      next_scan_b.valid = false;
    } else {
      next_scan_b = reader_b.readNextScan();
    }
  }

  bool is_finished() const {
    return !next_scan_t.valid && !next_scan_b.valid;
  }

  LaserData getNextScan() {
    if (is_finished()) {
      LaserData invalid_data; invalid_data.valid = false; return invalid_data;
    }

    LaserData current_data;
    bool t_chosen = false;

    if (!next_scan_t.valid) {
      t_chosen = false;
    } else if (!next_scan_b.valid) {
      t_chosen = true;
    } else if (next_scan_t.timestamp <= next_scan_b.timestamp) {
      t_chosen = true;
    } else {
      t_chosen = false;
    }

    if (t_chosen) {
      current_data = next_scan_t;
      next_scan_t = reader_t.readNextScan();
    } else {
      current_data = next_scan_b;
      next_scan_b = reader_b.readNextScan();
    }
    return current_data;
  }

private:
  LaserLogReader reader_t;
  LaserLogReader reader_b;
  LaserData next_scan_t;
  LaserData next_scan_b;
};

int main (int argc, char *argv[]) {
  int LIDAR_DIRECTION_SKIP = 1;   // LIDARデータの角度方向の読み飛ばし
  double CSIZE = 0.05;     // [m] 格子の解像度 0.025よりうまくいく

  double Wxy = 0.8;      // 探索範囲[m] (拡大: 0.6 → 0.8)
  double Wa  = M_PI/8;    // 角度[rad]

  // ガウシアンカーネル初期化（軽量版：σ=0.8, 半径=2, 5×5カーネル）
  GaussianKernel gaussian_kernel(0.8, 2);

  const std::string STORE_ROOT_DIR_NAME = "slam_result_251021-3";
  // ディレクトリが存在しない場合は作成
  if (!fs::exists(STORE_ROOT_DIR_NAME)) {
    fs::create_directories(STORE_ROOT_DIR_NAME);
  }

  std::ofstream fout_mapInfo(STORE_ROOT_DIR_NAME + "/mapInfo.lua");
  if (!fout_mapInfo) {
    std::cerr << "ファイルを開けませんでした。\n";
    return 1;
  }
  fout_mapInfo << "local mapInfo = {\n"
    //<< "\toriginX = " << originX << ",\n" 
    //<< "\toriginY = " << originY << ",\n"
    //<< "\tCSIZE = " << CSIZE << ",\n"
    //<< "\tminX = " << minX << ",\n"
    //<< "\tminY = " << minY << ",\n"
    //<< "\tmaxX = " << maxX << ",\n"
    //<< "\tmaxY = " << maxY << ",\n"
    //<< "\tmargin = 50,\n"
    //<< "}\n"
    << "return mapInfo\n";

  // LiDAR角度テーブルを初期化
  initialize_lidar_tables();

  // 読み込むLiDARデータのパス
#if 1
  const std::string PATH_TO_URGLOG_T = "./2025/10/05/184420/urglog_t";
  const std::string PATH_TO_URGLOG_B = "./2025/10/05/184420/urglog_b";
#endif

#if 0
  const std::string PATH_TO_URGLOG_T = "./2025/10/20/162619/urglog_t";
  const std::string PATH_TO_URGLOG_B = "./2025/10/20/162619/urglog_b";
#endif

  // LASERSCANRTの総数を事前に取得
  int total_data_count = count_laserscanrt_lines(PATH_TO_URGLOG_T) + 
    count_laserscanrt_lines(PATH_TO_URGLOG_B);
  std::cout << "総データ数: " << total_data_count << " (t:" << count_laserscanrt_lines(PATH_TO_URGLOG_T) 
    << ", b:" << count_laserscanrt_lines(PATH_TO_URGLOG_B) << ")" << std::endl;

  MergedLaserStream stream(PATH_TO_URGLOG_T, PATH_TO_URGLOG_B, LIDAR_DIRECTION_SKIP);

  // 確率的占有地図はSubMap内で管理
  std::vector<Pose> robot_poses;

  double current_x = 0;
  double current_y = 0;
  double current_a = 0;

  double p_odo_x = 0;
  double p_odo_y = 0;
  double p_odo_a = 0;

  // 累積走行距離計算用
  double global_total_distance = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  bool first_pose = true;

  // 部分地図管理用変数
  std::vector<SubMap> completed_submaps;         // 完成した部分地図
  SubMap current_submap;                         // 現在構築中の部分地図
  int next_submap_id = 0;                        // 次の部分地図ID
  //const double SUBMAP_DISTANCE = 5.0;            // 部分地図の区切り距離[m]
  const double SUBMAP_DISTANCE = std::numeric_limits<double>::infinity(); // 部分地図で分割したくないときは，正の無限大を使う
  double current_submap_local_distance = 0.0;    // 現在の部分地図内での累積走行距離
  bool submap_initialized = false;               // 最初の部分地図が初期化されたか

  int loop = 0;
  while (!stream.is_finished()) {
    LaserData current_data = stream.getNextScan();
    if (!current_data.valid) {
      continue;
    }

    // 選択されたデータを処理（不要なコピーを削除）
    long long timestamp = current_data.timestamp;
    long max_r = current_data.max_r;

    // センサー種別を文字列として準備
    // update_status_display の表示用
    std::string sensor_name = (current_data.sensor_type == 't') ? "Top" : "Bottom";

    // SLAM処理開始
    if(loop == 0) {
      // 最初の部分地図を初期化
      Pose initial_pose(timestamp, current_x, current_y, current_a);
      current_submap = SubMap(next_submap_id++, global_total_distance, initial_pose, CSIZE);
      current_submap_local_distance = 0.0; // 新しい部分地図開始時にリセット
      Pose relative_pose_origin(timestamp, 0.0, 0.0, 0.0);
      current_submap.trajectory.push_back(relative_pose_origin);
            current_submap.local_gmap = update_map(current_submap.local_gmap, current_data.points, 0.0, 0.0, 0.0,
                                                   current_submap.LOCAL_WIDTH, current_submap.LOCAL_HEIGHT,
                                                   current_submap.LOCAL_ORIGIN_X, current_submap.LOCAL_ORIGIN_Y, current_submap.LOCAL_CSIZE);
      robot_poses.emplace_back(timestamp, current_x, current_y, current_a);

      loop++;
      continue;
    }

    double dth = acos(1 - CSIZE*CSIZE/(2*(max_r/1000.0)*(max_r/1000.0)));
    double best_x, best_y, best_a, best_eval;

    // Localize within the current submap to get the new relative pose
    Pose prev_relative_pose = current_submap.trajectory.back();
    double rel_x, rel_y, rel_a;
        std::tie(rel_x, rel_y, rel_a, best_eval) = optimize_de(current_submap.local_gmap, current_data.points,
                                                             prev_relative_pose.x, prev_relative_pose.y, prev_relative_pose.a,
                                                             current_submap.LOCAL_ORIGIN_X, current_submap.LOCAL_ORIGIN_Y, current_submap.LOCAL_CSIZE, dth,
                                                             current_submap.LOCAL_WIDTH, current_submap.LOCAL_HEIGHT,
                                                             Wxy, Wa, 200, 100, 0.5, 0.2, &gaussian_kernel);
    // Convert the new relative pose to a global pose for logging and distance calculation
    double cos_start = cos(current_submap.start_pose.a);
    double sin_start = sin(current_submap.start_pose.a);
    best_x = current_submap.start_pose.x + rel_x * cos_start - rel_y * sin_start;
    best_y = current_submap.start_pose.y + rel_x * sin_start + rel_y * cos_start;
    best_a = current_submap.start_pose.a + rel_a;
    best_a = normalize_th(best_a);

    // 推定結果を固定表示関数で表示
    double angle_deg = best_a * 180.0 / M_PI;
    update_status_display(loop, sensor_name, timestamp, best_eval, best_x, best_y, angle_deg, total_data_count, global_total_distance);

    current_x = best_x;
    current_y = best_y;
    current_a = best_a;

    // 累積走行距離を計算
    if (!first_pose) {
      double distance_increment = sqrt((current_x - prev_x) * (current_x - prev_x) + 
                                       (current_y - prev_y) * (current_y - prev_y));
      global_total_distance += distance_increment;
    } else {
      first_pose = false;
    }
    prev_x = current_x;
    prev_y = current_y;

    // 現在の部分地図内での累積走行距離を計算
    double local_distance_increment = sqrt(pow(rel_x - prev_relative_pose.x, 2) + pow(rel_y - prev_relative_pose.y, 2));
    current_submap_local_distance += local_distance_increment;

    // 現在の部分地図に常にデータを蓄積
    current_submap.laser_data_sequence.push_back(current_data);
    current_submap.trajectory.emplace_back(timestamp, rel_x, rel_y, rel_a);

    // 部分地図内での走行距離チェック：新しい部分地図への切り替え
    if (current_submap_local_distance >= SUBMAP_DISTANCE) {
      // 現在の部分地図を完了（境界フレームは最後のデータとして既に追加済み）
      current_submap.global_end_distance = global_total_distance;

      // 部分地図の姿勢推定と地図構築を実行
      current_submap.build_submap();

      // 完成した部分地図をファイルに保存
      current_submap.save_submap_data(STORE_ROOT_DIR_NAME);

      // 完成した部分地図を保存
      completed_submaps.push_back(current_submap);

      // 統合地図を表示
      // create_and_show_integrated_map(completed_submaps); // 中間表示は最終出力時に統合するため一旦コメントアウト

      // 新しい部分地図を開始（境界フレームの姿勢を開始点とする）
      Pose new_start_pose;
      new_start_pose.ts = timestamp;
      new_start_pose.x = current_x;
      new_start_pose.y = current_y;
      new_start_pose.a = current_a;
      current_submap = SubMap(next_submap_id++, global_total_distance, new_start_pose, CSIZE);
      current_submap_local_distance = 0.0; // 新しい部分地図開始時にリセット

      // 境界フレームを新部分地図の最初のデータとして追加（境界データ重複）
      current_submap.laser_data_sequence.push_back(current_data);
      Pose relative_pose_origin;
      relative_pose_origin.ts = timestamp;
      relative_pose_origin.x = 0.0;
      relative_pose_origin.y = 0.0;
      relative_pose_origin.a = 0.0;
      current_submap.trajectory.push_back(relative_pose_origin);



      std::cout << "SubMap " << current_submap.submap_id << " 開始 距離:" << global_total_distance << "m（境界データ重複）" << std::endl;
    }

    // Save pose to robot_poses
    robot_poses.emplace_back(timestamp, current_x, current_y, current_a);

    // メインループでのLiDAR点数をデバッグ出力（最初のフレームのみ）
    if (loop == 1) {
      std::cout << "[デバッグ] メインループ最初のフレーム:" << std::endl;
      std::cout << "  LiDAR点数: " << current_data.points.size() << std::endl;
      std::cout << "  センサータイプ: " << current_data.sensor_type << std::endl;
      std::cout << "  タイムスタンプ: " << current_data.timestamp << std::endl;
    }

    // Update the current submap's local map with the new relative pose
    const auto& latest_relative_pose = current_submap.trajectory.back();
        current_submap.local_gmap = update_map(current_submap.local_gmap, current_data.points,
                                               latest_relative_pose.x, latest_relative_pose.y, latest_relative_pose.a,
                                               current_submap.LOCAL_WIDTH, current_submap.LOCAL_HEIGHT,
                                               current_submap.LOCAL_ORIGIN_X, current_submap.LOCAL_ORIGIN_Y, current_submap.LOCAL_CSIZE);
    // 移動体除去処理（Bottomセンサーデータのみ使用）
    if (current_data.sensor_type == 'b') {
          current_submap.local_gmap = remove_moving_objects(current_submap.local_gmap, current_data,
                                                            latest_relative_pose.x, latest_relative_pose.y, latest_relative_pose.a,
                                                            current_submap.LOCAL_WIDTH, current_submap.LOCAL_HEIGHT,
                                                            current_submap.LOCAL_ORIGIN_X, current_submap.LOCAL_ORIGIN_Y, current_submap.LOCAL_CSIZE);    }

    if (loop % 1 == 0) {
      current_submap.show_submap_progress(current_submap.trajectory.size() - 1);
    }
    loop++;
  }

  // 最後の部分地図を処理
  if (!current_submap.laser_data_sequence.empty()) {
    current_submap.global_end_distance = global_total_distance;
    current_submap.build_submap();

    // 最後の部分地図をファイルに保存
    current_submap.save_submap_data(STORE_ROOT_DIR_NAME);

    completed_submaps.push_back(current_submap);
    std::cout << "最後のSubMap " << current_submap.submap_id << " を処理完了" << std::endl;

    // 最終的な統合地図を表示
    create_and_show_integrated_map(completed_submaps);
  }

  std::cout << "全部分地図の構築完了 総数: " << completed_submaps.size() << std::endl;

  std::ofstream fout(STORE_ROOT_DIR_NAME + "/robot_poses.txt");
  for(int i = 0; i < robot_poses.size(); i++) {
    fout << robot_poses[i].ts << " " << robot_poses[i].x << " " << robot_poses[i].y << " " << robot_poses[i].a << "\n";
  }

  // 最終的な統合地図を生成
  std::cout << "最終的な統合地図を生成・保存します..." << std::endl;
  std::vector<std::vector<double>> final_map_data = create_and_show_integrated_map(completed_submaps);

  if (final_map_data.empty()) {
    std::cerr << "エラー: 最終地図が空です。保存処理を中断します。" << std::endl;
  } else {
    // 統合地図データを可視化用のcv::Matに変換
    int map_h = final_map_data.size();
    int map_w = final_map_data[0].size();
    cv::Mat integrated_map_img(map_h, map_w, CV_8UC3, cv::Scalar(50, 50, 50));

    const double HIGH_THRESHOLD = 0.405; // 60%
    for (int y = 0; y < map_h; y++) {
      for (int x = 0; x < map_w; x++) {
        double log_odds = final_map_data[y][x];
        if (log_odds > HIGH_THRESHOLD) {
          integrated_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // Obstacle
        } else if (log_odds < -HIGH_THRESHOLD) {
          integrated_map_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Free space
        }
      }
    }

    // 統合地図を保存
    std::string final_map_path = STORE_ROOT_DIR_NAME + "/integrated_occMap.png";
    cv::imwrite(final_map_path, integrated_map_img);
    std::cout << "統合占有地図を保存しました: " << final_map_path << std::endl;
  }

  std::cout << "Done." << std::endl;
  int key = cv::waitKey(0);
  return 0;
}

// SubMapクラスのsave_submap_data()メソッドの実装
void SubMap::save_submap_data(const std::string& base_dir) {
  // サブディレクトリ作成
  std::string submap_dir = base_dir + "/submaps/submap_" + 
    std::string(3 - std::to_string(submap_id).length(), '0') + 
    std::to_string(submap_id);

  // ディレクトリ作成
  std::string mkdir_cmd = "mkdir -p " + submap_dir;
  system(mkdir_cmd.c_str());

  // 1. メタデータ保存 (mapInfo.yaml)
  std::ofstream metadata_file(submap_dir + "/mapInfo.yaml");
  metadata_file << "submap_id: " << submap_id << std::endl;
  metadata_file << "global_start_distance: " << global_start_distance << std::endl;
  metadata_file << "global_end_distance: " << global_end_distance << std::endl;
  metadata_file << "start_pose:" << std::endl;
  metadata_file << "  x: " << start_pose.x << std::endl;
  metadata_file << "  y: " << start_pose.y << std::endl;
  metadata_file << "  a: " << start_pose.a << std::endl;
  metadata_file << "  timestamp: " << start_pose.ts << std::endl;
  metadata_file << "bounds:" << std::endl;
  metadata_file << "  min_x: " << min_x << std::endl;
  metadata_file << "  max_x: " << max_x << std::endl;
  metadata_file << "  min_y: " << min_y << std::endl;
  metadata_file << "  max_y: " << max_y << std::endl;
  metadata_file << "frame_count: " << laser_data_sequence.size() << std::endl;
  metadata_file.close();

  // 2. ローカル地図保存 (local_gmap.yml) - OpenCV FileStorage使用
  cv::FileStorage fs(submap_dir + "/local_gmap.yml", cv::FileStorage::WRITE);
  cv::Mat gmap_mat(LOCAL_HEIGHT, LOCAL_WIDTH, CV_64F);
  for (int y = 0; y < LOCAL_HEIGHT; y++) {
    for (int x = 0; x < LOCAL_WIDTH; x++) {
      gmap_mat.at<double>(y, x) = local_gmap[y][x];
    }
  }
  fs << "local_gmap" << gmap_mat;
  fs << "width" << LOCAL_WIDTH;
  fs << "height" << LOCAL_HEIGHT;
  fs << "origin_x" << LOCAL_ORIGIN_X;
  fs << "origin_y" << LOCAL_ORIGIN_Y;
  fs << "cell_size" << LOCAL_CSIZE;
  fs.release();

  // 3. 軌跡データ保存 (trajectory.txt) - メインループで推定された軌跡
  std::ofstream traj_file(submap_dir + "/trajectory.txt");
  double cumulative_submap_distance = 0.0;
  double prev_x_submap = 0.0; // trajectory[0].x is 0.0
  double prev_y_submap = 0.0; // trajectory[0].y is 0.0

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& pose = trajectory[i];
    if (i > 0) {
      double segment_length = sqrt(pow(pose.x - prev_x_submap, 2) + pow(pose.y - prev_y_submap, 2));
      cumulative_submap_distance += segment_length;
    }

    // "スタート時点からの累積走行距離" = この部分地図の開始時点からの累積距離 + 部分地図内での累積距離
    // global_start_distance is the total distance when this submap started.
    double total_run_distance_at_pose = global_start_distance + cumulative_submap_distance;

    traj_file << pose.ts << " " << pose.x << " " << pose.y << " " << pose.a << " "
      << std::fixed << std::setprecision(3) << cumulative_submap_distance << " "
      << std::fixed << std::setprecision(3) << total_run_distance_at_pose << std::endl;

    prev_x_submap = pose.x;
    prev_y_submap = pose.y;
  }
  traj_file.close();

  // 4. LiDARデータ保存 - センサ別に分離
  std::ofstream laser_b_file(submap_dir + "/laser_data_b.txt");
  std::ofstream laser_t_file(submap_dir + "/laser_data_t.txt");

  for (const auto& laser_data : laser_data_sequence) {
    if (!laser_data.valid) continue;

    std::ofstream* target_file = (laser_data.sensor_type == 'b') ? &laser_b_file : &laser_t_file;

    // urglog形式で出力
    *target_file << "LASERSCANRT " << laser_data.timestamp << " ";
    *target_file << laser_data.points.size() * 3 << " "; // count (3エコー分)
    *target_file << laser_data.start_angle << " " << laser_data.end_angle << " ";
    *target_file << laser_data.delta_th << " 3 "; // max_echo_size = 3

    // 距離データ（3エコー形式で出力）
    for (const auto& point : laser_data.points) {
      double range_mm = sqrt(point.x * point.x + point.y * point.y) * 1000.0;
      long r = static_cast<long>(range_mm);
      *target_file << r << " " << r << " " << r << " "; // 同じ値を3回（簡易版）
    }

    *target_file << "0.0 0.0 0.0 " << laser_data.timestamp << std::endl;
  }

  laser_b_file.close();
  laser_t_file.close();

  std::cout << "部分地図 " << submap_id << " を保存しました: " << submap_dir << std::endl;
}
