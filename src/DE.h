#ifndef __DE_H__
#define __DE_H__

#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <iomanip>  // std::setprecision
#include <tuple>
#include <random>

#include "Urg2d.h"

namespace fs = std::filesystem;

struct Point {
  double x;
  double y;

  Point() {};
  Point(double x, double y) {
    this->x = x;
    this->y = y;
  }
};

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

std::tuple<int, int> xy2index(double xd, double yd, double csize, int ox, int oy) {
  int ix = static_cast<int>( xd / csize) + ox;
  int iy = static_cast<int>(-yd / csize) + oy;
  return std::make_tuple(ix, iy);
}

std::vector<std::vector<int>> update_map(std::vector<std::vector<int>> &gmap,
    std::vector<Point> &pt1,
    double current_x, double current_y, double current_a,
    int width, int height, int ox, int oy, double csize) {
  int ix, iy;
  double cs = cos(current_a);
  double sn = sin(current_a);
  for (size_t i = 0; i < pt1.size(); i++) {
    double xd = pt1[i].x * cs - pt1[i].y * sn + current_x;
    double yd = pt1[i].x * sn + pt1[i].y * cs + current_y;
    auto [ix, iy] = xy2index(xd, yd, csize, ox, oy);
    if (0 <= ix && ix < width && 0 <= iy && iy < height) {
      gmap[iy][ix] = 1;
    }
  }

  return gmap;
}

cv::Mat gmap_show(std::vector<std::vector<int>> &gmap, double width, double height) {
  cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(123, 123, 123));
  //cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(0xed, 0xe7, 0xe6));
  for (int j = 0; j < gmap.size(); j++) {
    for (int i = 0; i < gmap[0].size(); i++) {
      if (gmap[j][i] == 1) {
        img.at<cv::Vec3b>(j, i)[0] = 0;
        img.at<cv::Vec3b>(j, i)[1] = 0;
        img.at<cv::Vec3b>(j, i)[2] = 0;
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

int match_count(std::vector<std::vector<int>> &gmap,
    std::vector<LSP> &pt,
    //std::vector<Point> &pt,
    double cx, double cy, double ca,
    double csize, int originX, int originY, int width, int height) {
  int eval = 0;
  double cs = cos(ca);
  double sn = sin(ca);
  for (int ind = 0; ind < pt.size(); ind += 1) {
    double xd = pt[ind].x * cs - pt[ind].y * sn + cx;
    double yd = pt[ind].x * sn + pt[ind].y * cs + cy;
    auto [ix, iy] = xy2index(xd, yd, csize, originX, originY);
    if (0 <= ix && ix < width && 0 <= iy && iy < height) {
      if (gmap[iy][ix] == 1) {
        eval += 1;
      }
    }
  }
  return eval;
}

#if 0
std::tuple<double, double, double, double> optimize_greedy(
    std::vector<std::vector<int>> &gmap, std::vector<Point> &pt,
    double current_x, double current_y, double current_a,
    int originX, int originY,
    double csize, double dth,
    int width, int height,
    double Wxy, double Wa) {
  double best_x, best_y, best_a;
  double best_eval = 0;
  for (double dx = -Wxy; dx < Wxy; dx += csize) {
    double cx = current_x + dx;
    for (double dy = -Wxy; dy < Wxy; dy += csize) {
      double cy = current_y + dy;
      for (double da = -Wa; da < Wa; da += dth) {
        double ca = current_a + da;
        int eval = match_count(gmap, pt, cx, cy, ca, csize, originX, originY, width, height);
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
#endif

// 乱数生成エンジンの準備
std::random_device rd; // ハードウェア乱数生成器
std::mt19937 gen(rd()); // メルセンヌ・ツイスタ法の乱数生成器
// -1から1の間の一様分布を定義
std::uniform_real_distribution<> dis(-1.0, 1.0);

std::tuple<double, double, double, double> optimize_de(
    std::vector<std::vector<int>> &gmap, std::vector<LSP> &lsp,
    double current_x, double current_y, double current_a,
    int originX, int originY,
    double csize, double dth,
    int width, int height,
    double Wxy, double Wa,
    int population_size, int generations, double F, double CR) {

  current_a = normalize_th(current_a);

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
      double va = a1 + F * (a2 - a3);

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
      //std::vector<Point> pt;
      //for (auto l: lsp) {
      //  pt.emplace_back(l.x, l.y);
      //}
      double eval = match_count(gmap, lsp, trial_x, trial_y, trial_a, csize, originX, originY, width, height);
      double _rot = current_a - trial_a;
      double _tran = std::hypot(current_x - trial_x, current_y - trial_y);
      _rot = normalize_th(_rot);
      double diff = _tran + 0.1*fabs(_rot);
      eval -= 1/(diff + 1e-4);

      // Selection: if trial is better, replace the target individual
      if (eval > best_eval) {
        best_eval = eval + diff;
        best_x = trial_x;
        best_y = trial_y;
        best_a = trial_a;
      }

      _rot = current_a - std::get<2>(population[i]);
      _tran = std::hypot(current_x - std::get<0>(population[i]), current_y - std::get<1>(population[i]));
      _rot = normalize_th(_rot);
      diff = _tran + 0.1*fabs(_rot);
      double s_eval = match_count(gmap, lsp, std::get<0>(population[i]), std::get<1>(population[i]), std::get<2>(population[i]), csize, originX, originY, width, height);
      s_eval -= 1/(diff + 1e-4);

      if (eval > s_eval) {
        population[i] = std::make_tuple(trial_x, trial_y, trial_a);
      }
    }
  }

  return std::make_tuple(best_x, best_y, best_a, best_eval);
}

/*
   int main (int argc, char *argv[]) {
   int DATA_SKIP = 1;
   double csize = 0.025;  // [m] 格子の解像度
// 図書館前~体育館
double minX = -85.0;
double minY = -160.0;
double maxX = 170.0;
double maxY = 35.0;
int originX = int(fabs(minX)/csize);
int originY = int(fabs(maxY)/csize);
int width = int((maxX - minX)/csize);
int height = int((maxY - minY)/csize);

double Wxy = 0.5;      // 探索範囲[m]
double Wa  = M_PI/5;    // 角度[rad]

const std::string STORE_ROOT_DIR_NAME = "slam_result_250405-1";
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
<< "\toriginX = " << originX << ",\n"
<< "\toriginY = " << originY << ",\n"
<< "\tcsize = " << csize << ",\n"
<< "\tminX = " << minX << ",\n"
<< "\tminY = " << minY << ",\n"
<< "\tmaxX = " << maxX << ",\n"
<< "\tmaxY = " << maxY << ",\n"
<< "\tmargin = 50,\n"
<< "}\n"
<< "return mapInfo\n";

const std::string PATH_TO_URGLOG = "../data250405a0/urglog";
try {
// ファイルコピー
fs::copy_file(PATH_TO_URGLOG, STORE_ROOT_DIR_NAME + "/urglog", fs::copy_options::overwrite_existing);
std::cout << "コピー成功\n";
} catch (const fs::filesystem_error& e) {
std::cerr << "ファイルコピーに失敗しました: " << e.what() << '\n';
}

std::vector<std::vector<int>> gmap(height, std::vector<int>(width, 123));

std::vector<Point> pt1;
std::vector<Pose> robot_poses;

double current_x = 0;
double current_y = 0;
double current_a =-5*M_PI/180.0;

double p_odo_x = 0;
double p_odo_y = 0;
double p_odo_a =-5*M_PI/180.0;

std::ifstream inFile;

inFile.open(PATH_TO_URGLOG);
std::string type;
int loop = 0;
while(1) {
std::vector<Point> pt;
inFile >> type;
if(inFile.eof()) break;
if (type == "LASERSCANRT") { // マルチエコーに対応済み
long long timestamp;
long long timestamp_end;
int count;
double START_ANGLE, END_ANGLE, deltaTH;
int max_echo_size;
long intensity;

inFile >> timestamp >> count >> START_ANGLE >> END_ANGLE >> deltaTH >> max_echo_size;

long max_r = 0;
long r;
double th = START_ANGLE * M_PI/180.0;
for (int i = 0; i < count/(max_echo_size+1); i+=DATA_SKIP) {
  inFile >> r;
  if(max_r < r) {
    max_r = r;
  }
  if (r > 200) {
    double x = (double)r * cos(th) / 1000.0;
    double y = (double)r * sin(th) / 1000.0;
    pt.emplace_back(x, y);
  }
  th += deltaTH * M_PI/180.0;
  // マルチエコーと反射強度を読み飛ばす
  inFile >> r >> r >> intensity;
}
double odo_x, odo_y, odo_a;
inFile >> odo_x >> odo_y >> odo_a >> timestamp_end;

if(loop == 0) {
  gmap = update_map(gmap, pt, current_x, current_y, current_a, width, height, originX, originY, csize);
  gmap_show(gmap, width, height);
  robot_poses.emplace_back(timestamp, current_x, current_y, current_a);
  loop++;

  p_odo_x = odo_x;
  p_odo_y = odo_y;
  p_odo_a = odo_a;
  continue;
}

double delta_x = odo_x - p_odo_x;
double delta_y = odo_y - p_odo_y;
double delta_a = odo_a - p_odo_a;
double relative_x = cos(p_odo_a) * delta_x + sin(p_odo_a) * delta_y;
double relative_y =-sin(p_odo_a) * delta_x + cos(p_odo_a) * delta_y;
double relative_a = delta_a;
relative_a = normalize_th(relative_a);

p_odo_x = odo_x;
p_odo_y = odo_y;
p_odo_a = odo_a;

double dth = acos(1 - csize*csize/(2*(max_r/1000.0)*(max_r/1000.0)));
double best_eval = 0;
double best_x = current_x + cos(current_a) * relative_x - sin(current_a) * relative_y;
double best_y = current_y + sin(current_a) * relative_x + cos(current_a) * relative_y;
double best_a = current_a + relative_a;

current_x = best_x;
current_y = best_y;
current_a = best_a;

//auto start = std::chrono::high_resolution_clock::now();
//std::tie(best_x, best_y, best_a, best_eval) = optimize_greedy(gmap, pt, current_x, current_y, current_a,
//                                            originX, originY, csize, dth, width, height, Wxy, Wa);

std::tie(best_x, best_y, best_a, best_eval) = optimize_de(gmap, pt, current_x, current_y, current_a,
    originX, originY, csize, dth, width, height,
    Wxy, Wa, 50, 30, 0.5, 0.5);

current_x = best_x;
current_y = best_y;
current_a = best_a;
//auto end = std::chrono::high_resolution_clock::now();
//std::chrono::duration<double> elapsed = end - start;
gmap = update_map(gmap, pt, current_x, current_y, current_a, width, height, originX, originY, csize);
//std::cout << std::fixed << std::setprecision(3);  // 小数点以下2桁
//std::cout << "Elapsed time: " << elapsed.count() << "[s]" << " ";
//std::cout << std::fixed << std::setprecision(3)  // 小数点以下2桁
//        << current_x << " "
//        << current_y << " "
//        << std::fixed << std::setprecision(1)  // 小数点以下3桁
//        << current_a * 180/M_PI << " "
//        << timestamp << "\n";
robot_poses.emplace_back(timestamp, current_x, current_y, current_a);
if (loop % 250 == 0) {
  cv::Mat img = gmap_show(gmap, width, height);
  int key = cv::waitKey(10);
}
loop++;
}
}

std::ofstream fout(STORE_ROOT_DIR_NAME + "/robot_poses.txt");
for(int i = 0; i < robot_poses.size(); i++) {
  fout << robot_poses[i].ts << " " << robot_poses[i].x << " " << robot_poses[i].y << " " << robot_poses[i].a << "\n";
}

cv::Mat img = gmap_show(gmap, width, height);
cv::imwrite(STORE_ROOT_DIR_NAME + "/map.png", img);

int key = cv::waitKey(0);
return 0;
}
*/
#endif
