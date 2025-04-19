#include "DELFM.h"

DELFM::DELFM() {
  ;
}

DELFM::DELFM(double Wxy, double Wa, int population_size, int generations, double F, double CR) {
  double max_r = 30000;
  this->dth = acos(1 - csize*csize/(2*(max_r/1000.0)*(max_r/1000.0)));
  this->Wxy = Wxy;
  this->Wa = Wa;
  this->population_size = population_size;
  this->generations = generations;
  this->F = F;
  this->CR = CR;

  // 乱数生成エンジンの準備
  std::random_device rd; // ハードウェア乱数生成器
  gen = std::mt19937(rd()); // メルセンヌ・ツイスタ法の乱数生成器
  // -1から1の間の一様分布を定義
  dis = std::uniform_real_distribution<>(-1.0, 1.0);
}

void DELFM::set_lfm(std::string path) {
  std::fstream inFile(path);
  int rows, cols;
  double val;

  inFile >> rows >> cols;
  LFM = cv::Mat(rows, cols, CV_64FC1);
  cv::Mat imgLFM(rows, cols, CV_8UC1);
  for (int iy = 0; iy < rows; iy++) {
    for (int ix = 0; ix < cols; ix++) {
      inFile >> val;
      LFM.at<double>(iy, ix) = val;
    }
  }

  // LFMの範囲を調べる
  double min = 1e9;
  double max = -1e9;
  for (int iy = 0; iy < LFM.rows; iy++) {
    for (int ix = 0; ix < LFM.cols; ix++) {
      if (LFM.at<double>(iy, ix) < 0) {
        continue;
      }
      if (min > LFM.at<double>(iy, ix)) {
        min = LFM.at<double>(iy, ix);
      }
      if (max < LFM.at<double>(iy, ix)) {
        max = LFM.at<double>(iy, ix);
      }
    }
  }
  for (int iy = 0; iy < imgLFM.rows; iy++) {
    for (int ix = 0; ix < imgLFM.cols; ix++) {
      if (LFM.at<double>(iy, ix) < 0) {
        imgLFM.at<uchar>(iy, ix) = 128;
      } else {
        unsigned char val;
        if (LFM.at<double>(iy, ix) < 1e-8)
          val = 0;
        else
          val = (LFM.at<double>(iy, ix) - min) / (max - min) * 255;
        imgLFM.at<uchar>(iy, ix) = val;
      }
    }
  }
  //cv::imwrite("lfm.png", imgLFM);
  //cv::imshow("lfm", imgLFM);
  //cv::waitKey();
}

void DELFM::set_mapInfo(std::string path) {
  YAML::Node mapInfo;
  try {
    mapInfo = YAML::LoadFile(path);
	} catch(YAML::BadFile &e) {
		std::cerr << "read error! yaml is not exist."<< std::endl;
    exit(1);
	}
  margin = mapInfo["margin"].as<int>();
  originX = margin + mapInfo["originX"].as<int>();
  originY = margin + mapInfo["originY"].as<int>();
  csize = mapInfo["csize"].as<double>();
}

double DELFM::measurement_model(const std::vector<LSP> &lsp,
    const double x, const double y, const double a) {
  // lspの点をLFMのインデックスに変換し尤度を得る
  double q = 0;
  for (auto lp: lsp) {
    double gx = x + lp.x * cos(a) - lp.y * sin(a);
    double gy = y + lp.x * sin(a) + lp.y * cos(a);
    int ix = originX + gx/csize;
    int iy = originY - gy/csize;
    if (ix >= 0 && ix < LFM.cols && iy >= 0 && iy < LFM.rows) {
      if (LFM.at<double>(iy, ix) > 0)
        q += LFM.at<double>(iy, ix);
    }
  }
  return q;
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

std::tuple<double, double, double, double> DELFM::optimize_de(
    std::vector<LSP> &lsp, double current_x, double current_y, double current_a) {

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

      double eval = measurement_model(lsp, trial_x, trial_y, trial_a);

      // Selection: if trial is better, replace the target individual
      if (eval > best_eval) {
        best_eval = eval;
        best_x = trial_x;
        best_y = trial_y;
        best_a = trial_a;
      }

      double s_eval = measurement_model(lsp, std::get<0>(population[i]), std::get<1>(population[i]), std::get<2>(population[i]));

      if (eval > s_eval) {
        population[i] = std::make_tuple(trial_x, trial_y, trial_a);
      }
    }
  }

  return std::make_tuple(best_x, best_y, best_a, best_eval);
}
