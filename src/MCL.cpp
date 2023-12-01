#include "MCL.h"

MCL::MCL() {
  ;
}

MCL::MCL(const Pose2d pose) {
  NUM = 2000;
  rng = cv::RNG(cv::getTickCount());

  for (int i = 0; i < NUM; i++) {
    particle.emplace_back(
        pose.x + rng.uniform(-0.5, 0.5),
        pose.y + rng.uniform(-0.5, 0.5),
        pose.a + rng.uniform(-10/180.0*M_PI, 10/180.0*M_PI));
  }
}

void MCL::set_currentPose(const Pose2d pose) {
  NUM = 1000;

  particle.clear();
  for (int i = 0; i < NUM; i++) {
    particle.emplace_back(
        pose.x + rng.uniform(-0.3, 0.3),
        pose.y + rng.uniform(-0.3, 0.3),
        pose.a + rng.uniform(-15/180.0*M_PI, 15/180.0*M_PI));
  }
}

void MCL::set_lfm(std::string path) {
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
}

void MCL::set_mapInfo(std::string path) {
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

void MCL::motion_update(double v_, double w_, double dt) {
  const std::vector<double> a{0.01, 0.01, 0.01, 0.01, 0.01};
  for (int i = 0; i < NUM; i++) {
    double v = v_ + rng.gaussian(a[0]*v_*v_ + a[1]*w_*w_);
    double w = w_ + rng.gaussian(a[2]*v_*v_ + a[3]*w_*w_);
    double r =      rng.gaussian(a[4]*v_*v_ + a[5]*w_*w_);
    if (fabs(w) < 1e-6) {
      particle[i].x += v * cos(particle[i].a) * dt;
      particle[i].y += v * sin(particle[i].a) * dt;
    } else {
      particle[i].x += -v/w*sin(particle[i].a) + v/w*sin(particle[i].a + w*dt);
      particle[i].y +=  v/w*cos(particle[i].a) - v/w*cos(particle[i].a + w*dt);
      particle[i].a += w*dt + r*dt;
    }
    if (particle[i].a < -M_PI) particle[i].a += 2*M_PI;
    if (particle[i].a >  M_PI) particle[i].a -= 2*M_PI;
  }
}

void MCL::measurement_update(const std::vector<LSP> &lsp) {
  double sum = 0;
  for (int i = 0; i < particle.size(); i++) {
    particle[i].weight = measurement_model(lsp,
        particle[i].x, particle[i].y, particle[i].a);
    sum += particle[i].weight;
  }
  // normalize
  for (int i = 0; i < particle.size(); i++) {
    particle[i].weight /= sum;
  }
}

double MCL::measurement_model(const std::vector<LSP> &lsp,
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

std::vector<Pose2d> MCL::get_particle_set() {
  return particle;
}

void MCL::resampling() {
  double r = rng.uniform(0.0, 1.0/NUM);
  double c = particle[0].weight;
  int i = 0;
  double U;
  std::vector<Pose2d> tmp(NUM);
  for (int m = 0; m < NUM; ++m) {
			U = r + m / NUM;
			while (U > c) {
				i += 1;
				c += particle[i].weight;
			}
			tmp[m].x = particle[i].x;
			tmp[m].y = particle[i].y;
			tmp[m].a = particle[i].a;
			tmp[m].weight = particle[i].weight;
		}
		// パーティクルを更新
		for (int i = 0; i < NUM; ++i) {
			particle[i].x = tmp[i].x;
			particle[i].y = tmp[i].y;
			particle[i].a = tmp[i].a;
			particle[i].weight = tmp[i].weight;
		}
}

void MCL::KLD_sampling(const std::vector<LSP> &lsp, const Pose2d &curX, const Pose2d &prevX) {
  // KDLサンプリングの諸パラメータ
  double M_x = 0.0;
  int M_k	   = 0;
  int M_xmin = 40;
  std::vector<Pose2i> bin;

  //１ステップ前のパーティクル数
  NUM = particle.size();
  std::vector<Pose2d> new_particle;
  for (int i = 0; (i < M_x || i < M_xmin); i++) {
    // 一つ前のパーティクル集合から一つをサンプリング
    int index = draw_index_with_probability(particle);
    // パーティクルを動作更新
    Pose2d new_p = sample_motion_model(particle[index], curX, prevX);
    // パーティクルの計測更新
    new_p.weight = measurement_model(lsp, new_p.x, new_p.y, new_p.a);
    // パーティクル集合に新しいパーティクルを追加
    new_particle.emplace_back(new_p);

    //新しいビンに追加されるかの判定
    Pose2i tmpBin;
    int bin_flag = 0;
    tmpBin.x = int(new_p.x / 0.5); //ヒストグラムの幅 0.5[m]
    tmpBin.y = int(new_p.y / 0.5); //ヒストグラムの幅 0.5[m]
    double tmp_angle_deg_0_360 = new_p.a/M_PI*180.0;
    if (tmp_angle_deg_0_360 < 0.0)
      tmp_angle_deg_0_360 = tmp_angle_deg_0_360 + 360.0;
    tmpBin.a = int(tmp_angle_deg_0_360/15.0); //ヒストグラムの幅 15[deg]
    for (auto b : bin) {
      if ((tmpBin.x == b.x) && (tmpBin.y == b.y) && (tmpBin.a == b.a)) {
        bin_flag = 1;
        break;
      }
    }

    //条件を満たせばビンに登録
    if (bin_flag == 0) {
      bin.push_back(tmpBin);
      M_k++;
      if (M_k > 1) {
        M_x = (M_k - 1) / (2 * 0.05) *
          pow(1.0 - 2.0 / (9.0 * (M_k - 1)) +
              sqrt((2.0 / (9.0 * (M_k - 1)))) * 2.33,
              3);
        // 2.33は(1-δ)が0.99のときの分位数(確率ロボティクスp238 5行目)
      }
    }
  }
  particle.clear();
  particle = new_particle;
  NUM = new_particle.size();

  double maxW = 0.0;
  int maxI;
  Pose2d estimatedPose;
  for (auto pt : particle) {
    if (maxW < pt.weight) {
      maxW = pt.weight;
      // 最大重みのパーティクル姿勢を推定結果とする
      estimatedPose = pt;
    }
  }
  estimatedPose.weight = maxW;
  estimatedPose.a = atan2(sin(estimatedPose.a), cos(estimatedPose.a));

  // Weight average
  double sum = 0.0;
  for (auto pt : particle) {
    sum += pt.weight;
  }

  double w_avg = sum / (double)particle.size();
  w_slow += alpha_slow * (w_avg - w_slow);
  w_fast += alpha_fast * (w_avg - w_fast);

  // Weight normalize
  for (int i = 0; i < particle.size(); i++) {
    particle[i].weight /= sum;
  }

  // AMCL core
  std::vector<Pose2d> amcl_particles;
  for (int i = 0; i < particle.size(); i++) {
    double pb = fmax(0.0, 1.0 - w_fast / w_slow);
    if (pb > rng.uniform(0.0, 1.0)) {
      Pose2d pt;
      pt.x = estimatedPose.x + rng.uniform(-0.5, 0.5);
      pt.y = estimatedPose.y + rng.uniform(-0.5, 0.5);
      pt.a = estimatedPose.a + rng.uniform(-20*M_PI/180.0, 20*M_PI/180.0);
      amcl_particles.emplace_back(pt);
    } else {
      int index = draw_index_with_probability(particle);
      amcl_particles.emplace_back(particle[index]);
    }
  }
  // Initialize particle weight based amcl
  for (int i = 0; i < amcl_particles.size(); i++) {
    amcl_particles[i].weight =
      measurement_model(lsp, amcl_particles[i].x, amcl_particles[i].y, amcl_particles[i].a);
  }

  // refresh particle set
  particle.clear();
  particle = amcl_particles;
  NUM = particle.size();
  //std::cout << particle.size() << "\t" << w_slow << "\t" << w_fast << "\n";
}

int MCL::draw_index_with_probability(const std::vector<Pose2d>& particle) {
  double maxW = 0;
  for (auto p: particle) {
    if (maxW < p.weight) {
      maxW = p.weight;
    }
  }

  for (;;) {
    int index = rng.uniform(0, particle.size());
    double wr = rng.uniform(double(0), maxW);
    if (particle[index].weight > wr) {
      return index;
    }
  }
}

Pose2d MCL::sample_motion_model(const Pose2d& particle,
    const Pose2d& curX, const Pose2d& prevX) {
  // オドメトリ動作モデルのノイズ
  const std::vector<double> a{0.01, 0.01, 0.01, 0.01};
  // オドメトリ動作モデルパラメータ
  double _rot1, _tran, _rot2, rot1, rot2, tran;

  // オドメトリ指令を計算
  double dx = curX.x - prevX.x;
  double dy = curX.y - prevX.y;
  double da = curX.a - prevX.a;

  _tran	= std::hypot(dx, dy);

  if (_tran < 1e-6) { // 場所が変わっていない処理をする
    if (fabs(da) < 1e-6) { // 向きも変わっていない場合
      _rot1 = 0.0;
      _rot2 = 0.0;
    } else { // 向きは変わっている
      _rot1 = 0.0;
      _rot2 = da;
    }
  } else {
    _rot1 = atan2(dy, dx) - prevX.a;
    if (_rot1 > M_PI) _rot1 -= 2*M_PI;
    else if (_rot1 < -M_PI) _rot1 += 2*M_PI;

    if (fabs(_rot1) > M_PI/2) {   // backward
      _rot1 = _rot1 - M_PI;
      _tran = -std::hypot(dx, dy);
    } else {
      _tran = std::hypot(dx, dy);
    }

    _rot2 = curX.a - prevX.a - _rot1;
    if (_rot2 > M_PI) _rot2 -= 2*M_PI;
    else if (_rot2 < -M_PI) _rot2 += 2*M_PI;
  }

#if 1
  _rot1 = atan2(sin(_rot1), cos(_rot1));
  _rot2 = atan2(sin(_rot2), cos(_rot2));
#endif

  rot1 = _rot1 - rng.gaussian(a[0] * _rot1 * _rot1 + a[1] * _tran * _tran);
  tran = _tran - rng.gaussian(a[2] * _tran * _tran + a[3] * _rot1 * _rot1 + a[3] * _rot2 * _rot2);
  rot2 = _rot2 - rng.gaussian(a[0] * _rot2 * _rot2 + a[1] * _tran * _tran);

  Pose2d new_particle;
  new_particle.x = particle.x + tran * cos(particle.a + rot1);
  new_particle.y = particle.y + tran * sin(particle.a + rot1);
  new_particle.a = particle.a + rot1 + rot2;
  new_particle.a = atan2(sin(new_particle.a), cos(new_particle.a));

  //std::cout << new_particle.x << " " << new_particle.y << " " << new_particle.a << "\n";

  return new_particle;
}

Pose2d MCL::get_best_pose() {
  double best = -999999;
  Pose2d result;
  for (auto p: particle) {
    if (best < p.weight) {
      best = p.weight;
      result = p;
    }
  }
  return result;
}

std::tuple<double, double> MCL::get_w() {
  return std::make_tuple(w_fast, w_slow);
}
