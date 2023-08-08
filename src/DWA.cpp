#include <tuple>
#include "DWA.h"

DynamicWindowApproach::DynamicWindowApproach(YAML::Node &coyomi_yaml) {
  // DWAのパラメータを読み出す
  limit_distance       = coyomi_yaml["DWA"]["limit_distance"].as<double>();
  dv                   = coyomi_yaml["DWA"]["dv"].as<double>();
  dw                   = coyomi_yaml["DWA"]["dw"].as<double>();
  acceleration_limit   = coyomi_yaml["DWA"]["acceleration_limit"].as<double>();
  w_acceleration_limit = coyomi_yaml["DWA"]["w_acceleration_limit"].as<double>() * M_PI/180.0;  // rad/s/s
  T                    = coyomi_yaml["DWA"]["T"].as<double>();
  v_max                = coyomi_yaml["DWA"]["v_max"].as<double>();
  v_min                = coyomi_yaml["DWA"]["v_min"].as<double>();
  w_max                = coyomi_yaml["DWA"]["w_max"].as<double>() * M_PI/180.0;
  w_min                = coyomi_yaml["DWA"]["w_min"].as<double>() * M_PI/180.0;
  path_divided_step    = coyomi_yaml["DWA"]["path_divided_step"].as<double>();
  alpha                = coyomi_yaml["DWA"]["alpha"].as<double>();
  beta                 = coyomi_yaml["DWA"]["beta"].as<double>();
  gamma                = coyomi_yaml["DWA"]["gamma"].as<double>();
  obstacle_size        = coyomi_yaml["DWA"]["obstacle_size"].as<double>();

  deltaT = T / path_divided_step;

  // Viewer用の設定
  REAL_WIDTH  =20.0;
  REAL_HEIGHT =20.0;
  IMG_WIDTH  = 600;
  IMG_HEIGHT = 600;
  IMG_ORIGIN_X = IMG_WIDTH/2;
  IMG_ORIGIN_Y = IMG_HEIGHT/2;
  csize = REAL_WIDTH/IMG_WIDTH;

  baseImg = cv::Mat(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, cv::Scalar(182, 182, 182));
  cv::line(baseImg,
      cv::Point(0, IMG_ORIGIN_Y), cv::Point(IMG_WIDTH, IMG_ORIGIN_Y),
      cv::Scalar(200, 0, 0), 1);
  cv::line(baseImg,
      cv::Point(IMG_ORIGIN_X, 0), cv::Point(IMG_ORIGIN_X, IMG_HEIGHT),
      cv::Scalar(200, 0, 0), 1);
}

std::tuple<double, double> DynamicWindowApproach::run(
    const std::vector<LSP> &lsp,                  // LiDARスキャンデータ
    const Pose2d pose,                            // ロボットの姿勢（グローバル座標系）
    const double robot_v, const double robot_w,   // 現在のロボットの速度
    const WAYPOINT target                         // 現在のターゲット座標（グローバル座標系）
    ) {
  // urgの近傍点リストを作成 local coordinate
  nearby_lsp.clear();
  for (auto l: lsp) {
    if (l.r < limit_distance) {
      nearby_lsp.emplace_back(l);
    }
  }

  // 現在の速度を中心としてDynamicWindowを形成する
  std::vector<U> u_list;
  double DT = 0.1;
  for (double v = fmax(v_min, robot_v - acceleration_limit*DT); v < fmin(v_max, robot_v + acceleration_limit*DT); v += dv) {
    for (double w = fmax(w_min, robot_w - w_acceleration_limit*DT); w < fmin(w_max, robot_w + w_acceleration_limit*DT); w += dw) {
      u_list.emplace_back(v, w);
    }
  }
  // (u.v, u.w) の場合にT秒後の自分のグローバルな姿勢を計算する
  for (int i = 0; i < u_list.size(); i++) {
    double x_dash, y_dash, a_dash;
    double uv = u_list[i].v;
    double uw = u_list[i].w;
    // velocity
    u_list[i].velocity = uv;

    // heading
    if (fabs(uw) < 1e-6) {
      x_dash = pose.x + uv * T * cos(pose.a);
      y_dash = pose.y + uv * T * sin(pose.a);
      a_dash = pose.a;
    } else {
      a_dash = pose.a + uw * T;
      x_dash = pose.x - uv/uw * sin(pose.a) + uv/uw * sin(a_dash);
      y_dash = pose.y + uv/uw * cos(pose.a) - uv/uw * cos(a_dash);
    }
    double angle = atan2(target.y - y_dash, target.x - x_dash) - a_dash;
    if (angle < -M_PI) angle += 2*M_PI;
    if (angle >  M_PI) angle -= 2*M_PI;
    u_list[i].heading = M_PI - fabs(angle);

    // distance
    double min_dist = 9999999;
    double tmp_x = 0;
    double tmp_y = 0;
    double tmp_a = 0;
    for (double t = 0; t < T; t += deltaT) {
      if (fabs(uw) < 1e-6) {
        tmp_x += uv * deltaT;
      } else {
        tmp_x += -uv/uw * sin(tmp_a) + uv/uw * sin(tmp_a + uw * deltaT);
        tmp_y +=  uv/uw * cos(tmp_a) - uv/uw * cos(tmp_a + uw * deltaT);
        tmp_a +=  uw * deltaT;
      }
      // 最近傍点との距離を測る
      for (auto np: nearby_lsp) {
        double dist = std::hypot(np.x - tmp_x, np.y - tmp_y);
        if (min_dist > dist) {
          min_dist = dist;
          u_list[i].obx = np.x;
          u_list[i].oby = np.y;
        }
      }
    }
    u_list[i].distance = min_dist;
    u_list[i].obstacle_distance = min_dist;
  }

	// 評価値を規格化
  double HUGE_VALUE = 9999999.0;
	// headingを規格化
	double min_heading = HUGE_VALUE;
	double max_heading = -HUGE_VALUE;
  for (auto ev: u_list) {
		if (min_heading > ev.heading) {
			min_heading = ev.heading;
		} else if (max_heading < ev.heading) {
			max_heading = ev.heading;
		}
	}
	for (int i = 0; i < u_list.size(); i++) {
    if ((max_heading - min_heading) > 0.0) {
      u_list[i].heading = (u_list[i].heading - min_heading)/(max_heading - min_heading);
    } else {
      u_list[i].heading = 1.0;
    }
	}
	// velocityを規格化
	double min_vel = HUGE_VALUE;
	double max_vel = -HUGE_VALUE;
	for (auto ev: u_list) {
		if (min_vel > ev.velocity) {
			min_vel = ev.velocity;
		} else if (max_vel < ev.velocity) {
			max_vel = ev.velocity;
		}
	}
	for (int i = 0; i < u_list.size(); i++) {
		if ((max_vel - min_vel) > 0.0)
			u_list[i].velocity = (u_list[i].velocity - min_vel)/(max_vel - min_vel);
		else
      u_list[i].velocity = 1.0;
	}
	// distanceを規格化
	double min_d = HUGE_VALUE;
	double max_d = -HUGE_VALUE;
	for (auto ev: u_list) {
		if (min_d > ev.distance) {
			min_d = ev.distance;
		} else if (max_d < ev.distance) {
			max_d = ev.distance;
		}
	}
	for (int i = 0; i < u_list.size(); i++) {
		if ((max_d - min_d) > 0.0)
			u_list[i].distance = (u_list[i].distance - min_d)/(max_d - min_d);
		else
      u_list[i].distance = 1.0;
	}
  // bestを探索する
  double max_eval = -99999999;
  ng.clear();
  for (auto u: u_list) {
    if (u.obstacle_distance < obstacle_size) {   // 障害物とぶつかるなら棄却する
      ng.emplace_back(u);
      continue;
    }

    double eval = alpha * u.heading + beta * u.distance + gamma * u.velocity;
    if (max_eval < eval) {
      max_eval = eval;
      best_score.v = u.v;
      best_score.w = u.w;
    }
  }
  // パスが生成されていない場合は動ける場所が無いので，停止指令を返す
  if (u_list.size() - ng.size() == 0) {
    //std::cout << "Can't find path.\t";
		best_score.v -= acceleration_limit * DT;
		if (best_score.v < 0.0) best_score.v = 0.0;
		if (best_score.w > 0.0) {
      best_score.w -= w_acceleration_limit * DT;
      if (best_score.w < 0.0) best_score.w = 0;
    } else {
      best_score.w += w_acceleration_limit * DT;
      if (best_score.w > 0.0) best_score.w = 0;
    }
    //std::cout << "List size: " << u_list.size() << " NG path: " << ng.size() << std::endl;
    return std::make_tuple(best_score.v, best_score.w);
  }
  return std::make_tuple(best_score.v, best_score.w);
}

void DynamicWindowApproach::view(const Pose2d pose, const WAYPOINT target, const std::vector<LSP> &lsp) {
  cv::Mat img;
  baseImg.copyTo(img);
  double sx = 0;
  double sy = 0;
  double sa = 0;
  for (auto lp: lsp) {
    int lx = IMG_ORIGIN_X - lp.y/csize;
    int ly = IMG_ORIGIN_Y - lp.x/csize;
    if (lx >= 0 && lx < img.cols && ly >= 0 && img.rows) {
      cv::circle(img,
          cv::Point(lx, ly), 1, cv::Scalar(100, 100, 0), -1);
    }
  }
  // 近傍点
  for (auto np: nearby_lsp) {
    if (np.r < obstacle_size) {
      cv::circle(img,
          cv::Point(IMG_ORIGIN_X -np.y/csize, IMG_ORIGIN_Y - np.x/csize), 1,
          cv::Scalar(200, 0, 0), -1);
    } else {
      cv::circle(img,
          cv::Point(IMG_ORIGIN_X -np.y/csize, IMG_ORIGIN_Y - np.x/csize), 1,
          cv::Scalar(100, 100, 100), -1);
    }
  }

  // ロボットから見たtargetをプロット
  double dx = target.x - pose.x;
  double dy = target.y - pose.y;
  int tx = ( cos(pose.a) * dx + sin(pose.a) * dy)/csize;
  int ty = (-sin(pose.a) * dx + cos(pose.a) * dy)/csize;
  cv::circle(img, cv::Point(IMG_ORIGIN_X -ty, IMG_ORIGIN_Y - tx), 10, cv::Scalar(0, 0, 0), -1);

  // 棄却した指令値を描く
  for (auto ng_u: ng) {
#if 0
    sx = 0; sy = 0; sa = 0;
    if (fabs(ng_u.w) < 1e-6) {
      for (double t = 0; t < T; t += deltaT) {
        sx += ng_u.v * deltaT * cos(sa);
        sy += ng_u.v * deltaT * sin(sa);
        cv::circle(img,
            cv::Point(IMG_ORIGIN_X -sy/csize, IMG_ORIGIN_Y - sx/csize), 2,
            cv::Scalar(0, 0, 200), -1);
      }
    } else {
      for (double t = 0; t < T; t += deltaT) {
      sx += -ng_u.v/ng_u.w*sin(sa) + ng_u.v/ng_u.w*sin(sa + ng_u.w*deltaT);
      sy +=  ng_u.v/ng_u.w*cos(sa) - ng_u.v/ng_u.w*cos(sa + ng_u.w*deltaT);
      sa += ng_u.w * deltaT;
      cv::circle(img,
          cv::Point(IMG_ORIGIN_X -sy/csize, IMG_ORIGIN_Y - sx/csize), 2,
          cv::Scalar(0, 0, 200), -1);
      }
    }
#endif
    // 棄却したパスの最近某点
    cv::circle(img,
        cv::Point(IMG_ORIGIN_X -ng_u.oby/csize, IMG_ORIGIN_Y - ng_u.obx/csize), 1,
        cv::Scalar(0, 200, 0), -1);
  }
  // bestの円弧を描画する
  sx = 0; sy = 0; sa = 0;
  if (fabs(best_score.w) < 1e-6) {
    for (double t = 0; t < T; t += deltaT) {
      sx += best_score.v * deltaT * cos(sa);
      sy += best_score.v * deltaT * sin(sa);
      cv::circle(img,
          cv::Point(IMG_ORIGIN_X -sy/csize, IMG_ORIGIN_Y - sx/csize), 2,
          cv::Scalar(200, 200, 200), -1);
    }
  } else {
    for (double t = 0; t < T; t += deltaT) {
      sx += -best_score.v/best_score.w*sin(sa) + best_score.v/best_score.w*sin(sa + best_score.w*deltaT);
      sy +=  best_score.v/best_score.w*cos(sa) - best_score.v/best_score.w*cos(sa + best_score.w*deltaT);
      sa +=  best_score.w * deltaT;
      cv::circle(img,
          cv::Point(IMG_ORIGIN_X -sy/csize, IMG_ORIGIN_Y - sx/csize), 2,
          cv::Scalar(200, 200, 200), -1);
    }
  }

  cv::imshow("DWA", img);
}
