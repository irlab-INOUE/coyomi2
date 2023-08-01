#include "Navigation.h"

using namespace std::chrono;

int search_nearest_index(Pose2d pose, std::vector<WAYPOINT> wp_list) {
  // wp_listの最初から順に距離を測り，極値を超えたところで止める
  int min_index = 0;
  double prev_dist = 99999;  // 前回点までの距離
  double px = pose.x;
  double py = pose.y;
  for (int i = 0; i < wp_list.size(); i++) {
    double dist = std::hypot(px - wp_list[i].x, py - wp_list[i].y);   // 今回は単純な直線距離だが，本来は到達可能性を考慮し，移動距離を図るべき
    if (dist < prev_dist) {
      prev_dist = dist;
      min_index = i;
    } else {
      break;
    }
  }
  return min_index;
}

std::vector<WAYPOINT> wpRead(std::string wpname) {
  if (wpname == "") {
    std::cerr << "WPファイルのパスを指定してください\n";
    exit(0);
  }
  std::cerr << wpname << " を読み込みます...";
  std::vector<WAYPOINT> wp;
  std::fstream fn;
  fn.open(wpname);

  WAYPOINT buf;
  fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  while (!fn.eof()) {
    wp.emplace_back(buf);
    fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  }
  std::cerr << "完了\n";
  return wp;
}

Navigation::Navigation() {
  ;
}

Navigation::Navigation(YAML::Node &coyomi_yaml) {
  // WP情報を読み出す
  for (int i = 0; i < coyomi_yaml["MapPath"].size(); i++) {
    std::string path_root = std::string(DEFAULT_ROOT) + "/"
        + coyomi_yaml["MapPath"][i]["path"].as<std::string>();
    map_path.emplace_back(
        path_root,
        path_root + "/" + coyomi_yaml["MapPath"][i]["occupancy_grid_map"].as<std::string>(),
        path_root + "/" + coyomi_yaml["MapPath"][i]["map_RGBA"].as<std::string>(),
        path_root + "/" + coyomi_yaml["MapPath"][i]["way_point"].as<std::string>(),
        path_root + "/" + coyomi_yaml["MapPath"][i]["likelyhood_field"].as<std::string>(),
        path_root + "/" + coyomi_yaml["MapPath"][i]["mapInfo"].as<std::string>(),
        coyomi_yaml["MapPath"][i]["init_x"].as<double>(),
        coyomi_yaml["MapPath"][i]["init_y"].as<double>(),
        coyomi_yaml["MapPath"][i]["init_a"].as<double>());
  }
  std::cerr << "[" << __FILE__ << "] " << "Total number wp: " << map_path.size() << "\n";

  // DWAの情報を読み出す
  dwa = DynamicWindowApproach(coyomi_yaml);

  arrived_check_distance = coyomi_yaml["MotionControlParameter"]["arrived_check_distance"].as<double>();

  navi_state = TaskState::Failure;
}

void Navigation::start() {
  std::cerr << "[" << __FILE__ << "] " << "Get Navigation ready\n";
  // log用のファイルを開く
  std::string log_dir = "./";
  std::ofstream urg2d_log(log_dir + "urglog");
  std::ofstream imu_log(log_dir + "imu.log");
  std::ofstream enc_log(log_dir + "enclog");

  //std::ofstream mcl_log(log_dir + "mcl.log");
  //std::ofstream dwa_log(log_dir + "dwa.log");
  // オドメトリ値の受け皿
  Pose2d currentPose(0.0, 0.0, 0.0);
  Pose2d previousPose = currentPose;

  // 2D-LIDARに接続
  std::cerr << "Urg2d connecting...";
  Urg2d urg2d;
#ifdef URGDEBUG
  while(1) {
    std::vector<LSP> result = urg2d.getData();
    urg2d.view(5);
  }
#endif
  std::cerr << "done.\n";

  // ロボットの実際の動作状態の受け皿
  double robot_v = 0;
  double robot_w = 0;

  // パーティクル初期配置
  std::cerr << "MCL setup...";
  MCL mcl(Pose2d(0.0, 0.0, 0.0));
  //MCL mcl(currentPose);
  std::cerr << "done.\n";

  // Taperに接続
  std::cerr << "Taper setup...\n";
  Taper taper;
  std::cerr << "done.\n";

  //DecelMapの読み込み
  Decel decel;

//#define CHECK
#ifdef CHECK
  taper.set_ut(0.5, 0.0);   // (v, w)
  sleep(5);
  taper.set_ut(0.0, 0.0);   // (v, w)
  return;
#endif

//#define MEASUREMENT
#ifdef MEASUREMENT
  while(1) {
    // ログの保存
    auto time_now = high_resolution_clock::now(); 	// 現在日時を取得
    long long ts =
      duration_cast<milliseconds>(time_now.time_since_epoch()).count(); // エポックからの経過時間をmsecで取得
    ENC enc_data_log = taper.getData();
    enc_log
      << ts << " "
      << enc_data_log.x << " " << enc_data_log.y << " " << enc_data_log.a
      << " " << "end" << "\n";

    std::vector<LSP> result = urg2d.getData();
    urg2d_log << "LASERSCANRT" << " "
      << ts << " "
      << result.size() * 3 << " "
      << "-100" << " " << "100" << " "
      << "0.25" << " " << "3" << " ";
    for (auto d: result) {
      urg2d_log << d.data << " " << "0" << " " << "0" << " ";
    }
    urg2d_log << ts << "\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << enc_data_log.x << " " << enc_data_log.y << " " << enc_data_log.a << "\n";
    std::cout << std::defaultfloat << "\33[1A";
  }
  std::cerr << "Bye.\n";
  return;
#endif


  int cnt = 0;
  WAYPOINT GOAL;  // ナビゲーションの最後のゴール到達処理用
  double total_travel = 0;  // 走行距離
  // 全てのWPファイルをナビゲーションさせる
  for (int i = 0; i < map_path.size(); i++) {
    taper.set_ut(0.0, 0.0);
    //cv::waitKey(0);
    std::cerr << "[" << __FILE__ << "] " << "WP " << i << "\n";
    Viewer view(map_path[i]);                                        // 現在のoccMapを表示する
    std::vector<WAYPOINT> wp_list = wpRead(map_path[i].way_point);   // map_path[i].way_pointのWPをリストへ格納する
    view.plot_wp(wp_list);                                           // wp_listを地図上に描画する
    view.hold();                                                     // viewを固定する(reset()でこの描画内容に戻せる)
    Pose2d init_pose(map_path[i].init_x, map_path[i].init_y, map_path[i].init_a);   // MCLの初期姿勢をcoyomi.yamlから読み出す
    mcl.set_currentPose(init_pose);
    mcl.set_lfm(map_path[i].likelyhood_field);
    mcl.set_mapInfo(map_path[i].mapInfo);

    GOAL = wp_list[wp_list.size()-1];   // 最後のゴール処理用に保持

    // 現在地から一番近いWPを最初のWPターゲットにセットする
    // そのWPを起点とし，最後のWPまで次々とナビする
    int index = search_nearest_index(mcl.get_best_pose(), wp_list);

    // 現在位置からwp_list[j]に向けたナビゲーションを最後まで繰り返す
    for (int j = index; j < wp_list.size(); j++) {
      // WPFでwp_list[j-1](最初のみcurrentPose)からwp_list[j]までの経路を生成
      std::vector<WAYPOINT> sub_goal;
      if (j == 0) {
        sub_goal = WaveFrontPlanner(mcl.get_best_pose(), wp_list[j], map_path[i], view.get_csize(), view.get_originX(), view.get_originY());
      } else {
        Pose2d wp0(wp_list[j-1].x, wp_list[j-1].y, 0);
        sub_goal = WaveFrontPlanner(wp0, wp_list[j], map_path[i], view.get_csize(), view.get_originX(), view.get_originY());
      }
      //sub_goal[0] = mcl.get_best_pose(); // サブゴールの始点は現在の推定値とする
      sub_goal[sub_goal.size()-1] = wp_list[j];   // サブゴールの終点はwp_list[j]に合わせる
      //view.plot_subGoal(sub_goal);                                           // sub_goalを地図上に描画する

      // ElasticBandsでパスを滑らかにする
      ElasticBands elb(map_path[i], view.get_csize(), view.get_originX(), view.get_originY());
      std::vector<WAYPOINT> smooth_path = elb.optimize(sub_goal);
      smooth_path[smooth_path.size()-1] = wp_list[j];
      smooth_path.emplace_back(wp_list[j]);

      view.plot_smoothPath(smooth_path);                                       // smooth_pathを地図上に描画する
                                                                               // WPFに沿ってwp_list[j]までナビ
      int target_index = 1;
      int look_ahead = 5;
      //int look_ahead = 10;
      int shift = look_ahead;    // 通常はlook_aheadと等しいが，最後のターゲットを超過する場合は0とする

      while(1) {
        // LiDAR計測値を得る
        std::vector<LSP> lsp = urg2d.getData();
        //std::vector<LSP> lsp = urg2d.getData(view.get_imgMap_original(), currentPose,
        //    view.get_originX(), view.get_originY(), view.get_csize());  // 仮想URG

        // MCL(KLD_sampling)
        // 動いてなければ自己位置推定はしない
        currentPose = taper.get_pose();
        double _rot = currentPose.a - previousPose.a;
        double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
        if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
          ;
        } else {
          mcl.KLD_sampling(lsp, currentPose, previousPose);
          total_travel += _tran;
        }
        Pose2d estimatedPose = mcl.get_best_pose();
        std::tie(robot_v, robot_w) = taper.get_ut();

        // DWAで制御指令値を算出する
        if (target_index + shift >= smooth_path.size()) {
          shift -= 1;
          if (shift < 0) shift = 0;
        } else {
          shift = look_ahead;
        }
        double v, w;
        std::tie(v, w) = dwa.run(lsp, estimatedPose, robot_v, robot_w, smooth_path[target_index + shift]);

        //減速率を付与
        double v_acc, w_acc;
        float d = decel.get_rate(estimatedPose);
        v_acc = d * v;
        w_acc = d * w;

        taper.set_ut(v_acc, w_acc);
        //taper.set_ut(v, w);

        if (cnt % 20 == 0) {
          dwa.view(estimatedPose, smooth_path[target_index + shift], lsp);
        }

        // 推定結果の表示
        //std::vector<WAYPOINT> test_smooth_path = elb.optimize(sub_goal);  // 未実装：最新の障害物情報を渡す
        if (cnt % 200 == 0) {
          view.reset();
          //0view.clear();
          view.robot(mcl.get_best_pose());
          view.particle(mcl.get_particle_set());
          view.urg(mcl.get_best_pose(), lsp);
          view.plot_smoothPath(smooth_path);                                       // smooth_pathを地図上に描画する
          view.show(5);
        }
        // 最後のターゲットに到着していたらこの区間は終了
        if (target_index >= smooth_path.size()) break;

        std::vector<Pose2d> pset = mcl.get_particle_set();
        std::cout << std::fixed << std::setprecision(2)
          << "\33[2K==============================\n"
          << "\33[2K" << "x:" << estimatedPose.x << " y:" << estimatedPose.y << " a:" << estimatedPose.a*180/M_PI << "[deg]\n"
          << "\33[2K" << "v:" << robot_v << "[m/s] w:" << robot_w*180/M_PI << "[deg/s]\n"
          << "\33[2K-----\n"
          << "\33[2K" << "d:" << d * 100 << "[%]\n"
	  << "\33[2K" << "V*:" << v * d << "[m/s]\n"
          << "\33[2K-----\n"
          << "\33[2K" << "Control v:" << v << "[m/s] w:" << w*180/M_PI << "[deg/s]\n"
          << "\33[2K-----\n"
          << "\33[2K" << "Target Index:" << target_index + shift << "/" << smooth_path.size() << "\t"
          << "Dist:" << std::hypot(estimatedPose.x - smooth_path[target_index + shift].x,
              estimatedPose.y - smooth_path[target_index + shift].y) << "[m]\n"
          << "\33[2K" << "Particle num:" << pset.size() << "\n"
          << "\33[2K" << "Travel: " << total_travel << "\n";
        std::cout << std::defaultfloat << "\33[12A";

        // ログの保存
        auto time_now = high_resolution_clock::now(); 	// 現在日時を取得
        long long ts =
          duration_cast<milliseconds>(time_now.time_since_epoch()).count(); // エポックからの経過時間をmsecで取得
        ENC enc_data = taper.getData();
        imu_log << ts << " "
          << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.a << " "
          << enc_data.ax << " " << enc_data.ay << " " << enc_data.az << " "
          << enc_data.wx << " " << enc_data.wy << " " << enc_data.wz << " "
          << enc_data.mx << " " << enc_data.my << " " << enc_data.mz << " "
          << robot_v << " " << robot_w << "\n";

        // 現在位置がターゲットに近づいたらターゲットを一つ先に移動させる
        if (std::hypot(estimatedPose.x - smooth_path[target_index + shift].x, estimatedPose.y - smooth_path[target_index + shift].y)
            < arrived_check_distance) {
          target_index += 1;
        }

        // 次のループの準備
        previousPose = currentPose;
        cnt++;
        //usleep(1000);
      }
      if (wp_list[j].stop_check == 1) {
        std::cout << j << ": 一時停止しています。画像にフォーカスしている状態で[a]キーを押してください\n";
        taper.set_ut(0.0, 0.0);
        while (cv::waitKey(5) != 'a') {
          taper.set_ut(0.0, 0.2);
          usleep(500000);
          taper.set_ut(0.0, -0.2);
          usleep(500000);
        }
        taper.set_ut(0.0, 0.0);
        previousPose = taper.get_pose();
      }
      view.reset();
    }
  }
  navi_state = TaskState::Success;
#if 1
  // GOALへのアプローチ
  cnt = 0;
  Viewer view(map_path[map_path.size() - 1]);                                        // 現在のoccMapを表示する
  view.hold();                                                     // viewを固定する(reset()でこの描画内容に戻せる)
  std::cout << std::defaultfloat << "\33[9B";
  std::cout << "Approaching to goal...\n";
  while (1) {
    // LiDAR計測値を得る
    std::vector<LSP> lsp = urg2d.getData();
    // MCL(KLD_sampling)
    // 動いてなければ自己位置推定はしない
    currentPose = taper.get_pose();
    double _rot = currentPose.a - previousPose.a;
    double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
    if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
      continue;
    } else {
      total_travel += _tran;
      mcl.KLD_sampling(lsp, currentPose, previousPose);
      Pose2d estimatedPose = mcl.get_best_pose();
      std::cout << std::fixed << std::setprecision(2)
          << "\33[2K==============================\n"
          << "\33[2K" << "x:" << estimatedPose.x << " y:" << estimatedPose.y << " a:" << estimatedPose.a*180/M_PI << "[deg]\n"
          << "\33[2K" << "Travel: " << total_travel << "\n";
        std::cout << std::defaultfloat << "\33[3A";
      if (std::hypot(estimatedPose.x - GOAL.x, estimatedPose.y - GOAL.y) < 0.3) {   // 最終ゴール到達判定
        break;
      }
      double v, w;
      std::tie(robot_v, robot_w) = taper.get_ut();
      std::tie(v, w) = dwa.run(lsp, estimatedPose, robot_v, robot_w, GOAL);

      //減速率を付与
      double v_acc, w_acc;
      float d = decel.get_rate(estimatedPose);
      v_acc = d * v;
      w_acc = d * w;

      taper.set_ut(v_acc, w_acc);
      //taper.set_ut(v, w);
      previousPose = currentPose;
    }
    // 推定結果の表示
    if (cnt % 10 == 0) {
      dwa.view(mcl.get_best_pose(), GOAL, lsp);
      view.clear();
      view.robot(mcl.get_best_pose());
      view.particle(mcl.get_particle_set());
      view.urg(mcl.get_best_pose(), lsp);
      view.show(5);
    }
    cnt++;
    usleep(1000);
  }
#endif
  taper.set_ut(0, 0);   // (v, w)
  std::cout << std::defaultfloat << "\33[3B";
  std::cout << std::fixed << std::setprecision(2)
    << "\33[2K" << "Travel: " << total_travel << "\n";
  std::cout << std::defaultfloat;

  std::cerr << "[" << __FILE__ << "] " << "Navigation finish\n";
}

TaskState Navigation::state() {
  return navi_state;
}
