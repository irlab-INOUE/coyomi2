#include "shared_struct.h"
#include "global_variable.h"

void thread_localization(std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data,
                         std::shared_ptr<LOC> loc,
                         std::shared_ptr<ENC> enc,
                         std::shared_ptr<URG2D> urg2d,
                         std::shared_ptr<WP_LIST> wp_list) {
  // coyomi_yamlをこのスレッド内で新しく取得する
  std::string path_to_yaml = DEFAULT_ROOT + std::string("/coyomi.yaml");
  YAML::Node coyomi_yaml = yamlRead(path_to_yaml);
  add_log(log_data, "coyomi.yaml is open in thread_localization.");

  add_log(log_data, "START LOCALIZATION SETUP");
  std::string MAP_PATH = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
  // Map file path
  std::string MAP_NAME
    = MAP_PATH+ "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
  MAP_NAME.copy(loc->path_to_map_dir, MAP_NAME.size());
  // Likelyhood file path
  std::string LIKELYHOOD_FIELD
    = MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["likelyhood_field"].as<std::string>();
  LIKELYHOOD_FIELD.copy(loc->path_to_likelyhood_field, LIKELYHOOD_FIELD.size());
  add_log(log_data, "DONE LOCALIZATION SETUP");
  // Initial pose
  if (loc->CURRENT_MAP_PATH_INDEX != 0) {
    double initial_pose_x = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_x"].as<double>();
    double initial_pose_y = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_y"].as<double>();
    double initial_pose_a = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_a"].as<double>() * M_PI/180;
    loc->x = initial_pose_x; loc->y = initial_pose_y; loc->a = initial_pose_a;
  }
  Pose2d currentPose = Pose2d(enc->x, enc->y, enc->a);
  Pose2d previousPose = currentPose;

  MapPath map_path(MAP_PATH, loc->path_to_map_dir, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);
  Viewer view(map_path);                        // 現在のoccMapを表示する
  view.hold();
  view.show(loc->x, loc->y, 5);
  //cv::moveWindow("occMap", 700, 0);
  loc->change_map_trigger = ChangeMapTrigger::kContinue;
  std::vector<WAYPOINT> wp;
  while (!wp_list->get_ready) {
    sleep_for(seconds(1));
  }
  for (int i = 0; i < wp_list->size_wp_list; i++) {
    wp.emplace_back(wp_list->wp_list[i].x, wp_list->wp_list[i].y, wp_list->wp_list[i].a, wp_list->wp_list[i].stop_check);
  }

#if 1
  // DE with LFM
  const double Window_xy = 0.2;            // 探索範囲[m]
  const double Window_a  = 10*M_PI/180;    // 角度[rad]
  const double population = 20;
  const double generates = 10;
  const double F = 0.5;
  const double CR = 0.1;

  add_log(log_data, "START DE setup");
  DELFM de(Window_xy, Window_a, population, generates, F, CR);
  add_log(log_data, "START DE lfm");
  de.set_lfm(loc->path_to_likelyhood_field);
  add_log(log_data, "START DE mapinfo");
  de.set_mapInfo(MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>());

  std::string de_logfile_path = log_path->path + "/delog";

  add_log(log_data, "START LOCALIZATION LOOP");
  while (running.load()) {
    if (loc->change_map_trigger == ChangeMapTrigger::kChange) break;
    view.plot_wp(wp);
    view.plot_current_wp(wp[enc->current_wp_index]);
    view.show(loc->x, loc->y, 5);
    // 動いてなければ自己位置推定はしない
    currentPose = Pose2d(enc->x, enc->y, enc->a);
    double _rot = currentPose.a - previousPose.a;
    double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
    std::vector<LSP> lsp;
    double best_x, best_y, best_a, best_eval;
    if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
      best_x = loc->x;
      best_y = loc->y;
      best_a = loc->a;
      best_eval = -1;
    } else {
      for (int k = 0; k < urg2d->size; k++) {
        lsp.emplace_back(urg2d->r[k], urg2d->r[k]/1000.0, urg2d->ang[k], urg2d->cs[k], urg2d->sn[k]);
      }
      // DEwithLFM で自己位置推定
      std::tie(best_x, best_y, best_a, best_eval) = de.optimize_de(lsp, loc->x, loc->y, loc->a);
    }
    Pose2d estimatedPose = Pose2d(best_x, best_y, best_a);

    view.reset();
    view.robot(estimatedPose);
    view.urg(estimatedPose, lsp);

    loc->x = estimatedPose.x;
    loc->y = estimatedPose.y;
    loc->a = estimatedPose.a;

    de_log.open(de_logfile_path, std::ios_base::app);
    long long ts = get_current_time();
    de_log
      << ts << " "
      << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.a << " "
      << best_eval << " "
      << "end" << "\n";
    de_log.close();

    // 次のループの準備
    previousPose = currentPose;
    usleep(10000);
  }
#endif
#if 0
  // パーティクル初期配置
  MCL mcl(Pose2d(loc->x, loc->y, loc->a));
  mcl.set_lfm(loc->path_to_likelyhood_field);
  mcl.set_mapInfo(MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>());
  // MCL(KLD_sampling)
  while(1) {
    if (loc->change_map_trigger == ChangeMapTrigger::kChange) break;
    view.plot_wp(wp);
    view.plot_current_wp(wp[enc->current_wp_index]);
    view.show(loc->x, loc->y, 5);
    // 動いてなければ自己位置推定はしない
    currentPose = Pose2d(enc->x, enc->y, enc->a);
    double _rot = currentPose.a - previousPose.a;
    double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
    std::vector<LSP> lsp;
    if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
      ;
    } else {
      for (int k = 0; k < urg2d->size; k++) {
        lsp.emplace_back(urg2d->r[k], urg2d->r[k]/1000.0, urg2d->ang[k], urg2d->cs[k], urg2d->sn[k]);
      }
      mcl.KLD_sampling(lsp, currentPose, previousPose);
    }
    double best_eval = 0;
    Pose2d estimatedPose;
    std::tie(estimatedPose, best_eval) = mcl.get_best_pose();
    std::vector<Pose2d> particle = mcl.get_particle_set();

    view.reset();
    view.robot(estimatedPose);
    view.urg(estimatedPose, lsp);
    view.particle(particle);

    loc->x = estimatedPose.x;
    loc->y = estimatedPose.y;
    loc->a = estimatedPose.a;

    std::string mcl_logfile_path = log_path->path + "/mcllog";
    //std::string path = log_path + "/mcllog";
    mcl_log.open(mcl_logfile_path, std::ios_base::app);
    long long ts = get_current_time();
    mcl_log
      << ts << " "
      << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.a << " "
      << best_eval << " " << particle.size() << " "
      << "end" << "\n";
    mcl_log.close();

    // 次のループの準備
    previousPose = currentPose;
    usleep(30000);
  }
#endif
  std::cout << "localization exit." << std::endl;
}

