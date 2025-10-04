/**************************************************************************
 * Robot control program using DE with LFM
 *
 * author: Kazumichi INOUE <kazumichiinoue@mail.saitama-u.ac.jp>
 * Github: https://github.com/irlab-INOUE/coyomi2
 * date:   2023/6/20
 * update: 2025/5/23
 ****************************************************************************/
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <SDL.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <ncurses.h>
#include <atomic>
#include <mutex>
#include <thread>

using namespace std::chrono;  // seconds, milliseconds
using std::this_thread::sleep_for;
using LockGuard = std::lock_guard<std::mutex>;
std::mutex mtx;

#include "common.h"
#include "shm_board.h"
#include "Viewer.h"
#include "yaml-cpp/yaml.h"
#include "DWA.h"
#include "checkDirectory.h"
#include "Config.h"
#include "wave_front_planner.h"
#include "time_utility.h"
#include "DELFM.h"
#include "MCL.h"
#include "GetUrg3d.h"
#include "shared_struct.h"
#include "thread_battery_logger.h"
#include "thread_sound.h"
#include "thread_3D_Lidar.h"
#include "thread_display.h"
#include "thread_2D_Lidar_b.h"
#include "thread_localization.h"
#include "global_variable.h"

/***************************************
 * Global variable
 ****************************************/
std::thread th_battery_logger;
std::thread th_sound_logger;
std::thread th_3D_Lidar;
std::thread th_display;
std::thread th_2D_Lidar_b;
std::thread th_2D_Lidar_t;
std::thread th_localization;

// ジョイスティック
SDL_Joystick* joystick;

// 共有オブジェクト
auto log_path = std::make_shared<LOGDIR_PATH>();
auto log_data = std::make_shared<LOG_DATA>();
auto disp     = std::make_shared<DisplayContents>();
auto bat      = std::make_shared<BAT>();
auto loc      = std::make_shared<LOC>();
auto enc      = std::make_shared<ENC>();
auto urg2d    = std::make_shared<URG2D>();
auto urg2d_t  = std::make_shared<URG2D>();
auto wp_list  = std::make_shared<WP_LIST>();

// Oriental Motors
int fd_motor;   // FDをOrientalMotorInterface.hで使うのでinclude前に定義
#include "OrientalMotorInterface.h"
//#define DEBUG_SENDRESP

void sigcatch(int);

// The max (min) value for each axis based measurement value
struct joy_calib {
  int max;
  int min;
  int zero;

  joy_calib() {
    max = 32767;
    min = -32768;
    zero = 0;
  };

  void set_val(int val) {
    if (max < val) {
      max = val;
    }

    if (min > val) {
      min = val;
    }
  };

  void set_zero(int val) {
    zero = val;
  };
};

void read_joystick(double &v, double &w, const std::vector<joy_calib> &j_calib) {
  const int DEAD_ZONE = 8000;
  double axis0 = 0;
  double axis1 = 0;

  auto set_vw = [&v, &w](double _v, double _w) {v = _v; w = _w;};

  /* read the joystick state */
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_JOYBUTTONUP:
      case 1:
      case 2:
      case 0:
      case 3:
      default:
        set_vw(0.0, 0.0);
        break;
      case SDL_JOYBUTTONDOWN:
        switch (static_cast<int>(e.jbutton.button)) {
          case 1:
            set_vw(0.5, 0.0);
            break;
          case 2:
            set_vw(-0.5, 0.0);
            break;
          case 0:
            set_vw(0.0, 1.0);
            break;
          case 3:
            set_vw(0.0, -1.0);
            break;
          case 4:
          case 5:
            get3DLidarData.store(true);
            break;
          case 10:
            gotoEnd.store(true);
            //std::cerr << "End\n";
            set_vw(0.0, 0.0);
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            sleep_for(milliseconds(150));
            break;
          case 12:
            //std::cerr << "FREE\n";
            set_vw(0.0, 0.0);
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            sleep_for(milliseconds(100));
            isFREE.store(!(isFREE.load()));
            if (isFREE.load())
              free_motors();
            else
              turn_on_motors();
            break;
          default:
            set_vw(0.0, 0.0);
            break;
        }
    }
    if (std::abs(e.jaxis.value) > DEAD_ZONE) {
      int axis_index = static_cast<int>(e.jaxis.axis);
      switch (axis_index) {
        case 0:
          axis0 =2.0 * e.jaxis.value / (j_calib[0].max - j_calib[0].min + 10);
          break;
        case 1:
          axis1 =2.0 * e.jaxis.value / (j_calib[1].max - j_calib[1].min + 10);
          break;
        default:
          break;
      }
    }
  }
}

std::vector<WAYPOINT> wpRead(std::string wpname) {
  if (wpname == "") {
    std::cerr << "WPファイルのパスを指定してください\n";
    exit(EXIT_FAILURE);
  }
  std::vector<WAYPOINT> wp;
  std::fstream fn;
  fn.open(wpname);

  WAYPOINT buf;
  fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  while (!fn.eof()) {
    wp.emplace_back(buf);
    fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  }
  return wp;
}

void WaypointEditor(std::string MAP_PATH, std::string WP_NAME, std::string OCC_NAME) {
  // Reading Way Point
  std::vector<WAYPOINT> wp;
  wp = wpRead(MAP_PATH + "/" + WP_NAME);
  int wp_index = 0;
  std::string map_name = MAP_PATH + "/" + OCC_NAME;
  MapPath map_path(MAP_PATH, map_name, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);
  Viewer view(map_path);                        // 現在のoccMapを表示する
  view.hold();
  view.show(0, 0, 5);
  cv::moveWindow("occMap", 400, 0);
  while (1) {
    view.reset();
    view.plot_wp(wp);
    view.plot_current_wp(wp[wp_index]);
    view.show(wp[wp_index].x, wp[wp_index].y, 5);
    std::cout << "INDEX:" << wp_index << " " << wp[wp_index].x << " " << wp[wp_index].y << "\n";
    int key = cv::waitKey();
    if (key == 'q') return;
    if (key == 'n') {
      wp_index++;
    } else if (key == 'p') {
      wp_index -= 1;
    } else if (key == 'r') {
      wp.clear();
      wp = wpRead(std::string(MAP_PATH) + "/" + "wp.txt");
      wp_index = 0;
    }
    if (wp_index >= static_cast<int>(wp.size())) {
      wp_index = 0;
    } else if (wp_index < 0) {
      wp_index = static_cast<int>(wp.size()) - 1;
    }
    sleep_for(milliseconds(5));
  }
}

/***************************************
 * MAIN
 ***************************************/
int main(int argc, char *argv[]) {
  /* Ctrl+c 対応 */
  if (SIG_ERR == signal( SIGINT, sigcatch )) {
    std::printf("failed to set signal handler\n");
    exit(EXIT_FAILURE);
  }

  /* Configその他の読み込みセクション */
  // coyomi.yamlに接続する
  std::string path_to_yaml = DEFAULT_ROOT + std::string("/coyomi.yaml");
  YAML::Node coyomi_yaml = yamlRead(path_to_yaml);
  std::cerr << "coyomi.yaml is open.\n";

  // Mode selector
  std::cerr << "Hello, Coyomi2" << "\n";
  std::cout << "Please select an action in following list.\n"
    << "[1]: Only localization (default)\n"
    << " 2 : Navigation\n"
    << " 3 : WayPoint editor\n"
    << " 4 : exit" << std::endl;
  char MODE;
  while (1) {
    MODE = getchar();
    if (MODE == '1' || MODE == '\n') {
      MODE = '1';
      std::cout << "Hello, Coyomi2 localization.\n";
      break;
    }
    else if (MODE == '2') {
      std::cout << "Hello, Coyomi2 Navigation.\n";
      break;
    }
    else if (MODE == '3') {
      std::cout << "Hello, Coyomi2 Waypoint editor.\n";
      int CURRENT_MAP_PATH_INDEX;

      std::cout << "Input CURRENT_MAP_PATH_INDEX number >> ";
      std::cin >> CURRENT_MAP_PATH_INDEX;

      std::string MAP_PATH = coyomi_yaml["MapPath"][CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
      std::string WP_NAME  = coyomi_yaml["MapPath"][CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>();
      std::string OCC_NAME = coyomi_yaml["MapPath"][CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();

      WaypointEditor(MAP_PATH, WP_NAME, OCC_NAME);

      return 0;
    }
    else if (MODE == '4') {
      std::cout << "Hello, Coyomi2. Exit." << std::endl;
      return 0;
    }
  }

  /***************************************************************************
   * LOG保管場所を作成する
   * DEFAULT_LOG_DIRの場所にcoyomi_log ディレクトリがあるかチェックし，
   * なければ作成する
   ***************************************************************************/
  // 現在日付時刻のディレクトリを作成する
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&now_c);

  checkDir(std::string(DEFAULT_LOG_DIR));
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y"); log_path->year = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%m"); log_path->mon  = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%d"); log_path->mday = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%H"); log_path->hour = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%M"); log_path->min  = oss.str(); oss.str(""); oss.clear();
  oss << std::put_time(&tm, "%S"); log_path->sec  = oss.str(); oss.str(""); oss.clear();
  log_path->path = std::string(DEFAULT_LOG_DIR) + "/" + log_path->year + "/" + log_path->mon + "/"
    + log_path->mday + "/" + log_path->hour + log_path->min + log_path->sec;
  checkDir(log_path->path);

  std::cerr << "path: " << log_path->path << "にログを保存します" << std::endl;

  th_battery_logger = std::thread(thread_battery_logger, log_path, log_data, bat);
  th_sound_logger   = std::thread(thread_sound, log_path, log_data, enc);
  th_3D_Lidar       = std::thread(thread_3D_Lidar, log_path, log_data);
  th_display        = std::thread(thread_display, log_path, log_data, disp, enc, loc, bat);
  th_2D_Lidar_b     = std::thread(thread_2D_Lidar_b, std::string("b"), log_path, log_data, urg2d);
  th_2D_Lidar_t     = std::thread(thread_2D_Lidar_b, std::string("t"), log_path, log_data, urg2d_t);
  th_localization   = std::thread(thread_localization, log_path, log_data, loc, enc, urg2d, wp_list);

  /**************************************************************************
   * Connect check & open serial port for MotoDriver
   ***************************************************************************/
  if((fd_motor = open(SERIAL_PORT_MOTOR, O_RDWR | O_NOCTTY)) == -1) {
    add_log(log_data, "Can't open serial port");
    gotoEnd.store(true);
  } else {
    std::string log_text = "Get fd_motor: " + std::to_string(fd_motor);
    add_log(log_data, log_text);
  }

  /**************************************************************************
   * Joystick setup
   ***************************************************************************/
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0) {
    std::string log_text = "Failure SDL initialize. " + std::string(SDL_GetError());
    add_log(log_data, log_text);
    gotoEnd.store(true);
  }
  joystick = SDL_JoystickOpen(0);

  // calibrate axis until JS_EVENT_BUTTON pressed
  std::vector<joy_calib> j_calib(6);
  SDL_Event e;
  add_log(log_data, "Joypad ready completed.");

  /**************************************************************************
   * Serial port setup for Motor Drivers
   ***************************************************************************/
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd_motor, TCSANOW, &tio);

  /**************************************************************************
   * Motor driver setup
   ***************************************************************************/
  // BLV-R Driver setup
  // ID Share Config.
  add_log(log_data, "ID Share configration...");
  simple_send_cmd(Query_IDshare_R, sizeof(Query_IDshare_R));
  simple_send_cmd(Query_IDshare_L, sizeof(Query_IDshare_L));
  simple_send_cmd(Query_READ_R,    sizeof(Query_READ_R));
  simple_send_cmd(Query_READ_L,    sizeof(Query_READ_L));
  simple_send_cmd(Query_WRITE_R,   sizeof(Query_WRITE_R));
  simple_send_cmd(Query_WRITE_L,   sizeof(Query_WRITE_L));
  add_log(log_data, "Done");

  //trun on exitation on RL motor
  turn_on_motors();

  /**************************************************************************
   * Waypoint setup
   * Note:
   * For the wavefront planner to work properly, the occMap should have
   * traversable areas marked in white.
   ***************************************************************************/
  loc->CURRENT_MAP_PATH_INDEX = 0;
  if (argc > 1) loc->CURRENT_MAP_PATH_INDEX = std::atoi(argv[1]);
  std::string MAP_PATH = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
  // Reading Way Point
  std::vector<WAYPOINT> tmp_wp, wp;
  tmp_wp = wpRead(MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>());
  WAYPOINT prev_target(0, 0, 0, 0);
  wavefrontplanner::Config cfg;
  cfg.map_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
  cfg.map_info_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>();
  wavefrontplanner::WaveFrontPlanner wfp(cfg);
  for (auto w: tmp_wp) {
    wavefrontplanner::Config cfg;
    cfg.start_x_m = prev_target.x;
    cfg.start_y_m = prev_target.y;
    cfg.goal_x_m  = w.x;
    cfg.goal_y_m  = w.y;
    wfp.Init(cfg);
    std::vector<wavefrontplanner::Path> path = wfp.SearchGoal();
    for (auto p: path) {
      WAYPOINT w(p.x, p.y, p.r, 0);
      wp.emplace_back(w);
    }
    wp.emplace_back(w);
    wp.back().stop_check = w.stop_check;
    prev_target.x = w.x;
    prev_target.y = w.y;
  }

  wp_list->size_wp_list = wp.size();
  for (int i = 0; i < wp.size(); i++) {
    wp_list->wp_list[i].x = wp[i].x;
    wp_list->wp_list[i].y = wp[i].y;
    wp_list->wp_list[i].a = wp[i].a;
    wp_list->wp_list[i].stop_check = wp[i].stop_check;
  }
  wp_list->get_ready = true;
  enc->current_wp_index = 0;

  /**************************************************************************
   * initial pose setup
   ***************************************************************************/
  loc->x = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_x"].as<double>();
  loc->y = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_y"].as<double>();
  loc->a = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["init_a"].as<double>() * M_PI/180;

  /**************************************************************************
   * initial enc setup
   ***************************************************************************/
  long long first_ts = get_current_time();
  ODOMETORY first_odo;
  read_state(first_odo, first_ts);
  sleep(1);
  enc->ts = first_ts;
  enc->x  = first_odo.rx;
  enc->y  = first_odo.ry;
  enc->a  = first_odo.ra;

  /**************************************************************************
   * Starting Main Process
   ***************************************************************************/
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  tcflush(fd_motor, TCIOFLUSH);
  DynamicWindowApproach dwa(coyomi_yaml);
  double arrived_check_distance = coyomi_yaml["MotionControlParameter"]["arrived_check_distance"].as<double>();

  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  usleep(100000);
  free_motors();
  std::string start_bell_cmd = "paplay /usr/share/sounds/freedesktop/stereo/bell.oga";
  int start_bell_ret = std::system(start_bell_cmd.c_str());

  while (isFREE.load()) {
    if (gotoEnd.load()) break;
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);

    long long ts = get_current_time();
    read_state(odo, ts);
    enc->ts = ts;
    enc->x = odo.rx;
    enc->y = odo.ry;
    enc->a = odo.ra;

    sleep_for(milliseconds(100));
  }
  start_bell_ret = std::system(start_bell_cmd.c_str());

  while(!gotoEnd.load()) {
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);

    std::vector<LSP> lsp;
    for (int k = 0; k < urg2d->size; k++) {
      lsp.emplace_back(urg2d->r[k], urg2d->r[k]/1000.0, urg2d->ang[k], urg2d->cs[k], urg2d->sn[k]);
    }
    Pose2d estimatedPose(loc->x, loc->y, loc->a);
    if (MODE == '1') {
      v = tmp_v; w = tmp_w;
      //isFREE = true;
    }
    if (MODE == '2') {
      double obx, oby;
      std::tie(v, w, obx, oby) = dwa.run(lsp, estimatedPose, v, w, wp[enc->current_wp_index]);
      disp->min_obstacle_x = obx;
      disp->min_obstacle_y = oby;

      // rotate ricovery
      if (fabs(v) < 1e-6 && fabs(w) <1e-6) {
        if (oby >= 0) w = -M_PI/8.0;
        else w = M_PI/8.0;
        v = 0.3 * w;  // rodate radius is 0.3[m]
        if (v > 0.0) v = -v;
      }

      // WP上の障害物判定を入れる
      // 必要ならばWFPを再実行（近距離だけ）

      // 通常処理
      double dist2wp = std::hypot(wp[enc->current_wp_index].x - estimatedPose.x,
          wp[enc->current_wp_index].y - estimatedPose.y);
      if (wp[enc->current_wp_index].stop_check == 2) {
        if (dist2wp < arrived_check_distance) {
          double dwa_v = v;
          v = v * 0.8;
          if (v < 0.1) v = 0.1;
        }
        if (dist2wp < 0.5) {
          enc->current_wp_index += 1;
        }
      } else if (wp[enc->current_wp_index].stop_check == 1) {
        if (dist2wp < 0.5) {
          enc->current_wp_index += 1;
          isFREE.store(true);
          free_motors();
          while (isFREE.load() || !gotoEnd.load()) {
            read_joystick(v, w, j_calib);
            long long ts = get_current_time();
            read_state(odo, ts);
            enc->ts = ts;
            enc->x = odo.rx;
            enc->y = odo.ry;
            enc->a = odo.ra;
          }
        }
      } else if (dist2wp < arrived_check_distance) {
        enc->current_wp_index += 1;
      }
      if (enc->current_wp_index >= wp.size()) {
        enc->current_wp_index = 0;
        v = 0; w = 0;
        calc_vw2hex(Query_NET_ID_WRITE, v, w);
        simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
        sleep(1);

        // Change current map
        loc->CURRENT_MAP_PATH_INDEX++;
        if (loc->CURRENT_MAP_PATH_INDEX >= coyomi_yaml["MapPath"].size()) {
          loc->CURRENT_MAP_PATH_INDEX = 0;
        }
        MAP_PATH = coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
        // Reading Way Point
        tmp_wp.clear();
        wp.clear();
        tmp_wp = wpRead(MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>());
        WAYPOINT prev_target(0, 0, 0, 0);
        wavefrontplanner::Config cfg;
        cfg.map_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
        cfg.map_info_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>();
        wavefrontplanner::WaveFrontPlanner wfp(cfg);
        for (auto w: tmp_wp) {
          wavefrontplanner::Config cfg;
          cfg.start_x_m = prev_target.x;
          cfg.start_y_m = prev_target.y;
          cfg.goal_x_m  = w.x;
          cfg.goal_y_m  = w.y;
          wfp.Init(cfg);
          std::vector<wavefrontplanner::Path> path = wfp.SearchGoal();
          for (auto p: path) {
            WAYPOINT w(p.x, p.y, 0, 0);
            wp.emplace_back(w);
          }
          wp.emplace_back(w);  // この行が抜けていたので追記
          wp.back().stop_check = w.stop_check;
          prev_target.x = w.x;
          prev_target.y = w.y;
        }
        std::cout << "wave front completed. path size=" << wp.size() << std::endl;
        wp_list->size_wp_list = wp.size();
        for (int i = 0; i < wp.size(); i++) {
          wp_list->wp_list[i].x = wp[i].x;
          wp_list->wp_list[i].y = wp[i].y;
          wp_list->wp_list[i].a = wp[i].a;
          wp_list->wp_list[i].stop_check = wp[i].stop_check;
        }
        // 地図・初期位置のリセットトリガー
        loc->change_map_trigger = ChangeMapTrigger::kChange;
        while(loc->change_map_trigger == ChangeMapTrigger::kChange) {
          usleep(100000);
        }
        clear();
      }
    }

    if (isFREE.load()) {
      v = 0.0;
      w = 0.0;
    }
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    long long ts = get_current_time();
    read_state(odo, ts);

    enc->ts = ts;
    enc->x = odo.rx;
    enc->y = odo.ry;
    enc->a = odo.ra;

    disp->v = v;
    disp->w = w;

    std::string path = log_path->path + "/enclog";
    enc_log.open(path, std::ios_base::app);
    enc_log
      << ts << " "
      << odo.rx << " " << odo.ry << " " << odo.ra << " "
      << v << " " << w << " "
      << "end" << "\n";
    enc_log.close();
  }
  //=====<<MAIN LOOP : END>>=====

  //CLEANUP:
  // safe stop
  v = 0.0;
  w = 0.0;
  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  sleep_for(seconds(1));
  turn_off_motors();
  std::cerr << TEXT_BLUE << "Motors safe stop\n" << TEXT_COLOR_RESET;
  close(fd_motor);

  SDL_JoystickClose(joystick);
  SDL_Quit();

  enc_log.close();
  fout_urg2d.close();
  mcl_log.close();
  de_log.close();

  double message_travel = enc->total_travel;
  double message_voltage = bat->voltage;

  running.store(false);
  th_display.join();
  th_battery_logger.join();
  th_sound_logger.join();
  th_3D_Lidar.join();
  th_2D_Lidar_b.join();
  th_2D_Lidar_t.join();
  th_localization.join();

  std::cerr << "===========" << std::endl;
  std::cerr << "Total travel: " << message_travel << "[m]" << std::endl;
  std::cerr << "Battery voltage: " << message_voltage << "[V]" << std::endl;

  return 0;
}

void sigcatch(int sig) {
  gotoEnd.store(true);

  std::cerr << TEXT_RED;
  std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;
}
