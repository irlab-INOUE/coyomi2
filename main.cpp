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

#include "Urg2d.h"
#include "MCL.h"
#include "shm_board.h"
#include "Viewer.h"
#include "yaml-cpp/yaml.h"
#include "DWA.h"
#include "checkDirectory.h"
#include "Config.h"
#include "wave_front_planner.h"

#define N 256 	// 日時の型変換に使うバッファ数

int fd_motor;
SDL_Joystick* joystick;

// 共有したい構造体毎にアドレスを割り当てる
ENC *shm_enc         = nullptr;
URG2D *shm_urg2d     = nullptr;
BAT *shm_bat         = nullptr;
LOC *shm_loc         = nullptr;
LOGDIR *shm_logdir   = nullptr;
WP_LIST *shm_wp_list = nullptr;
DISPLAY *shm_disp    = nullptr;
LOG_DATA *shm_log    = nullptr;

#include "OrientalMotorInterface.h"
//#define DEBUG_SENDRESP

using namespace std::chrono;

void sigcatch( int );

void signal_handler_SIGTERM(int signum) {
  if (signum == SIGTERM) {
    exit(0);
  }
};

bool LIDAR_STOP = false;
void signal_handler_SIGTERM_inLIDAR(int signum) {
  if (signum == SIGTERM) {
    LIDAR_STOP = true;
  }
};

bool isFREE = false;
bool gotoEnd = false;

// log file
std::ofstream enc_log;
std::ofstream fout_urg2d;
std::ofstream bat_log;
std::ofstream mcl_log;

YAML::Node yamlRead(std::string path) {
  try {
    return YAML::LoadFile(path);
  } catch(YAML::BadFile &e) {
    std::cerr << "read error! yaml is not exist."<< std::endl;
    exit(1);
  }
}

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

  /* read the joystick state */
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_JOYBUTTONUP:
      case 1:
        v = 0.0; w = 0.0;
        break;
      case 2:
        v = 0.0; w = 0.0;
        break;
      case 0:
        v = 0.0; w = 0.0;
        break;
      case 3:
        v = 0.0; w = 0.0;
        break;
      default:
        v = 0.0; w = 0.0;
        break;
      case SDL_JOYBUTTONDOWN:
        switch (static_cast<int>(e.jbutton.button)) {
          case 1:
            v = 0.5; w = 0.0;
            break;
          case 2:
            v = -0.5; w = 0.0;
            break;
          case 0:
            v = 0.0; w = 1.0;
            break;
          case 3:
            v = 0.0; w = -1.0;
            break;
          case 10:
            //std::cerr << "End\n";
            v = 0.0; w = 0.0;
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            usleep(1500000);
            gotoEnd = true;
            break;
          case 12:
            //std::cerr << "FREE\n";
            v = 0.0; w = 0.0;
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            usleep(1000000);
            isFREE = !isFREE;
            if (isFREE)
              free_motors();
            else
              turn_on_motors();
            break;
          default:
            v = 0.0; w = 0.0;
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
  // method 1
  //v = -axis1 * 0.8;
  //w = -axis0 * 10*M_PI/180.0;
  // method 2
}

std::vector<WAYPOINT> wpRead(std::string wpname) {
  if (wpname == "") {
    std::cerr << "WPファイルのパスを指定してください\n";
    exit(0);
  }
  //std::cerr << wpname << " を読み込みます...";
  std::vector<WAYPOINT> wp;
  std::fstream fn;
  fn.open(wpname);

  WAYPOINT buf;
  fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  while (!fn.eof()) {
    wp.emplace_back(buf);
    fn >> buf.x >> buf.y >> buf.a >> buf.stop_check;
  }
  //std::cerr << "完了\n";
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
    usleep(5000);
  }
}

long long get_current_time() {
  auto time_now = high_resolution_clock::now();
  long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
  return ts;
}

std::vector<pid_t> p_list;
int main(int argc, char *argv[]) {
  /* Ctrl+c 対応 */
  if (SIG_ERR == signal( SIGINT, sigcatch )) {
    std::printf("failed to set signal handler\n");
    exit(1);
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
    << " 3 : WayPoint editor\n";
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
      break;
    }
  }

  /**************************************************************************
        共有メモリの確保
     ***************************************************************************/
  // 共有したい構造体毎にアドレスを割り当てる
  shm_enc        =        (ENC *)shmAt(KEY_ENC, sizeof(ENC));
  shm_urg2d      =      (URG2D *)shmAt(KEY_URG2D, sizeof(URG2D));
  shm_bat        =        (BAT *)shmAt(KEY_BAT,   sizeof(BAT));
  shm_loc        =        (LOC *)shmAt(KEY_LOC, sizeof(LOC));
  shm_logdir     =     (LOGDIR *)shmAt(KEY_LOGDIR, sizeof(LOGDIR));
  shm_disp       =    (DISPLAY *)shmAt(KEY_DISPLAY, sizeof(DISPLAY));
  shm_wp_list    =    (WP_LIST *)shmAt(KEY_WP_LIST, sizeof(WP_LIST));
  shm_log        =   (LOG_DATA *)shmAt(KEY_LOG, sizeof(LOG_DATA));
  std::cerr << TEXT_GREEN << "Completed shared memory allocation\n" << TEXT_COLOR_RESET;
  /***************************************************************************
    LOG保管場所を作成する
    DEFAULT_LOG_DIRの場所にcoyomi_log ディレクトリがあるかチェックし，
        なければ作成する
     ***************************************************************************/
  std::string storeDir = DEFAULT_LOG_DIR;
  // 現在日付時刻のディレクトリを作成する
  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);
  char s[N] = {'\0'};
  bool STORE = true;
  if (STORE) {
    checkDir(storeDir);
    strftime(s, N, "%Y", pnow); strcpy(shm_logdir->year, s); shm_logdir->year[4] = '\0';
    strftime(s, N, "%m", pnow); strcpy(shm_logdir->mon,  s); shm_logdir->mon[2]  = '\0';
    strftime(s, N, "%d", pnow); strcpy(shm_logdir->mday, s); shm_logdir->mday[2] = '\0';
    strftime(s, N, "%H", pnow); strcpy(shm_logdir->hour, s); shm_logdir->hour[2] = '\0';
    strftime(s, N, "%M", pnow); strcpy(shm_logdir->min , s); shm_logdir->min[2]  = '\0';
    strftime(s, N, "%S", pnow); strcpy(shm_logdir->sec , s); shm_logdir->sec[2]  = '\0';
    storeDir += "/" + std::string(shm_logdir->year, 4)
      + "/" + std::string(shm_logdir->mon,  2)
      + "/" + std::string(shm_logdir->mday, 2)
      + "/" + std::string(shm_logdir->hour, 2)
      + std::string(shm_logdir->min, 2)
      + std::string(shm_logdir->sec, 2);
    std::cerr << storeDir << std::endl;
    storeDir.copy(shm_logdir->path, storeDir.size());
    checkDir(storeDir);

    std::cerr << "path: " << shm_logdir->path << "にログを保存します" << std::endl;
    std::cerr << "Press ENTER key";
    getchar();
  } else {
    std::cerr << "ログは保管しません\n";
  }

  /**************************************************************************
    セマフォの初期化
   ***************************************************************************/
  sem_init(&shm_log->sem, 1, 1); // 1 = プロセス間共有
  shm_log->current_index = 0;

  /**************************************************************************
        Connect check & open serial port
     ***************************************************************************/
  if((fd_motor = open(SERIAL_PORT_MOTOR, O_RDWR | O_NOCTTY)) == -1) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd_motor: " << fd_motor << "\n";
  }

  /**************************************************************************
        Joystick setup
     ***************************************************************************/
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0) {
    std::cerr << "Failure SDL initialize. " << SDL_GetError() << std::endl;
    return 1;
  }
  joystick = SDL_JoystickOpen(0);
  std::cerr << "Joystick detected:" << SDL_JoystickName(joystick) << std::endl;
  std::cerr << SDL_JoystickNumAxes(joystick) << " axis" << std::endl;
  std::cerr << SDL_JoystickNumButtons(joystick) << " buttons" << std::endl << std::endl;

  // calibrate axis until JS_EVENT_BUTTON pressed
  bool loop_out_flag = false;
  std::vector<joy_calib> j_calib(6);
  std::cerr << "Calibrate js axis. Rotate LEFT axis to the all direction, then press any button\n";
  SDL_Event e;
  while (1) {
    while (!loop_out_flag) {
      while (SDL_PollEvent(&e))
        switch (e.type) {
          case SDL_JOYBUTTONDOWN:
            std::cout << "Pressed SDL_JOYBUTTONDOWN\n";
            loop_out_flag = true;
            break;
          case SDL_JOYAXISMOTION:
            //j_calib[static_cast<int>(e.jaxis.axis)].set_val(e.jaxis.value);
            j_calib[static_cast<int>(e.jaxis.axis)].set_zero(e.jaxis.value);
            break;
          default:
            break;
        }
    }
    if (loop_out_flag) break;
    usleep(10000);
  }
  for (auto j: j_calib) {
    std::cout << j.min << " " << j.max << " " << j.zero << "\n";
  }
  std::cerr << "Joypad ready completed" << std::endl;

  /**************************************************************************
        Serial port setup
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
        Motor driver setup
     ***************************************************************************/
  // BLV-R Driver setup
  // ID Share Config.
  std::cerr << "ID Share configration...";
  simple_send_cmd(Query_IDshare_R, sizeof(Query_IDshare_R));
  simple_send_cmd(Query_IDshare_L, sizeof(Query_IDshare_L));
  simple_send_cmd(Query_READ_R,    sizeof(Query_READ_R));
  simple_send_cmd(Query_READ_L,    sizeof(Query_READ_L));
  simple_send_cmd(Query_WRITE_R,   sizeof(Query_WRITE_R));
  simple_send_cmd(Query_WRITE_L,   sizeof(Query_WRITE_L));
  std::cerr << "Done.\n";

  //trun on exitation on RL motor
  turn_on_motors();

  /**************************************************************************
        Ncurses setup
     ***************************************************************************/
  int key;    // curses用キーボード入力判定
  WINDOW *win = initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  curs_set(0);
  start_color();
  timeout(0);
  init_pair(1,COLOR_BLUE, COLOR_BLACK);

  /**************************************************************************
        Waypoint setup
        Note:
          For the wavefront planner to work properly, the occMap should have
          traversable areas marked in white.
     ***************************************************************************/
  shm_loc->CURRENT_MAP_PATH_INDEX = 0;
  if (argc > 1) shm_loc->CURRENT_MAP_PATH_INDEX = std::atoi(argv[1]);
  std::string MAP_PATH = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
  // Reading Way Point
  std::vector<WAYPOINT> tmp_wp, wp;
  tmp_wp = wpRead(MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>());
  WAYPOINT prev_target(0, 0, 0, 0);
  wavefrontplanner::Config cfg;
  cfg.map_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
  cfg.map_info_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>();
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
    wp.emplace_back(w);
    wp.back().stop_check = w.stop_check;
    prev_target.x = w.x;
    prev_target.y = w.y;
  }
  std::cout << "wave front completed. path size=" << wp.size() << std::endl;
  shm_wp_list->size_wp_list = wp.size();
  for (int i = 0; i < wp.size(); i++) {
    shm_wp_list->wp_list[i].x = wp[i].x;
    shm_wp_list->wp_list[i].y = wp[i].y;
    shm_wp_list->wp_list[i].a = wp[i].a;
    shm_wp_list->wp_list[i].stop_check = wp[i].stop_check;
  }
  shm_enc->current_wp_index = 0;
#if 0
  std::ofstream wplog("./wplog");
  for (int i = 0; i < shm_wp_list->size_wp_list; i++) {
    wplog << shm_wp_list->wp_list[i].x << " " << shm_wp_list->wp_list[i].y
      << " " << shm_wp_list->wp_list[i].a << " " << shm_wp_list->wp_list[i].stop_check << "\n";
  }
#endif

  /**************************************************************************
        initial pose setup
     ***************************************************************************/
  shm_loc->x = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_x"].as<double>();
  shm_loc->y = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_y"].as<double>();
  shm_loc->a = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_a"].as<double>() * M_PI/180;

  /**************************************************************************
        initial enc setup
     ***************************************************************************/
  long long first_ts = get_current_time();
  ODOMETORY first_odo;
  read_state(first_odo, first_ts);
  sleep(1);
  shm_enc->ts = first_ts;
  shm_enc->x = first_odo.rx;
  shm_enc->y = first_odo.ry;
  shm_enc->a = first_odo.ra;

  /**************************************************************************
        Multi threads setup
     ***************************************************************************/
  for (int i = 0; i < 5; i++) {
    pid_t c_pid = fork();
    if (c_pid == -1) {
      perror("fork");
      exit(EXIT_FAILURE);
    } else if (c_pid > 0) {
      switch (i) {
        case 0:
          std::cerr << "Start 2d-LiDAR log: " << c_pid << "\n";
          break;
        case 1:
          std::cerr << "Start battery log: " << c_pid << "\n";
          break;
        case 2:
          std::cerr << "Start Localization: " << c_pid << "\n";
          break;
        case 3:
          std::cerr << "Start state display: " << c_pid << "\n";
          break;
        case 4:
          std::cerr << "Start sound on: " << c_pid << "\n";
          break;
        default:
          std::cerr << "Error\n";
          break;
      }
      p_list.emplace_back(c_pid);
    } else {
      if (i == 0) {       // 2d-lidar
        signal(SIGTERM, signal_handler_SIGTERM_inLIDAR);
        shm_urg2d->start_angle = coyomi_yaml["2DLIDAR"]["start_angle"].as<double>();
        shm_urg2d->end_angle   = coyomi_yaml["2DLIDAR"]["end_angle"].as<double>();
        shm_urg2d->step_angle  = coyomi_yaml["2DLIDAR"]["step_angle"].as<double>();
        shm_urg2d->max_echo_size = coyomi_yaml["2DLIDAR"]["max_echo_size"].as<double>();
        shm_urg2d->size =
          ((shm_urg2d->end_angle - shm_urg2d->start_angle)/shm_urg2d->step_angle + 1)
          * shm_urg2d->max_echo_size;
        for (int i = 0; i < shm_urg2d->size; i++) {
          shm_urg2d->r[i] = 0;
        }
        Urg2d urg2d(shm_urg2d->start_angle, shm_urg2d->end_angle, shm_urg2d->step_angle);
        for (int i = 0; i < ((shm_urg2d->end_angle - shm_urg2d->start_angle)/shm_urg2d->step_angle + 1); i++) {
          double ang = (i * shm_urg2d->step_angle + shm_urg2d->start_angle)*M_PI/180;
          shm_urg2d->ang[i] = ang;
          shm_urg2d->cs[i] = cos(ang);
          shm_urg2d->sn[i] = sin(ang);
        }

        std::string path = shm_logdir->path;
        path += "/urglog";
        while(1) {
          fout_urg2d.open(path, std::ios_base::app);
          long long ts = get_current_time();
          std::vector<LSP> result = urg2d.getData();
          urg2d.view(5);
          fout_urg2d << "LASERSCANRT" << " "
            << ts << " "
            << static_cast<int>(result.size()) * shm_urg2d->max_echo_size << " "
            <<
            std::to_string(shm_urg2d->start_angle) << " "
            << std::to_string(shm_urg2d->end_angle) << " "
            << std::to_string(shm_urg2d->step_angle) << " "
            << shm_urg2d->max_echo_size << " ";
          for (auto d: result) {
            fout_urg2d << d.data << " " << "0" << " " << "0" << " ";
          }
          fout_urg2d << ts << "\n";
          fout_urg2d.close();

          shm_urg2d->ts = ts;
          shm_urg2d->ts_end = ts;
          shm_urg2d->size = result.size();
          for (int k = 0; k < result.size(); k++) {
            shm_urg2d->r[k] = result[k].data;
          }
          if (LIDAR_STOP) {
            urg2d.close();
            exit(0);
          }
        }
        exit(EXIT_SUCCESS);
      } else if (i == 1) { // battery
        signal(SIGTERM, signal_handler_SIGTERM);
        std::string path = shm_logdir->path;
        path += "/batlog";
        while (1) {
          bat_log.open(path, std::ios_base::app);
          long long ts = get_current_time();
          bat_log << shm_bat->ts << " " << shm_bat->voltage << "\n";
          sleep(1);
          bat_log.close();
          shm_disp->battery = shm_bat->voltage;
        }
        exit(EXIT_SUCCESS);
      } else if (i == 2) { // localization
        signal(SIGTERM, signal_handler_SIGTERM);
        while (1) {
          MAP_PATH = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
          // Map file path
          std::string MAP_NAME
            = MAP_PATH+ "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
          MAP_NAME.copy(shm_loc->path_to_map_dir, MAP_NAME.size());
          // Likelyhood file path
          std::string LIKELYHOOD_FIELD
            = MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["likelyhood_field"].as<std::string>();
          LIKELYHOOD_FIELD.copy(shm_loc->path_to_likelyhood_field, LIKELYHOOD_FIELD.size());
          // Initial pose
          if (shm_loc->CURRENT_MAP_PATH_INDEX != 0) {
            double initial_pose_x = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_x"].as<double>();
            double initial_pose_y = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_y"].as<double>();
            double initial_pose_a = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["init_a"].as<double>() * M_PI/180;
            shm_loc->x = initial_pose_x; shm_loc->y = initial_pose_y; shm_loc->a = initial_pose_a;
          }
          Pose2d currentPose = Pose2d(shm_enc->x, shm_enc->y, shm_enc->a);
          Pose2d previousPose = currentPose;
          // パーティクル初期配置
          MCL mcl(Pose2d(shm_loc->x, shm_loc->y, shm_loc->a));
          mcl.set_lfm(shm_loc->path_to_likelyhood_field);
          mcl.set_mapInfo(MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>());

          MapPath map_path(MAP_PATH, shm_loc->path_to_map_dir, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);
          Viewer view(map_path);                        // 現在のoccMapを表示する
          view.hold();
          view.show(shm_loc->x, shm_loc->y, 5);
          //cv::moveWindow("occMap", 700, 0);
          shm_loc->change_map_trigger = ChangeMapTrigger::kContinue;
          std::vector<WAYPOINT> wp;
          for (int i = 0; i < shm_wp_list->size_wp_list; i++) {
            wp.emplace_back(shm_wp_list->wp_list[i].x, shm_wp_list->wp_list[i].y, shm_wp_list->wp_list[i].a,
                            shm_wp_list->wp_list[i].stop_check);
          }
          // MCL(KLD_sampling)
          while(1) {
            if (shm_loc->change_map_trigger == ChangeMapTrigger::kChange) break;
            view.plot_wp(wp);
            view.plot_current_wp(wp[shm_enc->current_wp_index]);
            view.show(shm_loc->x, shm_loc->y, 5);
            // 動いてなければ自己位置推定はしない
            currentPose = Pose2d(shm_enc->x, shm_enc->y, shm_enc->a);
            double _rot = currentPose.a - previousPose.a;
            double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
            std::vector<LSP> lsp;
            if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
              ;
            } else {
              for (int k = 0; k < shm_urg2d->size; k++) {
                lsp.emplace_back(shm_urg2d->r[k], shm_urg2d->r[k]/1000.0, shm_urg2d->ang[k], shm_urg2d->cs[k], shm_urg2d->sn[k]);
              }
              mcl.KLD_sampling(lsp, currentPose, previousPose);
            }
            Pose2d estimatedPose = mcl.get_best_pose();
            std::vector<Pose2d> particle = mcl.get_particle_set();

            view.reset();
            view.robot(estimatedPose);
            view.urg(estimatedPose, lsp);
            view.particle(particle);

            shm_loc->x = estimatedPose.x;
            shm_loc->y = estimatedPose.y;
            shm_loc->a = estimatedPose.a;

            std::string path = shm_logdir->path;
            path += "/mcllog";
            mcl_log.open(path, std::ios_base::app);
            long long ts = get_current_time();
            mcl_log
              << ts << " "
              << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.a << " "
              << particle.size() << " "
              << "end" << "\n";
            mcl_log.close();

            // 次のループの準備
            previousPose = currentPose;
            usleep(30000);
          }
        }
        exit(EXIT_SUCCESS);
      } else if (i == 3) { // State display
        signal(SIGTERM, signal_handler_SIGTERM);
        std::cerr << "Please waite 5 sec...";
        sleep(5);
        clear();

        // 下部10行分のサブウィンドウを作成
        int screen_height, screen_width;
        getmaxyx(stdscr, screen_height, screen_width); // 端末サイズ取得
        int log_height = 10;
        int log_width = screen_width;
        int log_starty = screen_height - log_height;
        int log_startx = 0;
        WINDOW* log_win = newwin(log_height, log_width, log_starty, log_startx);

        std::string log_text = "TEST LOG START";
        add_log(shm_log, log_text);

        int ROW_MCL = 0;
        int ROW_MOTOR = 7;
        int ROW_TOTAL_TRAVEL = 14;
        int ROW_PATH = 15;
        int ROW_CURRENT_MAP_PATH_INDEX = 16;
        mvprintw(ROW_MCL,     0, "MCL Information");
        mvprintw(ROW_MCL+1,   0, "X[m]     Y[m]      A[deg]");
        mvprintw(ROW_MOTOR,   0, "Motor Information");
        mvprintw(ROW_MOTOR+1, 0, "X[m]     Y[m]      A[deg]");
        while (1) {
          // update status window
          shm_disp->temp_driver_L = shm_enc->temp_driver_L;
          shm_disp->temp_driver_R = shm_enc->temp_driver_R;
          shm_disp->temp_motor_L = shm_enc->temp_motor_L;
          shm_disp->temp_motor_R = shm_enc->temp_motor_R;

          move(ROW_MCL+2, 0); clrtoeol();
          mvprintw(ROW_MCL+2, 0, "%.3f", shm_disp->loc_x);
          mvprintw(ROW_MCL+2, 9, "%.3f", shm_disp->loc_y);
          mvprintw(ROW_MCL+2,19, "%.1f", shm_disp->loc_a*180/M_PI);

          move(ROW_MCL+3, 0); clrtoeol();
          printw("Current WP Index: %d", shm_disp->current_wp_index);

          move(ROW_MCL+4, 0); clrtoeol();
          printw("v: %.2f  w: %.2f", shm_disp->v, shm_disp->w);

          move(ROW_MCL+5, 0); clrtoeol();
          printw("obx: %.3f  oby: %.3f  ang: %.1f", shm_disp->min_obstacle_x, shm_disp->min_obstacle_y,
                 atan2(shm_disp->min_obstacle_y, shm_disp->min_obstacle_x) * 180/M_PI);

          move(ROW_MOTOR+2, 0); clrtoeol();
          mvprintw(ROW_MOTOR+2, 0, "%.3f", shm_disp->enc_x);
          mvprintw(ROW_MOTOR+2, 9, "%.3f", shm_disp->enc_y);
          mvprintw(ROW_MOTOR+2,19, "%.1f", shm_disp->enc_a*180/M_PI);

          move(ROW_MOTOR+3, 0); clrtoeol();
          printw("Voltage: %.1f", shm_disp->battery);

          move(ROW_MOTOR+4, 0); clrtoeol();
          printw("TmpL_D %.1f  TmpR_D %.1f", shm_disp->temp_driver_L, shm_disp->temp_driver_R);

          move(ROW_MOTOR+5, 0); clrtoeol();
          printw("TmpL_M %.1f  TmpR_M %.1f", shm_disp->temp_motor_L, shm_disp->temp_motor_R);

          move(ROW_TOTAL_TRAVEL, 0); clrtoeol();
          printw("Total %.1f", shm_disp->total_travel);

          move(ROW_PATH, 0); clrtoeol();
          printw("%s", shm_loc->path_to_map_dir);

          move(ROW_CURRENT_MAP_PATH_INDEX, 0); clrtoeol();
          printw("CURRENT_MAP_PATH_INDEX %d", shm_loc->CURRENT_MAP_PATH_INDEX);

          // update Log window
          draw_log_window(log_win, shm_log, log_width, log_height);

          refresh();
        }
        exit(EXIT_SUCCESS);
      } else if (i == 4) { // sound on
        signal(SIGTERM, signal_handler_SIGTERM);
        int prev_wp_index = 0;
        std::string path = shm_logdir->path;
        path += "/sound_log";
        std::ofstream sound_log;
        while (1) {
          sound_log.open(path, std::ios_base::app);
          long long ts = get_current_time();
          sound_log << ts << " " << prev_wp_index << " " << shm_enc->current_wp_index << "\n";
          sound_log.close();
          if (prev_wp_index != shm_enc->current_wp_index) {
            std::string cmd = "paplay /usr/share/sounds/freedesktop/stereo/complete.oga";
            int ret = std::system(cmd.c_str());
            prev_wp_index = shm_disp->current_wp_index;
            // WP更新をログメッセージ出す
            std::string log_text = "WP updated. Next target->" + std::to_string(shm_disp->current_wp_index);
            add_log(shm_log, log_text);
          }
          std::string log_text = "Running process. TimeStamp-> " + std::to_string(ts);
          add_log(shm_log, log_text);
          sleep(1);
        }
        exit(EXIT_SUCCESS);
      }
    }
  }

  /**************************************************************************
        Starting Main Process
     ***************************************************************************/
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  int number_of_lidar_view_count = 1;
  int lidar_view_countdown = number_of_lidar_view_count;
  tcflush(fd_motor, TCIOFLUSH);
  DynamicWindowApproach dwa(coyomi_yaml);
  double arrived_check_distance = coyomi_yaml["MotionControlParameter"]["arrived_check_distance"].as<double>();

  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  usleep(100000);
  isFREE = !isFREE;
  free_motors();
  std::string start_bell_cmd = "paplay /usr/share/sounds/freedesktop/stereo/bell.oga";
  int start_bell_ret = std::system(start_bell_cmd.c_str());

  while (isFREE) {
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);
    if (gotoEnd) goto CLEANUP;
    usleep(100000);

    long long ts = get_current_time();
    read_state(odo, ts);
    shm_enc->ts = ts;
    shm_enc->x = odo.rx;
    shm_enc->y = odo.ry;
    shm_enc->a = odo.ra;

    shm_disp->enc_x = odo.rx;
    shm_disp->enc_y = odo.ry;
    shm_disp->enc_a = odo.ra;
  }
  start_bell_ret = std::system(start_bell_cmd.c_str());

  while(1) {
    double tmp_v, tmp_w;
    read_joystick(tmp_v, tmp_w, j_calib);
    if (gotoEnd) goto CLEANUP;

    std::vector<LSP> lsp;
    for (int k = 0; k < shm_urg2d->size; k++) {
      lsp.emplace_back(shm_urg2d->r[k], shm_urg2d->r[k]/1000.0, shm_urg2d->ang[k], shm_urg2d->cs[k], shm_urg2d->sn[k]);
    }
    Pose2d estimatedPose(shm_loc->x, shm_loc->y, shm_loc->a);
    if (MODE == '1') {
      v = tmp_v; w = tmp_w;
      //isFREE = true;
    }
    if (MODE == '2') {
      double obx, oby;
      std::tie(v, w, obx, oby) = dwa.run(lsp, estimatedPose, v, w, wp[shm_enc->current_wp_index]);
      shm_disp->min_obstacle_x = obx;
      shm_disp->min_obstacle_y = oby;

#if 1
      // rotate ricovery
      if (fabs(v) < 1e-6 && fabs(w) <1e-6) {
        if (oby >= 0) w = -M_PI/8.0;
        else w = M_PI/8.0;
        v = 0.3 * w;  // rodate radius is 0.3[m]
        if (v > 0.0) v = -v;
        //sleep(1);
      }
#endif

      double dist2wp = std::hypot(wp[shm_enc->current_wp_index].x - estimatedPose.x,
                                  wp[shm_enc->current_wp_index].y - estimatedPose.y);
      if (wp[shm_enc->current_wp_index].stop_check == 2) {
        if (dist2wp < arrived_check_distance) {
          double dwa_v = v;
          v = v * 0.8;
          if (v < 0.1) v = 0.1;
        }
        if (dist2wp < 0.5) {
          shm_enc->current_wp_index += 1;
        }
      } else if (wp[shm_enc->current_wp_index].stop_check == 1) {
        if (dist2wp < 0.5) {
          shm_enc->current_wp_index += 1;
          isFREE = !isFREE;
          free_motors();
          while (isFREE) {
            read_joystick(v, w, j_calib);
            long long ts = get_current_time();
            read_state(odo, ts);
            shm_enc->ts = ts;
            shm_enc->x = odo.rx;
            shm_enc->y = odo.ry;
            shm_enc->a = odo.ra;

            shm_disp->enc_x = odo.rx;
            shm_disp->enc_y = odo.ry;
            shm_disp->enc_a = odo.ra;
          }
        }
      } else if (dist2wp < arrived_check_distance) {
        shm_enc->current_wp_index += 1;
#ifdef THETAV
        v = 0; w = 0;
        calc_vw2hex(Query_NET_ID_WRITE, v, w);
        simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
        sleep(2);

        std::string lx = std::to_string(estimatedPose.x);
        std::string ly = std::to_string(estimatedPose.y);
        std::string cmd = "./bin/capture " + lx + " " + ly;
        int ret = std::system(cmd.c_str());
        if (ret == -1) {
          std::cerr << "script error" << std::endl;
        }
#endif
      }
      if (shm_enc->current_wp_index >= wp.size()) {
        shm_enc->current_wp_index = 0;
        v = 0; w = 0;
        calc_vw2hex(Query_NET_ID_WRITE, v, w);
        simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
        sleep(1);

        // Change current map
        shm_loc->CURRENT_MAP_PATH_INDEX++;
        if (shm_loc->CURRENT_MAP_PATH_INDEX >= coyomi_yaml["MapPath"].size()) {
          shm_loc->CURRENT_MAP_PATH_INDEX = 0;
        }
        MAP_PATH = coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["path"].as<std::string>();
#if 0
        // Reading Way Point
        wp = wpRead(MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>());
        shm_wp_list->size_wp_list = wp.size();
        for (int i = 0; i < wp.size(); i++) {
          shm_wp_list->wp_list[i].x = wp[i].x;
          shm_wp_list->wp_list[i].y = wp[i].y;
          shm_wp_list->wp_list[i].a = wp[i].a;
          shm_wp_list->wp_list[i].stop_check = wp[i].stop_check;
        }
#endif
        // Reading Way Point
        tmp_wp.clear();
        wp.clear();
        tmp_wp = wpRead(MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["way_point"].as<std::string>());
        WAYPOINT prev_target(0, 0, 0, 0);
        wavefrontplanner::Config cfg;
        cfg.map_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["occupancy_grid_map"].as<std::string>();
        cfg.map_info_path = MAP_PATH + "/" + coyomi_yaml["MapPath"][shm_loc->CURRENT_MAP_PATH_INDEX]["mapInfo"].as<std::string>();
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
        shm_wp_list->size_wp_list = wp.size();
        for (int i = 0; i < wp.size(); i++) {
          shm_wp_list->wp_list[i].x = wp[i].x;
          shm_wp_list->wp_list[i].y = wp[i].y;
          shm_wp_list->wp_list[i].a = wp[i].a;
          shm_wp_list->wp_list[i].stop_check = wp[i].stop_check;
        }
        // 地図・初期位置のリセットトリガー
        shm_loc->change_map_trigger = ChangeMapTrigger::kChange;
        while(shm_loc->change_map_trigger == ChangeMapTrigger::kChange) {
          usleep(100000);
        }
        clear();
      }
    }

    if (isFREE) {
      v = 0.0;
      w = 0.0;
    }
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    long long ts = get_current_time();
    read_state(odo, ts);

    shm_enc->ts = ts;
    shm_enc->x = odo.rx;
    shm_enc->y = odo.ry;
    shm_enc->a = odo.ra;

    shm_disp->enc_x = odo.rx;
    shm_disp->enc_y = odo.ry;
    shm_disp->enc_a = odo.ra;
    shm_disp->loc_x = shm_loc->x;
    shm_disp->loc_y = shm_loc->y;
    shm_disp->loc_a = shm_loc->a;
    shm_disp->total_travel = shm_enc->total_travel;
    shm_disp->current_wp_index = shm_enc->current_wp_index;
    shm_disp->current_map_path_index = shm_loc->CURRENT_MAP_PATH_INDEX;
    shm_disp->v = v;
    shm_disp->w = w;

    std::string path = shm_logdir->path;
    path += "/enclog";
    enc_log.open(path, std::ios_base::app);
    enc_log
      << ts << " "
      << odo.rx << " " << odo.ry << " " << odo.ra << " "
      << v << " " << w << " "
      << "end" << "\n";
    enc_log.close();
  }
  //=====<<MAIN LOOP : END>>=====

CLEANUP:
  endwin();   // ncurses end
  std::cerr << "Total travel: " << shm_enc->total_travel << "[m]\n";
  std::cerr << "Battery voltage: " << shm_bat->voltage << "[V]\n";
  // turn off exitation on RL motor
  turn_off_motors();

  close(fd_motor);
  SDL_JoystickClose(joystick);
  SDL_Quit();

  enc_log.close();
  fout_urg2d.close();
  bat_log.close();
  mcl_log.close();

  for (auto pid: p_list) {
    kill(pid, SIGTERM);
  }
  pid_t wait_pid;
  while ((wait_pid = wait(nullptr)) > 0)
    std::cout << "wait:" << wait_pid << "\n";

  // 共有メモリのクリア
  std::ofstream shmid(std::string(shm_logdir->path) + "/shmID.txt");
  shmdt(shm_urg2d);
  shmdt(shm_bat);
  shmdt(shm_loc);
  shmdt(shm_logdir);
  shmdt(shm_disp);
  shmdt(shm_wp_list);
  shmdt(shm_log);
  int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT); shmid << "URG2D " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT); shmid << "BAT " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT); shmid << "LOC " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOGDIR, sizeof(LOGDIR), 0666 | IPC_CREAT); shmid << "LOGDIR " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_DISPLAY, sizeof(DISPLAY), 0666 | IPC_CREAT); shmid << "DISPLAY " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_WP_LIST, sizeof(WP_LIST), 0666 | IPC_CREAT); shmid << "WP_LIST " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOG, sizeof(LOG_DATA), 0666 | IPC_CREAT); shmid << "LOG_DATA " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);

  return 0;
}

void sigcatch(int sig) {
  endwin();   // ncurses end

  std::cerr << "Total travel: " << shm_enc->total_travel << "[m]\n";
  std::cerr << "Battery voltage: " << shm_bat->voltage << "[V]\n";

  std::cerr << TEXT_RED;
  std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;

  for (auto pid: p_list) {
    kill(pid, SIGTERM);
  }
  pid_t wait_pid;
  while ((wait_pid = wait(nullptr)) > 0) {
    // std::cout << "wait:" << wait_pid << "\n";
  }

  pid_t pid = getpid();
  std::cout << "If main can't stop, execute next command " << TEXT_GREEN << "**kill " << pid << "** "
    << TEXT_COLOR_RESET << "in terminal.\n";
  // safe stop
  double v = 0.0;
  double w = 0.0;
  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  usleep(1500000);
  turn_off_motors();
  std::cerr << TEXT_BLUE << "Motors safe stop\n" << TEXT_COLOR_RESET;

  SDL_JoystickClose(joystick);
  SDL_Quit();
  close(fd_motor);

  enc_log.close();
  fout_urg2d.close();
  bat_log.close();
  mcl_log.close();

  // 共有メモリのクリア
  std::ofstream shmid(std::string(shm_logdir->path) + "/shmID.txt");
  shmdt(shm_urg2d);
  shmdt(shm_bat);
  shmdt(shm_loc);
  shmdt(shm_logdir);
  shmdt(shm_disp);
  shmdt(shm_wp_list);
  shmdt(shm_log);
  int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT); shmid << "URG2D " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT); shmid << "BAT " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT); shmid << "LOC " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOGDIR, sizeof(LOGDIR), 0666 | IPC_CREAT); shmid << "LOGDIR " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_DISPLAY, sizeof(DISPLAY), 0666 | IPC_CREAT); shmid << "DISPLAY " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_WP_LIST, sizeof(WP_LIST), 0666 | IPC_CREAT); shmid << "WP_LIST " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_LOG, sizeof(LOG_DATA), 0666 | IPC_CREAT); shmid << "LOG_DATA " << keyID << "\n";
  shmctl(keyID, IPC_RMID, nullptr);

  std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;

  exit(1);
}
