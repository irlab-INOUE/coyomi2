#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <linux/joystick.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
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


//#define MAP_PATH "map/log230731_1F/"
#define MAP_PATH "map/log230729_2F/go/"
//#define MAP_PATH "map/log230729_2F/back/"

#define N 256 	// 日時の型変換に使うバッファ数
               
int fd;
int fd_js;    // file descriptor to joystick
// 共有したい構造体毎にアドレスを割り当てる
ENC *shm_enc        = nullptr;
URG2D *shm_urg2d    = nullptr;
BAT *shm_bat        = nullptr;
LOC *shm_loc        = nullptr;
LOGDIR *shm_logdir  = nullptr;

#include "OrientalMotorInterface.h"
//#define DEBUG_SENDRESP

using namespace std::chrono;

void sigcatch( int );

// joystick setup parameter
int *axis = NULL, num_of_axis = 0, num_of_buttons = 0;
char *button = NULL, name_of_joystick[80];
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

void read_joystick(js_event &js, double &v, double &w) {
  /* read the joystick state */
  ssize_t a = read(fd_js, &js, sizeof(struct js_event));

  /* see what to do with the event */
  switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_AXIS:
      axis[js.number] = js.value;
      break;
    case JS_EVENT_BUTTON:
      button[js.number] = js.value;
      if(js.value){
        while (js.value) {
          ssize_t a = read(fd_js, &js, sizeof(struct js_event));
          usleep(1000);
        }

        switch(js.number) {
          case 0:
            std::cerr << "No." << (int)js.number << "\tTurn Left" << std::endl;
            w = 0.5;
            break;
          case 1:
            std::cerr << "No." << (int)js.number << "\tFoward" << std::endl;
            w = 0.0;
            if (v < 0.01) v = 0.1;
            break;
          case 2:
            std::cerr << "No." << (int)js.number << "\tStop" << std::endl;
            v = 0.0;
            w = 0.0;
            break;
          case 3:
            std::cerr << "No." << (int)js.number << "\tRight" << std::endl;
            w = -0.5;
            break;
          case 4:
            std::cerr << "No." << (int)js.number << "\tSpeed Down" << std::endl;
            v -= 0.1;
            if (v < 0.0) v = 0.0;
            break;
          case 5:
            std::cerr << "No." << (int)js.number << "\tSpeed Up" << std::endl;
            v += 0.1;
            if (v > FORWARD_MAX_SPEED) v = FORWARD_MAX_SPEED;
            break;
          case 6:
            //std::cerr << "End\n";
            v = 0.0;
            w = 0.0;
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            usleep(1500000);
            gotoEnd = true;
            break;
          case 7:
            //std::cerr << "FREE\n";
            v = 0.0;
            w = 0.0;
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
            break;
        }
      }
      break;
  }
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

void WaypointEditor() {
  // Reading Way Point
  std::vector<WAYPOINT> wp;
  wp = wpRead(std::string(MAP_PATH) + "wp.txt");
  int wp_index = 0;
  std::string path_to_map = std::string(MAP_PATH);
  std::string map_name = path_to_map + "occMap.png";
  MapPath map_path(path_to_map, map_name, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);
  Viewer view(map_path);                        // 現在のoccMapを表示する
  view.hold();
  cv::moveWindow("occMap", 400, 0);
  while (1) {
    view.reset();
    view.plot_wp(wp);
    view.show(wp[wp_index].x, wp[wp_index].y, 5);
    std::cout << "INDEX:" << wp_index << " " << wp[wp_index].x << " " << wp[wp_index].y << "\n";
    int key = cv::waitKey();
    if (key == 'q') return;
    if (key == 'n') wp_index++;
    else if (key == 'p') wp_index--;
    else if (key == 'r') {
      wp = wpRead(std::string(MAP_PATH) + "wp.txt");
      wp_index = 0;
    }
    if (wp_index >= wp.size()) wp_index = 0;
    else if (wp_index < 0) wp_index = wp.size() - 1;
    usleep(5000);
  }
}

std::vector<pid_t> p_list;
int main(int argc, char *argv[]) {
	/* Ctrl+c 対応 */
	if (SIG_ERR == signal( SIGINT, sigcatch )) {
		std::printf("failed to set signal handler\n");
		exit(1);
	}

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
      std::cout << "Hello, Coyomi2 localization.\n";
      break;
    }
    else if (MODE == '2') {
      std::cout << "Hello, Coyomi2 Navigation.\n";
      break;
    }
    else if (MODE == '3') {
      std::cout << "Hello, Coyomi2 Waypoint editor.\n";
      WaypointEditor();
      return 0;
      break;
    }
  }


  /*
   * Configその他の読み込みセクション
   */
	// coyomi.yamlに接続する
  std::string path_to_yaml = DEFAULT_ROOT + std::string("/coyomi.yaml");
	YAML::Node coyomi_yaml = yamlRead(path_to_yaml);
  std::cerr << "coyomi.yaml is open.\n";


	/**************************************************************************
		共有メモリの確保
	 ***************************************************************************/
	// 共有したい構造体毎にアドレスを割り当てる
	shm_enc        =        (ENC *)shmAt(KEY_ENC, sizeof(ENC));
	shm_urg2d      =      (URG2D *)shmAt(KEY_URG2D, sizeof(URG2D));
	shm_bat        =        (BAT *)shmAt(KEY_BAT,   sizeof(BAT));
	shm_loc        =        (LOC *)shmAt(KEY_LOC, sizeof(LOC));
	shm_logdir     =     (LOGDIR *)shmAt(KEY_LOGDIR, sizeof(LOGDIR));
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
    getchar();
	} else {
		std::cerr << "ログは保管しません\n";
	}


  if((fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY)) == -1) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd: " << fd << "\n";
  }

  if ((fd_js = open(JS_PORT, O_RDONLY)) == -1) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd_js: " << fd_js << "\n";
  }

  // Joystick setup
  struct js_event js;
  ioctl(fd_js, JSIOCGAXES, &num_of_axis);
  ioctl(fd_js, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(fd_js, JSIOCGNAME(80), &name_of_joystick);

  axis = (int *)calloc(num_of_axis, sizeof(int));
  button = (char *)calloc(num_of_buttons, sizeof(char));

  std::cerr << "Joystick detected:" << name_of_joystick << std::endl;
  std::cerr << num_of_axis << " axis" << std::endl;
  std::cerr << num_of_buttons << " buttons" << std::endl << std::endl;

  fcntl(fd_js, F_SETFL, O_NONBLOCK);   /* use non-blocking mode */
  js.number = 0;
  std::cerr << "Joypad ready completed" << std::endl;

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd, TCSANOW, &tio);

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

  /* -----< curses : START >----- */
  int key;    // curses用キーボード入力判定
  WINDOW *win = initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  curs_set(0);
  start_color();
  timeout(0);
  init_pair(1,COLOR_BLUE, COLOR_BLACK);
  /* -----< curses : END >----- */

  // Reading Way Point
  std::vector<WAYPOINT> wp;
  wp = wpRead(std::string(MAP_PATH) + "wp.txt");
  shm_enc->current_wp_index = 0;

  // Create the multi threads
  for (int i = 0; i < 4; i++) {
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
        default:
          std::cerr << "Error\n";
          break;
      }
      p_list.emplace_back(c_pid);
    } else {
      if (i == 0) {       // 2d-lidar
        Urg2d urg2d;
        shm_urg2d->start_angle = -135.0;
        shm_urg2d->end_angle = 135.0;
        shm_urg2d->step_angle = 0.25;
        shm_urg2d->max_echo_size = 3;
        shm_urg2d->size = 
          ((shm_urg2d->end_angle - shm_urg2d->start_angle)/shm_urg2d->step_angle + 1) 
          * shm_urg2d->max_echo_size;
        for (int i = 0; i < shm_urg2d->size; i++) {
          shm_urg2d->r[i] = 0;
        }

        std::string path = shm_logdir->path;
        path += "/urglog";
        while(1) {
          fout_urg2d.open(path, std::ios_base::app);
          auto time_now = high_resolution_clock::now();
          long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
          std::vector<LSP> result = urg2d.getData();
          //urg2d.view(5);
          fout_urg2d << "LASERSCANRT" << " " 
            << ts << " "
            << result.size() * 3 << " " 
            << "-135" << " " << "135" << " " 
            << "0.25" << " " << "3" << " ";
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
        }
        exit(EXIT_SUCCESS);
      } else if (i == 1) { // battery
        std::string path = shm_logdir->path;
        path += "/batlog";
        while (1) {
          bat_log.open(path, std::ios_base::app);
          auto time_now = high_resolution_clock::now();
          long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
          bat_log << shm_bat->ts << " " << shm_bat->voltage << "\n";
          sleep(1);
          bat_log.close();
        }
        exit(EXIT_SUCCESS);
      } else if (i == 2) { // localization
        // Map file path
        std::string path_to_map = std::string(MAP_PATH);
        std::string map_name = path_to_map + "occMap.png";
				map_name.copy(shm_loc->path_to_map_dir, map_name.size());
        // Likelyhood file path
        std::string likelyhood_field = path_to_map + "lfm.txt";
        likelyhood_field.copy(shm_loc->path_to_likelyhood_field, likelyhood_field.size());
        // Initial pose 
        double initial_pose_x = 0.0;
        double initial_pose_y = 0.0;
        double initial_pose_a = 0.0;

        shm_loc->x = initial_pose_x; shm_loc->y = initial_pose_y; shm_loc->a = initial_pose_a;
        //std::cerr << "初期姿勢を" << shm_loc->x << "," << shm_loc->y << "," << shm_loc->a << "に設定\n";
        Pose2d currentPose(0.0, 0.0, 0.0);
        Pose2d previousPose = currentPose;
        // パーティクル初期配置
        //std::cerr << "MCL setup...";
        MCL mcl(Pose2d(initial_pose_x, initial_pose_y, initial_pose_a));
        mcl.set_lfm(likelyhood_field);
        mcl.set_mapInfo(path_to_map + "mapInfo.yaml");
        //std::cerr << "done.\n";

        MapPath map_path(path_to_map, map_name, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);
        Viewer view(map_path);                        // 現在のoccMapを表示する
        view.hold();
        view.show(0, 0, 5);
        cv::moveWindow("occMap", 400, 0);
        // MCL(KLD_sampling)h 
        while(1) {
          view.plot_wp(wp);
          view.show(shm_loc->x, shm_loc->y, 5);
          // 動いてなければ自己位置推定はしない
          currentPose = Pose2d(shm_enc->x, shm_enc->y, shm_enc->a);
          double _rot = currentPose.a - previousPose.a;
          double _tran = std::hypot(currentPose.x - previousPose.x, currentPose.y - previousPose.y);
          std::vector<LSP> lsp;
          if ((_tran < 1e-4) && (fabs(_rot) < 1e-8)) {
            ;
          } else {
            double th = shm_urg2d->start_angle * M_PI/180.0;
            double dth = shm_urg2d->step_angle * M_PI/180.0;
            for (int k = 0; k < shm_urg2d->size; k++) {
              lsp.emplace_back(shm_urg2d->r[k], shm_urg2d->r[k]/1000.0, th);
              th += dth;
            }
            mcl.KLD_sampling(lsp, currentPose, previousPose);
            //total_travel += _tran;
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
          auto time_now = high_resolution_clock::now();
          long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
          mcl_log 
            << ts << " " 
            << estimatedPose.x << " " << estimatedPose.y << " " << estimatedPose.a << " "
            << particle.size() << " "
            << "end" << "\n";
          mcl_log.close();

          // 次のループの準備
          previousPose = currentPose;
          usleep(20000);
        }
        exit(EXIT_SUCCESS);
      } else if (i == 3) { // State display
        std::cerr << "Please waite 5 sec...";
        sleep(5);
        clear();
        mvprintw(0, 0, "MCL Information");
        mvprintw(1, 0, "X[m]     Y[m]      A[deg]");
        mvprintw(4, 0, "Motor Information");
        mvprintw(5, 0, "X[m]     Y[m]      A[deg]");
        while (1) {
          move(2, 0); clrtoeol();
          mvprintw(2, 0, "%.3f", shm_loc->x);
          mvprintw(2, 9, "%.3f", shm_loc->y);
          mvprintw(2,19, "%.3f", shm_loc->a*180/M_PI);
          move(3, 0); clrtoeol();
          mvprintw(3, 0, "Current WP Index: %d", shm_enc->current_wp_index);
          move(6, 0); clrtoeol();
          mvprintw(6, 0, "%.3f", shm_enc->x);
          mvprintw(6, 9, "%.3f", shm_enc->y);
          mvprintw(6,19, "%.3f", shm_enc->a*180/M_PI);
          move(7, 0); clrtoeol();
          mvprintw(7, 0, "Total %.1f", shm_enc->total_travel);
          move(8, 0); clrtoeol();
          mvprintw(8, 0, "Voltage: %.1f", shm_enc->battery);
          move(9, 0); clrtoeol();
          mvprintw(9, 0, "TmpL_D %.1f  TmpR_D %.1f", shm_enc->temp_driver_L, shm_enc->temp_driver_R);
          move(10, 0); clrtoeol();
          mvprintw(10, 0, "TmpL_M %.1f  TmpR_M %.1f", shm_enc->temp_motor_L, shm_enc->temp_motor_R);
          refresh();
        }
      }
    }
  }

  //----------------------------------------------------------
  // Starting Main Process
  //----------------------------------------------------------
  //std::cerr << "Start rotation. Please hit Enter key.\n";
  //getchar();
  //std::cerr << "\033[2J" << "\033[1;1H";
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  int number_of_lidar_view_count = 1;
  int lidar_view_countdown = number_of_lidar_view_count;
  tcflush(fd, TCIOFLUSH);
  DynamicWindowApproach dwa(coyomi_yaml);
  double arrived_check_distance = coyomi_yaml["MotionControlParameter"]["arrived_check_distance"].as<double>();


  v = 0.0;
  w = 0.0;
  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  usleep(1000000);
  isFREE = !isFREE;
  free_motors();
  while (isFREE) {
    read_joystick(js, v, w);
    if (gotoEnd) goto CLEANUP;
    usleep(100000);

    auto time_now = high_resolution_clock::now();
    long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
    read_state(odo, ts);
    shm_enc->ts = ts;
    shm_enc->x = odo.rx;
    shm_enc->y = odo.ry;
    shm_enc->a = odo.ra;
  }
  while(1) {
    read_joystick(js, v, w);
    if (gotoEnd) goto CLEANUP;

    std::vector<LSP> lsp;
    double th = shm_urg2d->start_angle * M_PI/180.0;
    double dth = shm_urg2d->step_angle * M_PI/180.0;
    for (int k = 0; k < shm_urg2d->size; k++) {
      lsp.emplace_back(shm_urg2d->r[k], shm_urg2d->r[k]/1000.0, th);
      th += dth;
    }
    Pose2d estimatedPose(shm_loc->x, shm_loc->y, shm_loc->a);
    if (MODE == '2') {
      std::tie(v, w) = 
        dwa.run(lsp, estimatedPose, v, w, wp[shm_enc->current_wp_index]);
    }
    double dist2wp = std::hypot(wp[shm_enc->current_wp_index].x - estimatedPose.x, wp[shm_enc->current_wp_index].y - estimatedPose.y);
    if (dist2wp > arrived_check_distance && dist2wp < 4*arrived_check_distance) {
      double dwa_v = v;
      v = v * 0.8;
      if (v < 0.1) v = 0.1; 
    } else
    if (dist2wp < arrived_check_distance) {
      if (wp[shm_enc->current_wp_index].stop_check == 1) {
        isFREE = !isFREE;
        free_motors();
        while (isFREE) {
          read_joystick(js, v, w);
          usleep(500000);
        }
      }
      shm_enc->current_wp_index += 1;
#ifdef THETAV
      calc_vw2hex(Query_NET_ID_WRITE, 0, 0);
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
      //isFREE = !isFREE;
      //if (isFREE) 
      //  free_motors();
    }
    if (shm_enc->current_wp_index >= wp.size()) {
      shm_enc->current_wp_index = 0;
      calc_vw2hex(Query_NET_ID_WRITE, 0, 0);
      simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
      sleep(3);

      double omega = 45.0/180.0*M_PI;         // rad/s
      double achieve_angle = 5.0/180.0*M_PI;  // rad
      calc_vw2hex(Query_NET_ID_WRITE, 0, omega);
      while (1) {
        simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
        auto time_now = high_resolution_clock::now();
        long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
        read_state(odo, ts);

        shm_enc->ts = ts;
        shm_enc->x = odo.rx;
        shm_enc->y = odo.ry;
        shm_enc->a = odo.ra;

        if (fabs(shm_loc->a) < achieve_angle) {
          calc_vw2hex(Query_NET_ID_WRITE, 0, 0);
          simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
          sleep(3);
          break;
        }
      }
      continue;
      goto CLEANUP;
    }

    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    auto time_now = high_resolution_clock::now();
    long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
    read_state(odo, ts);

    shm_enc->ts = ts;
    shm_enc->x = odo.rx;
    shm_enc->y = odo.ry;
    shm_enc->a = odo.ra;

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
  // turn off exitation on RL motor
  turn_off_motors();

  close(fd);
  close(fd_js);

  enc_log.close();
  fout_urg2d.close();
  bat_log.close();
  mcl_log.close();

	for (auto pid: p_list) {
		kill(pid, SIGKILL);
	}
	pid_t wait_pid;
	while ((wait_pid = wait(nullptr)) > 0)
		std::cout << "wait:" << wait_pid << "\n";

	// 共有メモリのクリア
	shmdt(shm_urg2d);
	shmdt(shm_bat);
	shmdt(shm_loc);
	shmdt(shm_logdir);
	int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;
	keyID = shmget(KEY_LOGDIR, sizeof(LOGDIR), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);

  return 0;
}

void sigcatch(int sig) {
  endwin();   // ncurses end

  std::cerr << "Total travel: " << shm_enc->total_travel << "[m]\n";

  std::cerr << TEXT_RED;
  std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;

  for (auto pid: p_list) {
    kill(pid, SIGKILL);
  }
  pid_t wait_pid;
  while ((wait_pid = wait(nullptr)) > 0)
    std::cout << "wait:" << wait_pid << "\n";

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

  close(fd_js);
  close(fd);

  enc_log.close();
  fout_urg2d.close();
  bat_log.close();
  mcl_log.close();

  // 共有メモリのクリア
  shmdt(shm_urg2d);
  shmdt(shm_bat);
	shmdt(shm_loc);
	shmdt(shm_logdir);
  int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT);
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
  shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_LOGDIR, sizeof(LOGDIR), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);

  std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;

  exit(1);
}
