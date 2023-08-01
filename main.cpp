#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/wait.h>
#include <linux/joystick.h>

#include "Urg2d.h"
#include "MCL.h"
#include "shm_board.h"
#include "Viewer.h"
#include "yaml-cpp/yaml.h"
#include "DWA.h"

int fd;
int fd_js;    // file descriptor to joystick
// 共有したい構造体毎にアドレスを割り当てる
ENC *shm_enc        = nullptr;
URG2D *shm_urg2d    = nullptr;
BAT *shm_bat        = nullptr;
LOC *shm_loc        = nullptr;

#include "Config.h"
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
            std::cerr << "End\n";
            v = 0.0;
            w = 0.0;
            calc_vw2hex(Query_NET_ID_WRITE, v, w);
            simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
            usleep(1500000);
            gotoEnd = true;
            break;
          case 7:
            std::cerr << "FREE\n";
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
  long long temp;
  fn >> buf.x >> buf.y >> buf.a >> temp;
  buf.stop_check = 0;
  while (!fn.eof()) {
    wp.emplace_back(buf);
    fn >> buf.x >> buf.y >> buf.a >> temp;
    buf.stop_check = 0;
  }
  std::cerr << "完了\n";
  return wp;
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
    << " 2 : Navigation\n";
  int MODE = -1;
  while (MODE < 0) {
    MODE = getchar();
  }
  if (MODE == '1' || MODE == '\n')
    std::cout << "Hello, Coyomi2 localization.\n";
  else if (MODE == '2')
    std::cout << "Hello, Coyomi2 Navigation.\n";

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
  std::cerr << TEXT_GREEN << "Completed shared memory allocation\n" << TEXT_COLOR_RESET;

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

  // Create the multi threads
  for (int i = 0; i < 3; i++) {
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

        while(1) {
          fout_urg2d.open("urglog", std::ios_base::app);
          auto time_now = high_resolution_clock::now();
          long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
          std::vector<LSP> result = urg2d.getData();
          urg2d.view(5);
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
        while (1) {
          bat_log.open("batlog", std::ios_base::app);
          auto time_now = high_resolution_clock::now();
          long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
          bat_log << shm_bat->ts << " " << shm_bat->voltage << "\n";
          sleep(1);
          bat_log.close();
        }
        exit(EXIT_SUCCESS);
      } else if (i == 2) { // localization
        // Map file path
        std::string map_name = "occMap.png";
				map_name.copy(shm_loc->path_to_map_dir, map_name.size());
        // Likelyhood file path
        std::string likelyhood_field = "bin/lfm.txt";
        likelyhood_field.copy(shm_loc->path_to_likelyhood_field, likelyhood_field.size());
        // Initial pose 
        shm_loc->x = 0.0; shm_loc->y = 0.0; shm_loc->a = 0.0;
        std::cerr << "初期姿勢を" << shm_loc->x << "," << shm_loc->y << "," << shm_loc->a << "に設定\n";
        Pose2d currentPose(0.0, 0.0, 0.0);
        Pose2d previousPose = currentPose;
        // パーティクル初期配置
        std::cerr << "MCL setup...";
        MCL mcl(Pose2d(0.0, 0.0, 0.0));
        mcl.set_lfm(likelyhood_field);
        mcl.set_mapInfo("bin/mapInfo.yaml");
        std::cerr << "done.\n";

        MapPath map_path("bin/", "bin/"+map_name, "","","bin/lfm.txt", "mapInfo.yaml", 0, 0, 0);
        Viewer view(map_path);                        // 現在のoccMapを表示する
        view.hold();
        // MCL(KLD_sampling)h 
        while(1) {
          view.show(5);
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

          // 次のループの準備
          previousPose = currentPose;
          usleep(20000);
        }
        exit(EXIT_SUCCESS);
      }
    }
  }

  //----------------------------------------------------------
  // Starting Main Process
  //----------------------------------------------------------
  //std::cerr << "Start rotation. Please hit Enter key.\n";
  //getchar();
  std::cerr << "\033[2J" << "\033[1;1H";
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  int number_of_lidar_view_count = 1;
  int lidar_view_countdown = number_of_lidar_view_count;
  tcflush(fd, TCIOFLUSH);
  DynamicWindowApproach dwa(coyomi_yaml);
  double arrived_check_distance = coyomi_yaml["MotionControlParameter"]["arrived_check_distance"].as<double>();
  std::cerr << "arrived distance: " << arrived_check_distance << "\n";
  std::vector<WAYPOINT> wp;
#if 1
  wp.emplace_back(3.0, 0.0);
  wp.emplace_back(5.5, 0.0);
  wp.emplace_back(5.5, -4.0);
  wp.emplace_back(5.5, -12.0);
  wp.emplace_back(11.5, -12.0);
  wp.emplace_back(1.0, -12.0);
  wp.emplace_back(6.0, -12.0);
  wp.emplace_back(6.0, -10.0);
  wp.emplace_back(5.5, 0.5);
  wp.emplace_back(0.0, 0.0);
#else
  std::cout << "wp reading...";
  wp = wpRead("bin/wp.txt");
  std::cout << "done.\n";
#endif
  int wp_index = 0;
  MODE = 2;
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
    if (MODE == 2) {
      std::tie(v, w) = 
        dwa.run(lsp, estimatedPose, v, w, wp[wp_index]);
    }
    if (std::hypot(wp[wp_index].x - estimatedPose.x, wp[wp_index].y - estimatedPose.y) < 1.0) {
      //calc_vw2hex(Query_NET_ID_WRITE, 0, 0);
      //simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
      //usleep(250000);
      wp_index++;
    }
    if (wp_index >= wp.size()) {
      wp_index = 0;
      calc_vw2hex(Query_NET_ID_WRITE, 0, 0);
      simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
      sleep(3);
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

    enc_log.open("enclog", std::ios_base::app);
    enc_log 
      << ts << " " 
      << odo.rx << " " << odo.ry << " " << odo.ra << " "
      << "end" << "\n";
    enc_log.close();
  }
  //=====<<MAIN LOOP : END>>=====

CLEANUP:
  // turn off exitation on RL motor
  turn_off_motors();

  close(fd);
  close(fd_js);

  enc_log.close();
  fout_urg2d.close();
  bat_log.close();

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
	int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;

  return 0;
}

void sigcatch(int sig) {
  std::cerr << TEXT_RED;
  std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;

  for (auto pid: p_list) {
    kill(pid, SIGKILL);
  }
  pid_t wait_pid;
  while ((wait_pid = wait(nullptr)) > 0)
    std::cout << "wait:" << wait_pid << "\n";

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

  // 共有メモリのクリア
  shmdt(shm_urg2d);
  shmdt(shm_bat);
	shmdt(shm_loc);
  int keyID = shmget(KEY_URG2D, sizeof(URG2D), 0666 | IPC_CREAT);
  shmctl(keyID, IPC_RMID, nullptr);
  keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
  shmctl(keyID, IPC_RMID, nullptr);
	keyID = shmget(KEY_LOC, sizeof(LOC), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);

  std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;
  exit(1);
}
