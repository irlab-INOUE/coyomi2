#ifndef COYOMI_INCLUDE_SHM_BOARD_H
#define COYOMI_INCLUDE_SHM_BOARD_H

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/shm.h>
#include <stdlib.h>
#include <semaphore.h>
#include "Urg2d.h"

// shm取得から割り当てをまとめて行う
void *shmAt(key_t KEY, size_t key_size) {
	int keyID = shmget(KEY, key_size, 0666 | IPC_CREAT);
  if (keyID == -1) {
    std::cerr << KEY << "のshmgetに失敗しました\n";
    std::cerr << std::strerror(errno) << "\n";
    exit(1);
  } else {
    //std::cerr << keyID << "を取得済み\n";
  }
  return shmat(keyID, 0, 0);
}

enum class Status {
  Ready,
  Run
};

// for Encoder receiver
#define KEY_ENC	1235
struct ENC {
	Status state;
  long long ts;
  double x;
  double y;
  double a;
  double v;
  double omega;
  double ac;
  double wa;
  double total_travel;
  int cmdLed;
	long long left;
	long long right;
	double ax;
	double ay;
	double az;
	double wx;
	double wy;
	double wz;
	double mx;
	double my;
	double mz;
  double battery;
  double temp_driver_R;
  double temp_motor_R;
  double temp_driver_L;
  double temp_motor_L;
  int current_wp_index;
};

// for 2D-LIDAR
#define KEY_URG2D 1244
struct URG2D {
	Status state;
	long long ts;
	long long ts_end;
  double start_angle;
  double end_angle;
  double step_angle;
  int size;
  int max_echo_size;
  long r[5000];
  double ang[1081];
  double cs[1081];
  double sn[1081];
};

// for 3D-LIDAR
#define KEY_URG3D 1255
struct POINT3D {
  double x;
  double y;
  double z;
  double r;
  double phi;
  double theta;
  int intensity;
};

struct URG3D {
  Status state;
  long long ts;
  int size;
  POINT3D pt[30000];
  bool measure;
};

// for LOG-DIR
#define KEY_LOGDIR 1266
struct LOGDIR {
  char year[5];
  char  mon[3];
  char mday[3];
  char hour[3];
  char  min[3];
  char  sec[3];
  char path[256]; 	// フルパス
};

#define keyJs	333
#define keyLoc	4000

// for Control
#define KEY_CTRL 1277
struct CTRL {
  double v;
  double w;
	double velocity_max;
	double velocity_down_coefficient;
	double back_velocity;
	double arrived_check_distance;
	bool auto_loop;
};

// for ROUTE_LIST
#define KEY_WP_LIST 1288
enum class ChangeWPTrigger {
	kContinue = 0,
	kChange = 1,
};
struct ROUTE_POINT {
	double x;
	double y;
	double a;
  int stop_check;
};
struct WP_LIST {
	char path_to_wp_file[256]; 			// 現在使用中のWPファイルへのフルパス
	ChangeWPTrigger change_wp_trigger;	// WPのリセットトリガー
	int size_wp_list;
	int size_route_list;
	int target_index;
	ROUTE_POINT wp_list[3000]; 		// WAY POINT
	ROUTE_POINT route_list[3000]; 	// 細分化した通過点
	int wp_index_list[3000]; 		// 通過点が目指しているWAY POINTのインデックス
};

// for Localization
#define KEY_LOC 1299
enum class ChangeMapTrigger {
	kContinue = 0,
	kChange = 1,
};
struct LOC {
	Status state; 							// localization実行/停止
	char path_to_map_dir[256]; 				// 現在使用中の占有格子地図があるディレクトリパス
	char path_to_likelyhood_field[256]; 	// 現在使用中の尤度場へのパス
	ChangeMapTrigger change_map_trigger;	// 地図・初期位置のリセットトリガー
  long long ts;
  double x;
  double y;
  double a;
  int CURRENT_MAP_PATH_INDEX;
  bool MCL_EXE;
};

// for Battery
#define KEY_BAT 1300
struct BAT {
	Status state;
  long long ts;
  double voltage;
};

// for DISPLAY
#define KEY_DISPLAY 1301
struct DISPLAY {
  double enc_x;
  double enc_y;
  double enc_a;
  double loc_x;
  double loc_y;
  double loc_a;
  double total_travel;
  double battery;
  double temp_driver_L;
  double temp_driver_R;
  double temp_motor_L;
  double temp_motor_R;
  int current_wp_index;
  int current_map_path_index;
  double v;
  double w;
  double min_obstacle_x;
  double min_obstacle_y;
};

// for LOG Display
const int LOG_HEIGHT = 10;
#define LOG_SIZE 1024
#define NUM_LOGS 1000
// 共有メモリのデータ構造
#define KEY_LOG 1302
struct LOG_DATA {
    sem_t sem;            // セマフォで排他制御
    size_t current_index; // 現在のログインデックス
    char logs[NUM_LOGS][LOG_SIZE]; // ログ用の固定サイズ配列
    int operation_flag;   // 操作フラグ（イベント駆動）
};
// ログの追加
void add_log(LOG_DATA *shared, const std::string& log) {
    sem_wait(&shared->sem); // セマフォで排他制御

    if (shared->current_index < NUM_LOGS) {
        strncpy(shared->logs[shared->current_index], log.c_str(), LOG_SIZE - 1);
        shared->logs[shared->current_index][LOG_SIZE - 1] = '\0';
        shared->current_index++;
    } else {
        //std::cerr << "ログがいっぱいです!" << std::endl;
        shared->current_index = 0;
    }

    sem_post(&shared->sem); // セマフォを解放
}

void draw_log_window(WINDOW* win, LOG_DATA *shared, int width, int height) {
    werase(win);
    box(win, 0, 0);

    // タイトル表示（カラー対応）
    if (has_colors()) {
        wattron(win, COLOR_PAIR(1));
        mvwprintw(win, 0, 2, " Log Window ");
        wattroff(win, COLOR_PAIR(1));
    } else {
        mvwprintw(win, 0, 2, " Log Window ");
    }

    //sem_wait(shared->sem); // セマフォで排他制御
    int max_lines = height - 2;
    int start = std::max(0, (int)shared->current_index - max_lines);

    for (int i = 0; i < max_lines; ++i) {
        if (i + start < (int)shared->current_index) {
            mvwprintw(win, i + 1, 2, "%-*s", width - 4, shared->logs[i + start]);
        } else {
            mvwprintw(win, i + 1, 2, "%-*s", width - 4, "");  // 空行でクリア
        }
    }
    //sem_post(shared->sem); // セマフォを解放

    wrefresh(win);
}

#endif // COYOMI_INCLUDE_SHM_BOARD_H
