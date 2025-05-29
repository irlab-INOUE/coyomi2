#ifndef __SHARED_STRUCT_H__
#define __SHARED_STRUCT_H__
struct LOGDIR_PATH {
  std::string year;
  std::string mon;
  std::string mday;
  std::string hour;
  std::string min;
  std::string sec;
  std::string path;
};

struct DisplayContents {
  double v, w;
  double min_obstacle_x, min_obstacle_y;
};

// for Battery
struct BAT {
  long long ts;
  double voltage;
};

// for Localization
enum class ChangeMapTrigger {
	kContinue = 0,
	kChange = 1,
};
struct LOC {
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

// for Encoder receiver
struct ENC {
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
struct URG2D {
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

struct WP_LIST {
  // for ROUTE_LIST
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
  char path_to_wp_file[256]; 			// 現在使用中のWPファイルへのフルパス
  ChangeWPTrigger change_wp_trigger;	// WPのリセットトリガー
  int size_wp_list;
  int size_route_list;
  int target_index;
  ROUTE_POINT wp_list[3000]; 		// WAY POINT
  ROUTE_POINT route_list[3000]; 	// 細分化した通過点
  int wp_index_list[3000]; 		// 通過点が目指しているWAY POINTのインデックス
  bool get_ready;

  WP_LIST() {
    get_ready = false;
  }
};
#endif
