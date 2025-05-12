#include "shared_struct.h"
#include "global_variable.h"

void thread_sound(std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data,
    std::shared_ptr<DisplayContents> disp, std::shared_ptr<ENC> enc) {
  int prev_wp_index = 0;
  std::string sound_logfile_path = log_path->path + "/sound_log";
  std::ofstream ofs;
  ofs.open(sound_logfile_path);
  while (running.load()) {
    long long ts = get_current_time();
    ofs << ts << " " << prev_wp_index << " " << enc->current_wp_index << "\n";
    if (prev_wp_index != enc->current_wp_index) {
      std::string cmd = "paplay /usr/share/sounds/freedesktop/stereo/complete.oga";
      int ret = std::system(cmd.c_str());
      prev_wp_index = disp->current_wp_index;
      // WP更新をログメッセージ出す
      std::string log_text = "WP updated. Next target->" + std::to_string(disp->current_wp_index);
      add_log(log_data, log_text);
    }
    std::string log_text = "Running process. TimeStamp-> " + std::to_string(ts);
    add_log(log_data, log_text);
    sleep_for(seconds(1));
  }
  std::cout << "Sound log exit." << std::endl;
}

