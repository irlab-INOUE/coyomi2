#include "shared_struct.h"
#include "global_variable.h"

void thread_battery_logger(std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data,
    std::shared_ptr<DisplayContents> disp, std::shared_ptr<BAT> bat) {
  std::string path = log_path->path + "/batlog";
  std::ofstream bat_log;
  bat_log.open(path);
  while (running.load()) {
    bat->ts = get_current_time();
    bat_log << bat->ts << " " << bat->voltage << "\n";
    disp->battery = bat->voltage;
    sleep_for(seconds(1));
  }
  std::cout << "Battery logger2 exit." << std::endl;
}

