#include "shared_struct.h"
#include "global_variable.h"
#include "common.h"
#include "Urg2d.h"

void thread_2D_Lidar_b(const std::string lidar_select, std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data, std::shared_ptr<URG2D> urg2d) {
  // coyomi_yamlをこのスレッド内で新しく取得する
  std::string path_to_yaml = DEFAULT_ROOT + std::string("/coyomi.yaml");
  YAML::Node coyomi_yaml = yamlRead(path_to_yaml);
  add_log(log_data, "coyomi.yaml is open in thread_2D_Lidar_" + lidar_select + ".");

  urg2d->start_angle   = coyomi_yaml["2DLIDAR"]["start_angle"].as<double>();
  urg2d->end_angle     = coyomi_yaml["2DLIDAR"]["end_angle"].as<double>();
  urg2d->step_angle    = coyomi_yaml["2DLIDAR"]["step_angle"].as<double>();
  urg2d->max_echo_size = coyomi_yaml["2DLIDAR"]["max_echo_size"].as<double>();
  urg2d->size =
    ((urg2d->end_angle - urg2d->start_angle)/urg2d->step_angle + 1) * urg2d->max_echo_size;
  for (int i = 0; i < urg2d->size; i++) {
    urg2d->r[i] = 0;
  }
  for (int i = 0; i < ((urg2d->end_angle - urg2d->start_angle)/urg2d->step_angle + 1); i++) {
    double ang = (i * urg2d->step_angle + urg2d->start_angle)*M_PI/180;
    urg2d->ang[i] = ang;
    urg2d->cs[i] = cos(ang);
    urg2d->sn[i] = sin(ang);
  }
  std::string path = log_path->path + "/urglog_" + lidar_select;

  Urg2d urg2d_b(lidar_select, urg2d->start_angle, urg2d->end_angle, urg2d->step_angle);
  // urgのopen可否を受け取る
  if(urg2d_b.getConnectionSuccessfully() == false) {
    while (running.load()) {
      add_log(log_data, "2D-Urg Open Error");
      sleep_for(seconds(5));
    }
  } else {
    while (running.load()) {
      fout_urg2d.open(path, std::ios_base::app);
      long long ts = get_current_time();
      std::vector<LSP> result = urg2d_b.getData();
      urg2d_b.view(lidar_select, 5);
      fout_urg2d << "LASERSCANRT" << " "
        << ts << " "
        << static_cast<int>(result.size()) * urg2d->max_echo_size << " "
        << std::to_string(urg2d->start_angle) << " "
        << std::to_string(urg2d->end_angle) << " "
        << std::to_string(urg2d->step_angle) << " "
        << urg2d->max_echo_size << " ";
      for (auto d: result) {
        fout_urg2d << d.data << " " << "0" << " " << "0" << " ";
      }
      fout_urg2d << ts << "\n";
      fout_urg2d.close();

      urg2d->ts = ts;
      urg2d->ts_end = ts;
      urg2d->size = result.size();
      for (int k = 0; k < result.size(); k++) {
        urg2d->r[k] = result[k].data;
      }
      sleep_for(milliseconds(100));
    }
    urg2d_b.close();
  }
  std::cout << "2D_Lidar_" << lidar_select << " exit." << std::endl;
}

