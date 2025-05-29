#include "shared_struct.h"
#include "global_variable.h"

void thread_3D_Lidar(std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data) {
  std::string path = log_path->path + "/urg3dlog";

  bool is3DLidar_OK = true;
  std::string addr = "192.168.11.99";
  long port = 10904;
  GetUrg3d urg3d(addr, port);
  if(urg3d.initUrg3d() == -1) {
    add_log(log_data, "3D-Urg Open Error");
    is3DLidar_OK = false;
  }
  while (running.load()) {
    if (!is3DLidar_OK) {
      add_log(log_data, "3D-Urg Open Error");
      sleep_for(seconds(5));
      continue;
    }
    if (get3DLidarData.load()) {
      add_log(log_data, "3D LiDAR measured");
      std::vector<pointUrg3d> data;
      data = urg3d.get1Frame();
      if (data.size() > 0) {
        std::ofstream ofs;
        ofs.open(path);
        //ofs << "#x_m, #y_m, #z_m, #r_m, #theta, #phi, #intensity" << std::endl;
        for(int i=0; i<data.size(); i++){
          ofs << data[i].spot << " ";
          ofs << data[i].x << " " << -data[i].y << " " << -data[i].z << " ";
          ofs << data[i].r << " " << -data[i].phi << " " << -data[i].theta << " " << data[i].i;
          ofs << std::endl;
        }
      }
      get3DLidarData.store(false);
    }
    sleep_for(milliseconds(100));
  }
  std::cout << "3D_Lidar exit." << std::endl;
}

