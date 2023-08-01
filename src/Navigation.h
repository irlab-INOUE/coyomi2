#pragma onece

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "TaskStatus.h"
#include "MapPath.h"
#include "WAYPOINT.h"
#include "Viewer.h"
#include "Config.h"
#include "DWA.h"
#include "Urg2d.h"
#include "MCL.h"

class Navigation {
  private:
    std::vector<MapPath> map_path;
    TaskState navi_state;
    DynamicWindowApproach dwa;
    double arrived_check_distance;
  public:
    Navigation();
    Navigation(YAML::Node &coyomi_yaml);
    void start();
    TaskState state();
};
