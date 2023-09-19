#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "yaml-cpp/yaml.h"
#include "CreateCostMap.h"
#include "../MapPath.h"

int main(int argc, char *argv[]) {
  std::string MAP_PATH = "../../../map/log230908_2F";
  std::string OCC_NAME = "occMap.png";
  std::string map_name = MAP_PATH + "/" + OCC_NAME;
  MapPath map_path(MAP_PATH, map_name, "","","lfm.txt", "mapInfo.yaml", 0, 0, 0);

  CreateCostMap ccm(map_path);
  cv::Mat imgMap_cost = ccm.run();

  cv::imshow("occMap_cost", imgMap_cost);
  cv::imwrite("occMap_cost.png", imgMap_cost);
  cv::waitKey();

  return 0;
}
