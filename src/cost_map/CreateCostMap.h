#ifndef __CREATE_COST_MAP_H__
#define __CREATE_COST_MAP_H__

#include <fstream>
#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "../MapPath.h"

class CreateCostMap {
private:
  cv::Mat imgMap;
  cv::Mat imgMap_original;
  cv::Mat imgMap_hold;
  double csize;
  int originX, originY;
  void update_value(int iy, int ix, cv::Mat &imgMap, int filter);
  std::vector<std::vector<int>> filter;

public:
  CreateCostMap(MapPath path);
  ~CreateCostMap() {};
  double get_csize();
  int get_originX();
  int get_originY();
  cv::Mat get_imgMap_original();
  cv::Mat run();
};
#endif
