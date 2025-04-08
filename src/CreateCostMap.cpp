#include "CreateCostMap.h"

CreateCostMap::CreateCostMap(MapPath path) {
  imgMap = cv::imread(path.occupancy_grid_map);
  imgMap.copyTo(imgMap_original);

	// mapInfo.yamlに接続する
	YAML::Node mapInfo_yaml;
	try {
		std::string path_to_yaml = path.path + std::string("/mapInfo.yaml");
		mapInfo_yaml = YAML::LoadFile(path_to_yaml);
    //std::cerr << "mapInfo.yaml is open.\n";
	} catch(YAML::BadFile &e) {
		//std::cerr << "read error! mapInfo.yaml is not exist."<< std::endl;
		exit(1);
	}

  csize = mapInfo_yaml["csize"].as<double>();
  originX = mapInfo_yaml["originX"].as<int>() + mapInfo_yaml["margin"].as<int>();
  originY = mapInfo_yaml["originY"].as<int>() + mapInfo_yaml["margin"].as<int>();

  const size_t filter_size = 19;
  filter.resize(filter_size);
  for (int i = 0; i < filter.size(); i++) {
    filter[i].resize(filter_size);
  }
  int s = 100;
  int w = 50;
  int p = 30;
  int q = 10;
  filter[0]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, q, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  filter[1]  = {0, 0, 0, 0, 0, 0, 0, 0, q, q, q, 0, 0, 0, 0, 0, 0, 0, 0};
  filter[2]  = {0, 0, 0, 0, 0, 0, 0, q, q, q, q, q, 0, 0, 0, 0, 0, 0, 0};
  filter[3]  = {0, 0, 0, 0, 0, 0, q, q, q, q, q, q, q, 0, 0, 0, 0, 0, 0};
  filter[4]  = {0, 0, 0, 0, 0, q, q, q, q, q, q, q, q, q, 0, 0, 0, 0, 0};
  filter[5]  = {0, 0, 0, 0, q, q, q, q, q, q, q, q, q, q, q, 0, 0, 0, 0};
  filter[6]  = {0, 0, 0, q, q, q, q, q, q, p, q, q, q, q, q, q, 0, 0, 0};
  filter[7]  = {0, 0, q, q, q, q, q, q, p, w, p, q, q, q, q, q, q, 0, 0};
  filter[8]  = {0, q, q, q, q, q, q, p, w, s, w, p, q, q, q, q, q, q, 0};
  filter[9]  = {q, q, q, q, q, q, p, w, s, 0, s, w, p, q, q, q, q, q, q};
  filter[10] = {0, q, q, q, q, q, q, p, w, s, w, p, q, q, q, q, q, q, 0};
  filter[11] = {0, 0, q, q, q, q, q, q, p, w, p, q, q, q, q, q, q, 0, 0};
  filter[12] = {0, 0, 0, q, q, q, q, q, q, p, q, q, q, q, q, q, 0, 0, 0};
  filter[13] = {0, 0, 0, 0, q, q, q, q, q, q, q, q, q, q, q, 0, 0, 0, 0};
  filter[14] = {0, 0, 0, 0, 0, q, q, q, q, q, q, q, q, q, 0, 0, 0, 0, 0};
  filter[15] = {0, 0, 0, 0, 0, 0, q, q, q, q, q, q, q, 0, 0, 0, 0, 0, 0};
  filter[16] = {0, 0, 0, 0, 0, 0, 0, q, q, q, q, q, 0, 0, 0, 0, 0, 0, 0};
  filter[17] = {0, 0, 0, 0, 0, 0, 0, 0, q, q, q, 0, 0, 0, 0, 0, 0, 0, 0};
  filter[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, q, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}

double CreateCostMap::get_csize() {
  return csize;
}

int CreateCostMap::get_originX() {
  return originX;
}

int CreateCostMap::get_originY() {
  return originY;
}

void CreateCostMap::update_value(int iy, int ix, cv::Mat &imgMap, int filter) {
  int val = imgMap.at<cv::Vec3b>(iy, ix)[0];
  val -= filter;
  if (val < 0) val = 0;

  imgMap.at<cv::Vec3b>(iy, ix)[0] = val;
  imgMap.at<cv::Vec3b>(iy, ix)[1] = val;
  imgMap.at<cv::Vec3b>(iy, ix)[2] = val;
}

cv::Mat CreateCostMap::run() {
  for (int iy = filter.size()/2; iy < (imgMap.rows - filter.size()/2); iy++) {
    for (int ix = filter[0].size()/2; ix < (imgMap.cols - filter[0].size()/2); ix++) {
      cv::Vec3b color = imgMap_original.at<cv::Vec3b>(iy, ix);
      if (color[0] < 20) {
        for (int row = 0; row < filter.size(); row++) {
          for (int col = 0; col < filter[0].size(); col++) {
            update_value(iy + row - 9, ix + col - 9, imgMap, filter[row][col]);
          }
        }
      }
    }
  }
  return imgMap;
}

