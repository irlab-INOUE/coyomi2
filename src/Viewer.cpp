#include "Viewer.h"

Viewer::Viewer(MapPath path) {
  imgMap = cv::imread(path.occupancy_grid_map);
  AccMap = cv::imread(path.map_RGBA, -1);
  imgMap.copyTo(imgMap_original);
  AccMap.copyTo(AccMap_original);

	// mapInfo.yamlに接続する
	YAML::Node mapInfo_yaml;
	try {
		std::string path_to_yaml = path.path + std::string("/mapInfo.yaml");
		mapInfo_yaml = YAML::LoadFile(path_to_yaml);
    std::cerr << "mapInfo.yaml is open.\n";
	} catch(YAML::BadFile &e) {
		std::cerr << "read error! mapInfo.yaml is not exist."<< std::endl;
		exit(1);
	}

  csize = mapInfo_yaml["csize"].as<double>();
  originX = mapInfo_yaml["originX"].as<int>() + mapInfo_yaml["margin"].as<int>();
  originY = mapInfo_yaml["originY"].as<int>() + mapInfo_yaml["margin"].as<int>();

  cv::circle(imgMap, cv::Point(originX, originY), 0.5/csize, cv::Scalar(0, 0, 0), 1, cv::LINE_8);
}

double Viewer::get_csize() {
  return csize;
}

int Viewer::get_originX() {
  return originX;
}

int Viewer::get_originY() {
  return originY;
}

void Viewer::show() {
  cv::imshow("occMap", imgMap);
  cv::waitKey(0);
}

void Viewer::show(int time) {
  double scale = 1.0;
  double length;
  if (imgMap.rows > imgMap.cols)
    scale = imgMap.rows / 500.0;
  else
    scale = imgMap.cols / 650.0;
  cv::Mat dst;
  cv::Mat acc;
  cv::resize(imgMap, dst, cv::Size(imgMap.cols/scale, imgMap.rows/scale));
  //cv::resize(AccMap, acc, cv::Size(imgMap.cols/scale, imgMap.rows/scale));
  //cv::Mat dst = cv::Mat(imgMap, cv::Rect(0, 600, 600, 1200));
  cv::imshow("occMap", dst);
  //cv::imshow("AccMap", acc);
  cv::waitKey(time);
}

void Viewer::show(double x, double y, int time) {
  int wx = 1200;
  int wy = 1000;
  int cx = x / csize + originX;
  int cy =-y / csize + originY;
  int x0 = cx - wx/2;
  int y0 = cy - wy/2;
  int x1 = cx + wx/2;
  int y1 = cy + wy/2;

  if (x0 < 0) {
    x0 = 0;
    x1 = wx;
    if (x1 >= imgMap.cols) x1 = imgMap.cols - 1;
  }
  if (x1 >= imgMap.cols) {
    x1 = imgMap.cols - 1;
    x0 = x1 - wx;
    if (x0 < 0) x0 = 0;
  }
  if (y0 < 0) {
    y0 = 0;
    y1 = wy;
    if (y1 >= imgMap.rows) y1 = imgMap.rows - 1;
  }
  if (y1 >= imgMap.rows) {
    y1 = imgMap.rows - 1;
    y0 = y1 - wy;
    if (y0 < 0) y0 = 0;
  }

  wx = x1 - x0;
  wy = y1 - y0;
  cv::Mat dst = cv::Mat(imgMap, cv::Rect(x0, y0, wx, wy));
  cv::imshow("occMap", dst);
  cv::waitKey(time);
}
void Viewer::plot_wp(std::vector<WAYPOINT> wp_list) {
  for (auto wp: wp_list) {
    cv::circle(imgMap,
        cv::Point(originX + wp.x/csize, originY - wp.y/csize),
        0.5/csize, cv::Scalar(225, 105, 65), -1, cv::LINE_8);
    cv::circle(imgMap,
        cv::Point(originX + wp.x/csize, originY - wp.y/csize),
        1.0/csize, cv::Scalar(225, 105, 65), 2, cv::LINE_8);
  }
}

void Viewer::plot_current_wp(const WAYPOINT &wp) {
  cv::circle(imgMap,
      cv::Point(originX + wp.x/csize, originY - wp.y/csize),
      0.5/csize, cv::Scalar(0, 0, 200), -1, cv::LINE_8);
  cv::circle(imgMap,
      cv::Point(originX + wp.x/csize, originY - wp.y/csize),
      1.0/csize, cv::Scalar(0, 0, 200), 2, cv::LINE_8);
}

void Viewer::plot_subGoal(std::vector<WAYPOINT> sub_goal) {
  for (auto sg: sub_goal) {
    cv::circle(imgMap,
        cv::Point(originX + sg.x/csize, originY - sg.y/csize),
        0.15/csize, cv::Scalar(0, 200, 0), 1, cv::LINE_8);
  }
}

void Viewer::plot_smoothPath(std::vector<WAYPOINT> smooth_path) {
  for (auto pt: smooth_path) {
    cv::circle(imgMap,
        cv::Point(originX + pt.x/csize, originY - pt.y/csize),
        0.3/csize, cv::Scalar(200, 200, 0), -1, cv::LINE_8);
  }
}

void Viewer::robot(Pose2d pose) {
    cv::circle(imgMap,
        cv::Point(originX + pose.x/csize, originY - pose.y/csize),
        0.25/csize, cv::Scalar(0, 0, 255), -1, cv::LINE_8);
    cv::circle(AccMap,
        cv::Point(originX + pose.x/csize, originY - pose.y/csize),
        1.0/csize, cv::Scalar(0, 0, 255), -1, cv::LINE_8);
}

void Viewer::particle(const std::vector<Pose2d> &particle) {
  for (auto p: particle) {
    cv::circle(imgMap,
        cv::Point(originX + p.x/csize, originY - p.y/csize),
        0.2/csize, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
  }
}

void Viewer::line(Pose2d pose, WAYPOINT wp) {
  cv::line(imgMap,
      cv::Point(originX + pose.x/csize, originY - pose.y/csize),
      cv::Point(originX + wp.x/csize, originY - wp.y/csize),
      cv::Scalar(0, 255, 0), 1, cv::LINE_8);
}

void Viewer::hold() {
  imgMap.copyTo(imgMap_hold);
  AccMap.copyTo(AccMap_hold);
}

void Viewer::reset() {
  imgMap_hold.copyTo(imgMap);
  //AccMap_hold.copyTo(AccMap);
}

void Viewer::clear() {
  imgMap_original.copyTo(imgMap);
  AccMap_original.copyTo(AccMap);
  imgMap_original.copyTo(imgMap_hold);
  AccMap_original.copyTo(AccMap_hold);
}

cv::Mat Viewer::get_imgMap_original() {
  return imgMap_original;
}

void Viewer::urg(const Pose2d pose, const std::vector<LSP> &lsp) {
  double rx = pose.x;
  double ry = pose.y;
  double ra = pose.a;
  double cs = cos(ra);
  double sn = sin(ra);

  for (auto d: lsp) {
    if (d.data > 30000) continue;
    double x = (d.x * cs - d.y * sn) + rx;
    double y = (d.x * sn + d.y * cs) + ry;
    int ix = originX + x/csize;
    int iy = originY - y/csize;
    if (ix >= 0 && ix < imgMap.cols && iy >= 0 && iy < imgMap.rows) {
      cv::circle(imgMap,
          cv::Point(ix, iy), 2, cv::Scalar(255, 0, 0), -1, cv::LINE_8);
    }
  }
}
