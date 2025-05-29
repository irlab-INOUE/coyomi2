#include "Urg2d.h"

Urg2d::Urg2d (const std::string lidar_select, double start_angle, double end_angle, double step_angle) {
  baseImg = cv::Mat(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, cv::Scalar(182, 182, 182));
  cv::line(baseImg, cv::Point(IMG_ORIGIN_X, 0), cv::Point(IMG_ORIGIN_X, baseImg.rows), cv::Scalar(0, 0, 0), 1);
  cv::line(baseImg, cv::Point(0, IMG_ORIGIN_Y), cv::Point(baseImg.cols, IMG_ORIGIN_Y), cv::Scalar(0, 0, 0), 1);

  double SCAN_START_ANGLE = start_angle;
  double SCAN_END_ANGLE =    end_angle;
  for (double ang = SCAN_START_ANGLE; ang <= SCAN_END_ANGLE; ang += step_angle) {
    ang_list.emplace_back(ang*M_PI/180);
    cos_list.emplace_back(cos(ang*M_PI/180));
    sin_list.emplace_back(sin(ang*M_PI/180));
  }

	// 接続
  qrk::Connection_information information(lidar_select);
  while (!urg.open(information.device_or_ip_name(),
                   information.baudrate_or_port_number(),
                   information.connection_type())) {
    // 実際のURGの接続に失敗した場合は終了する
    std::cerr << "Urg_driver::open(): " << information.device_or_ip_name()
              << ": " << urg.what() << std::endl;
    isConnectionSuccessful = false;
    //exit(EXIT_FAILURE);
	}

  if (isConnectionSuccessful) {
    //std::cout << information.device_or_ip_name() << " " << std::endl;
    //urg.wakeup();
    //while(!urg.is_stable()) {
    //  sleep(1);
    //}
    //std::cout << "LIDAR set up" << std::endl;
    // 計測範囲の設定
    urg.set_scanning_parameter(urg.deg2step(SCAN_START_ANGLE), urg.deg2step(SCAN_END_ANGLE), 0);
    // 計測開始命令を送信
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);
  }
}

Urg2d::~Urg2d() {
  //cv::destroyWindow("testImg");
  //urg.sleep();
}

bool Urg2d::getConnectionSuccessfully() {
  return isConnectionSuccessful;
}

std::vector<LSP> Urg2d::getData() {
  long time_stamp = 0;    // URGが発行するタイムスタンプの受け皿．使用していないが，ダミーで使う
	std::vector<long> data;
	//data.resize(1081);
  std::vector<LSP> result;
  while (!urg.get_distance(data,  &time_stamp)) {
    std::cerr << "Urg_driver::get_distance(): " << urg.what() << std::endl;
    sleep(1);
  }
  store_data.clear();
  for (int k = 0; k < data.size(); k++) {
    store_data.emplace_back(data[k], data[k]/1000.0, ang_list[k], cos_list[k], sin_list[k]);
  }
  return store_data;
}

std::vector<LSP> Urg2d::getData(
    const cv::Mat &imgMap_original, const Pose2d &pose, const int originX, const int originY,
    const double csize) {
  cv::Mat img;
  imgMap_original.copyTo(img);
  std::vector<LSP> data;
  for (int i = 0; i < ang_list.size(); i++) {
    for (double r = 0; r < 35; r += csize) {
      int cx = originX + (pose.x + r * cos(ang_list[i] + pose.a))/csize;
      int cy = originY - (pose.y + r * sin(ang_list[i] + pose.a))/csize;
      // (cx, cy)の画素値をチェック
      if (r < 35 && imgMap_original.at<cv::Vec3b>(cy, cx)[0] < 50) {
        data.emplace_back(r*1000, r, ang_list[i], cos_list[i], sin_list[i]);
        break;
      }
    }
  }

  for (auto lp: data) {
    int cx = originX + (pose.x + lp.x * cos(pose.a) - lp.y * sin(pose.a))/csize;
    int cy = originY - (pose.y + lp.x * sin(pose.a) + lp.y * cos(pose.a))/csize;
    cv::line(img,
        cv::Point(originX + pose.x/csize, originY - pose.y/csize),
        cv::Point(cx, cy), cv::Scalar(200, 80, 80), 1);
  }
  cv::imshow("TEST", img);
  cv::waitKey(5);
  return data;
}

int Urg2d::view(int wait_time) {
  cv::Mat testImg;
  baseImg.copyTo(testImg);
  int k = 0;
  for (auto d: store_data) {
    if (d.r < 35 && k % 10 == 0) {
      // std::cout << d.r * cos((-135 + 0.25*k)/180.0*M_PI) << " " << d.r * sin((-135 + 0.25*k)/180.0*M_PI) << "\n";
      // std::cout << d.x << " " << d.y << " ";
      int ix = IMG_ORIGIN_X - d.y / csize;
      int iy = IMG_ORIGIN_Y - d.x / csize;
      //std::cout << ix << " " << iy << "\n";
      if (ix >= 0 && ix < 600 && iy >= 0 && iy < 600) {
        cv::circle(testImg, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 0), -1);
      }
    }
    k++;
  }
  cv::imshow("testImg", testImg);
  return cv::waitKey(wait_time);
}

int Urg2d::view(const std::string lidar_select, int wait_time) {
  cv::Mat testImg;
  baseImg.copyTo(testImg);
  if (lidar_select == "t") {
    int k = 0;
    for (auto d: store_data) {
      if (d.r < 35 && k % 10 == 0) {
        int ix = IMG_ORIGIN_X + d.y / csize;
        int iy = IMG_ORIGIN_Y - d.x / csize;
        if (ix >= 0 && ix < 600 && iy >= 0 && iy < 600) {
          cv::circle(testImg, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 0), -1);
        }
      }
      k++;
    }
    cv::imshow("lidar_top", testImg);
  } else if (lidar_select == "b") {
    int k = 0;
    for (auto d: store_data) {
      if (d.r < 35 && k % 10 == 0) {
        int ix = IMG_ORIGIN_X - d.y / csize;
        int iy = IMG_ORIGIN_Y - d.x / csize;
        if (ix >= 0 && ix < 600 && iy >= 0 && iy < 600) {
          cv::circle(testImg, cv::Point(ix, iy), 1, cv::Scalar(0, 0, 0), -1);
        }
      }
      k++;
    }
    cv::imshow("lidar_bottom", testImg);
  }
  return cv::waitKey(wait_time);
}

void Urg2d::close() {
  urg.close();
}
