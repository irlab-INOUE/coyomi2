/***********************************************************************
 * GetDistance3D
 *
 * 3DURGからのデータ取得
 *
 * Copyright (c) Kazumichi INOUE <k.inoue@oyama-ct.ac.jp>, 14/Aug./2020
 ***********************************************************************/
#include <chrono>
#include <fstream>
#include <string>
#include <signal.h>
#include <opencv2/opencv.hpp>
//#include <sys/shm.h>

//#include "shm_board.h"
#include "GetUrg3d.h"
//#include "Config.h"

using namespace std::chrono;

void sigcatch( int );
void lidar_view(std::vector<pointUrg3d> &data);

//URG3D *shm_urg3d = nullptr;

int main(int argc, char *argv[])
{
	std::string addr = "172.16.20.214";
	long port = 10904;

	/* Ctrl+c 対応 */
	if (SIG_ERR == signal( SIGINT, sigcatch )) {
		std::printf("failed to set signal handler\n");
		exit(1);
	}

  // 保存するディレクトリを共有メモリから得る
  // LOGDIR *shm_logdir = (LOGDIR *)shmAt(KEY_LOGDIR, sizeof(LOGDIR));

	// 保存するパスを作成する
  //std::string storeDir  = shm_logdir->path;
	//std::cerr << "保存先を" << storeDir + "/urg3dlog" << "に設定して開始します\nよろしければ, Enter を押してください" << std::endl;
	//getchar();


	// shm_board上のURG2D設定
	//shm_urg3d = (URG3D *)shmAt(KEY_URG3D, sizeof(URG3D));

	GetUrg3d urg3d(addr, port);
	if(urg3d.initUrg3d() == -1) {
		std::cerr << "3D-Urg Open Errorです" << std::endl;
		return -1;
	}

	//shm_urg3d->state = Status::Run;
	std::vector<pointUrg3d> data;
	for (;;) {
		auto time_now = high_resolution_clock::now(); 	// 現在日時を取得
		long long tp = duration_cast<microseconds>(time_now.time_since_epoch()).count(); // エポックからの経過時間をmsecで取得
    urg3d.getAXData();
		data = urg3d.get1Frame();
#if 0
		data = urg3d.rotateY(39.0/180.0*M_PI);
		data = urg3d.rotateX(2.0/180.8*M_PI);
		data = urg3d.transX(0.32);
#endif
		//Info inf = urg3d.info();

		//urg3d.savePointUrg3d_continuity(storeDir);

		//shm_urg3d->ts = tp;
    //shm_urg3d->size = data.size();
		//for (int i = 0; i < data.size(); i++) {
		//	shm_urg3d->pt[i].x         = data[i].x;
		//	shm_urg3d->pt[i].y         =-data[i].y;
		//	shm_urg3d->pt[i].z         =-data[i].z;
		//	shm_urg3d->pt[i].r         = data[i].r;
		//	shm_urg3d->pt[i].phi       = data[i].phi;
		//	shm_urg3d->pt[i].theta     = data[i].theta;
		//	shm_urg3d->pt[i].intensity = data[i].i;
		//}

		// このプロセスで直接ファイルへ書き出したい場合は以下を有効化する
#if 1
		std::ofstream ofs("test.txt");
		//ofs << "#x_m, #y_m, #z_m, #r_m, #theta, #phi, #intensity" << std::endl;

		for(int i=0; i<data.size(); i++){
			//std::cout << data[i].x << " " << data[i].y << " " << data[i].z << " ";
			//std::cout << data[i].r << " " << data[i].theta << " " << data[i].phi << " " << data[i].i;
			//std::cout << std::endl;
      ofs << data[i].spot << " ";
			ofs << data[i].x << " " << -data[i].y << " " << -data[i].z << " ";
			ofs << data[i].r << " " << -data[i].phi << " " << -data[i].theta << " " << data[i].i;
			ofs << std::endl;
		}
#endif
    lidar_view(data);
    return 0;
	}
	return 0;
}

void lidar_view(std::vector<pointUrg3d> &data) {
  int IMG_WIDTH = 800;
  int IMG_HEIGHT = 600;
  int IMG_ORIGIN_X = 400;
  int IMG_ORIGIN_Y = 500;
  double csize = 0.01;
  cv::Mat baseImg = cv::Mat(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, cv::Scalar(182, 182, 182));
  cv::line(baseImg,
      cv::Point(0, IMG_ORIGIN_Y), cv::Point(IMG_WIDTH, IMG_ORIGIN_Y),
      cv::Scalar(200, 0, 0), 1);
  cv::line(baseImg,
      cv::Point(IMG_ORIGIN_X, 0), cv::Point(IMG_ORIGIN_X, IMG_HEIGHT),
      cv::Scalar(200, 0, 0), 1);
  double th = 45.0*M_PI/180.0;
  double cs = cos(th);
  double sn = sin(th);
  for (auto d: data) {
    double vx = cs * d.x + sn * d.z;
    double vy = d.y;
    double vz = -sn * d.x + cs * d.z;
    int iy =-vx / csize + IMG_ORIGIN_Y;
    int ix = vy / csize + IMG_ORIGIN_X;
    if (ix >= 0 && ix < baseImg.cols && iy >= 0 && iy < baseImg.rows) {
      baseImg.at<cv::Vec3b>(iy, ix) = cv::Vec3b(100, 100, 0);
    }
  }
  cv::imshow("TEST", baseImg);
  cv::waitKey();
}

void sigcatch(int sig)
{
	std::printf("Catch signal %d\n", sig);
	//shm_urg3d->state = Status::Ready;
	exit(1);
}
