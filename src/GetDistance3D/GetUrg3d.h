#ifndef __GETURG3D_H__
#define __GETURG3D_H__

#include<iostream>
#include<fstream>
#include<sstream>
#include<chrono>
#include<iomanip>
#include<math.h>
#include<sys/stat.h>
#include<vector>
#include<unistd.h>
#include<string.h>
#include <chrono>

#include<urg3d_sensor.h>

typedef struct { 

  double x;
  double y;
  double z;
  double r;       // range
  double phi;     // vertical rad
  double theta;   // horizontal rad
  double i;       // refrection intensity
} pointUrg3d; 

struct Info{
  double Xmax;
  double Xmin;
  double Ymax;
  double Ymin;
  double Zmax;
  double Zmin;
};

class GetUrg3d{

  private:
    urg3d_t urg;
    urg3d_vssp_header_t header;
    urg3d_measurement_data_t measurement_data;
    char data[URG3D_MAX_RX_LENGTH];
    int length_data;
    int buffer_length = 0;
    char buffer[2048*2048]; //4MB

    std::string device = "192.16.0.10"; /* device ip address */
    long port = 10940; /* device port number. It is a fixed value */

    std::vector<pointUrg3d> data1Frame;

    int numOfInterlaceH = 4;
    int numOfInterlaceV = 8;

    void addXYZ_toBuf(const urg3d_t* urg, const urg3d_measurement_data_t* measurement_data);

    std::string getTodayDate();

    std::string path_to_continuous_store = "";  // 時系列データの連続保存ファイル名．初回に一回だけ作成される．

    long long ts = 0;    // タイムスタンプの初期値をシフト用

  public:
    GetUrg3d();
    GetUrg3d(std::string adr, long port);
    void setUrg3dAdr(std::string adr, long port);
    int initUrg3d();
    std::vector<pointUrg3d> get1Frame();
    std::vector<pointUrg3d> get1Frame_file(std::string path="");
    void savePointUrg3d(std::string path = "");
    void savePointUrg3d_continuity(std::string path = "");  // 時系列データをタイムスタンプ区切り行を入れて一つのファイルに格納する

    std::vector<pointUrg3d> rotateY(double p);
    std::vector<pointUrg3d> rotateX(double p);
    std::vector<pointUrg3d> transX(double p);           // X方向へ平行移動

    Info info();                                        // 測定データのレンジなどの情報を出力する

};
#endif
