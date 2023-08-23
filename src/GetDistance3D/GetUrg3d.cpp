#include "GetUrg3d.h"
#include "matrix.h"
//#include "Config.h"

#define HUGE_VALUE 9999999

GetUrg3d::GetUrg3d(){
}


GetUrg3d::GetUrg3d(std::string adr, long port){

  device = adr;
  port   = port;

  // タイムスタンプの初期値読み込み
  std::ifstream inFile;
  inFile.open("/tmp/timeStart.txt");
  inFile >> ts;
}


void GetUrg3d::setUrg3dAdr(std::string adr, long port){

  device = adr;
  port   = port;

  // タイムスタンプの初期値読み込み
  std::ifstream inFile;
  inFile.open("/tmp/timeStart.txt");
  inFile >> ts;
}

std::string GetUrg3d::getTodayDate(){

  //現在日時を取得する
  time_t t = time(nullptr);

  //形式を変換する
  const tm* lt = localtime(&t);

  //sに独自フォーマットになるように連結していく
  std::stringstream s;
  s <<"20";
  s << lt->tm_year-100; //100を引くことで20xxのxxの部分になる
  s << std::setw(2) << std::setfill('0') << lt->tm_mon+1;
  s << std::setw(2) << std::setfill('0') << lt->tm_mday;
  s << "_";
  s << std::setw(2) << std::setfill('0') << lt->tm_hour;
  s << std::setw(2) << std::setfill('0') << lt->tm_min;

  std::string result = s.str();
  return result;

}


void GetUrg3d::addXYZ_toBuf(const urg3d_t* urg, const urg3d_measurement_data_t* measurement_data){

  const int maxOfIntensityAvg = 2000;
  pointUrg3d p;

  for(int spot=0; spot < measurement_data->spot_count; ++spot) {
    for(int echo=0; echo < measurement_data->spots[spot].echo_count; ++echo) {

      p.x     = static_cast<double>(measurement_data->spots[spot].point[echo].x_m);
      p.y     = static_cast<double>(measurement_data->spots[spot].point[echo].y_m);
      p.z     = static_cast<double>(measurement_data->spots[spot].point[echo].z_m);
      p.r     = static_cast<double>(measurement_data->spots[spot].polar[echo].range_m);
      p.phi   = static_cast<double>(measurement_data->spots[spot].polar[echo].vertical_rad);
      p.theta = static_cast<double>(measurement_data->spots[spot].polar[echo].horizontal_rad);
      p.i     = static_cast<double>(measurement_data->spots[spot].polar[echo].intensity);

      data1Frame.push_back(p);
    }
  }
}


int GetUrg3d::initUrg3d(){

  int ret;

  //open the connection to the 3d urg
  if((ret=urg3d_open(&urg, device.c_str(), port)) <0){

    std::cout << "Error urg3d_open" << ret << std::endl;
    return -1;
  }
  else{
    std::cout << "Open OK" << std::endl;
  }

  //set blocking function timeout (3000ms)
  urg3d_high_set_blocking_timeout_ms(&urg, 3000);


  //initialize the urg3d session (get transform tables)
  if((ret = urg3d_high_blocking_init(&urg)) < 0) {
    printf("error urg3d_high_blocking_init %d\n", ret);
    ret = urg3d_close(&urg);

    return -1;
  }


  //send interlace count for device (2 h-field / v-field )
  if((ret = urg3d_high_blocking_set_horizontal_interlace_count(&urg, numOfInterlaceH)) < 0) {
    printf("error urg3d_high_blocking_set_horizontal_interlace_count %d\n", ret);
    ret = urg3d_close(&urg);

    return -1;
  }


  //send interlace count for device (4 v-field / frame )
  if((ret = urg3d_high_blocking_set_vertical_interlace_count(&urg, numOfInterlaceV)) < 0) {
    printf("error urg3d_high_blocking_set_vertical_interlace_count %d\n", ret);
    ret = urg3d_close(&urg);

    return -1;
  }


  //start the acquisition mode. possible value are:
  if((ret = urg3d_high_start_data(&urg, URG3D_DISTANCE_INTENSITY)) < 0) {
    printf("error send start %d\n", ret);
    ret = urg3d_close(&urg);

    return -1;
  }
  return 0;
}


std::vector<pointUrg3d> GetUrg3d::get1Frame(){

  int ret = 0; /* operation return */
  int prev_frame = -1;
  FILE *fp;
  data1Frame.clear();

  while(1) {

    //check received data
    if(urg3d_next_receive_ready(&urg)) {

      //pick up the data (non-blocking)
      if(urg3d_high_get_measurement_data(&urg, &measurement_data) > 0) {

        //wait for first data
        if(prev_frame == -1) {

          //check line and field number
          if(measurement_data.line_number == 0 && measurement_data.v_field_number == 0 && measurement_data.h_field_number == 0) {
            prev_frame = measurement_data.frame_number;
          }
        }

        //start frame data
        if(prev_frame != -1) {

          //if frame number is changed, break the loop
          if(prev_frame != measurement_data.frame_number) {
            break;
          }

          //add data for buffer
          addXYZ_toBuf(&urg, &measurement_data);
        }
      }else if(urg3d_low_get_binary(&urg, &header, data, &length_data) > 0) {

        //check error data
        if(strncmp(header.type, "ERR", 3) == 0 || strncmp(header.type, "_er", 3) == 0) {
          printf("error %c%c%c %s", header.status[0], header.status[1], header.status[2], data);
          if(header.status[0] != '0'){
            break;
          }
        }
      }
    }else{
      usleep(10000);
    }
  }

  return data1Frame;
}


std::vector<pointUrg3d> GetUrg3d::get1Frame_file(std::string path){

  std::ifstream ifs(path);
  std::cout << "sensor data file : " << path << "\n" << std::endl;

  std::vector<pointUrg3d> data1frame;

  if(ifs){
    std::string str;
    while(std::getline(ifs, str)){
      std::stringstream ss;
      ss << str;

      while(!ss.eof()){
        if(str[0]=='#'){    //skip comment
          break;
        }
        pointUrg3d p;
        ss >> p.x;
        ss >> p.y;
        ss >> p.z;
        ss >> p.r;
        ss >> p.phi;
        ss >> p.theta;
        ss >> p.i;

        data1frame.push_back(p);
      }
    }
  }
  else{
    std::cout << "File Open Error" << std::endl;
  }
  return data1frame;
}


void GetUrg3d::savePointUrg3d(std::string path)
{
  // 保存ファイル名を固定する
  std::string file_name = "3durglog";

  // 保存ファイル名を日付にする
  //std::string file_name = getTodayDate() + ".txt";

  path += file_name;

  std::cerr << path << std::endl;
  std::ofstream ofs(path, std::ios_base::app);
  if(ofs){
    ofs << "#x_m, #y_m, #z_m, #r_m, #phi, #theta, #intensity" << std::endl;
    for(int i=0; i<data1Frame.size(); i++){
      ofs << data1Frame[i].x << " " << data1Frame[i].y  << " " << data1Frame[i].z  << " "
        << data1Frame[i].r << " " << data1Frame[i].phi << " " << data1Frame[i].theta << " "
        << data1Frame[i].i << std::endl;
    }
  }
  else{
    std::cout << "output file open error" << std::endl;
  }
}

void GetUrg3d::savePointUrg3d_continuity(std::string path) {
#if 0
  // 時系列データをタイムスタンプ区切り行を入れて一つのファイルに格納する
  path += path_to_continuous_store;
  if (path_to_continuous_store == "") {
    path_to_continuous_store = getTodayDate() + ".txt";
    path += path_to_continuous_store;
  }
#else
  // 保存ファイル名を固定する
  std::string file_name = "/urg3dlog";
  path += file_name;
#endif

  std::chrono::system_clock::time_point current_time;
  current_time = std::chrono::system_clock::now();
  long long timestamp = (current_time.time_since_epoch().count() - ts) / 1000000;

  std::ofstream ofs(path, std::ios_base::app);
  if(ofs){
    // [識別子] [タイムスタンプ] [データ行数] [x y a] xyaはロボット姿勢（URG ではない）．何も無ければローカル座標系なので0,0,0
    ofs << "LASERSCAN3D" << " " << timestamp << " " << data1Frame.size() << " 0.0 0.0 0.0" << "\n";
    //ofs << "#x_m, #y_m, #z_m, #r_m, #theta, #phi, #intensity" << std::endl;
    for(int i=0; i<data1Frame.size(); i++){
      ofs << data1Frame[i].x << " " << data1Frame[i].y  << " " << data1Frame[i].z  << " "
        << data1Frame[i].r << " " << data1Frame[i].phi << " " << data1Frame[i].theta << " " << data1Frame[i].i << std::endl;
    }
  }
  else{
    std::cerr << "output file open error" << std::endl;
  }
}

std::vector<pointUrg3d> GetUrg3d::rotateY(double p){
  Matrix R(3,3);
  R <<
    cos(p), 0, sin(p),
    0, 1,      0,
    -sin(p), 0, cos(p);

  Matrix pt(3,1);
  for (int i = 0; i < data1Frame.size(); i++) {
    pt << data1Frame[i].x, data1Frame[i].y, data1Frame[i].z;
    Matrix result = R * pt;
    data1Frame[i].x = result(0,0);
    data1Frame[i].y = result(1,0);
    data1Frame[i].z = result(2,0);
    data1Frame[i].phi = atan2(data1Frame[i].z, std::hypot(data1Frame[i].x, data1Frame[i].y));
    data1Frame[i].theta = atan2(data1Frame[i].y, data1Frame[i].x);
  }

  return data1Frame;
}

std::vector<pointUrg3d> GetUrg3d::rotateX(double p){
  Matrix R(3,3);
  R <<
    1,      0,      0,
    0, cos(p),-sin(p),
    0, sin(p), cos(p);

  Matrix pt(3,1);
  for (int i = 0; i < data1Frame.size(); i++) {
    pt << data1Frame[i].x, data1Frame[i].y, data1Frame[i].z;
    Matrix result = R * pt;
    data1Frame[i].x = result(0,0);
    data1Frame[i].y = result(1,0);
    data1Frame[i].z = result(2,0);
    data1Frame[i].phi = atan2(data1Frame[i].z, std::hypot(data1Frame[i].x, data1Frame[i].y));
    data1Frame[i].theta = atan2(data1Frame[i].y, data1Frame[i].x);
  }

  return data1Frame;
}

std::vector<pointUrg3d> GetUrg3d::transX(double p){
  for (int i = 0; i < data1Frame.size(); i++) {
    data1Frame[i].x += p;
  }

  return data1Frame;
}

Info GetUrg3d::info(){
  Info tmp;
  tmp.Xmax = -HUGE_VALUE;
  tmp.Xmin =  HUGE_VALUE;
  tmp.Ymax = -HUGE_VALUE;
  tmp.Ymin =  HUGE_VALUE;
  tmp.Zmax = -HUGE_VALUE;
  tmp.Zmin =  HUGE_VALUE;
  for (int i = 0; i < data1Frame.size(); i++) {
    if (tmp.Xmax < data1Frame[i].x) tmp.Xmax = data1Frame[i].x;
    if (tmp.Xmin > data1Frame[i].x) tmp.Xmin = data1Frame[i].x;
    if (tmp.Ymax < data1Frame[i].y) tmp.Ymax = data1Frame[i].y;
    if (tmp.Ymin > data1Frame[i].y) tmp.Ymin = data1Frame[i].y;
    if (tmp.Zmax < data1Frame[i].z) tmp.Zmax = data1Frame[i].z;
    if (tmp.Zmin > data1Frame[i].z) tmp.Zmin = data1Frame[i].z;
  }
  std::cerr << "=====<INFO>=====\n"
    << "Xmin:" << tmp.Xmin << "\t" << "Xmax:" << tmp.Xmax << "\n"
    << "Ymin:" << tmp.Ymin << "\t" << "Ymax:" << tmp.Ymax << "\n"
    << "Zmin:" << tmp.Zmin << "\t" << "Zmax:" << tmp.Zmax << "\n";
  return tmp;
}
