#include "GetUrg3d.h"
#include "matrix.h"
//#include "Config.h"

#define HUGE_VALUE 9999999

#define MAXIMUM_RECORD_TIMES 10
#define MAXIMUM_CAPTURING_COUNT 10
#define INVALID_TIMESTAMP -1.0
//! scaling factor for angular velocity (500.0 * 0.001 / 32767) [deg/msec]
#define YVT_X002_ANGULAR_VELOCITY_SCALING_FACTOR 0.00001525925
//! scaling factor for acceleration (1 / 8192) [G]
#define YVT_X002_ACCELERATION_SCALING_FACTOR 0.00012207031
//! scaling factor for temperature (1.0 / 333.87) [degree]
#define YVT_X002_TEMPERATURE_SCALING_FACTOR 0.0029951776
//! shift value for temperature [degree]
#define YVT_X002_TEMPERATURE_SHIFT_VALUE 21.0

enum YVT_X002_AUXILIARY_DATA_INDEX {
  ROTATION_ANGULAR_VELOCITY_ABOUT_X_AXIS = 0,
  ROTATION_ANGULAR_VELOCITY_ABOUT_Y_AXIS,
  ROTATION_ANGULAR_VELOCITY_ABOUT_Z_AXIS,

  ACCELERATION_IN_X_DIRECTION,
  ACCELERATION_IN_Y_DIRECTION,
  ACCELERATION_IN_Z_DIRECTION,

  TEMPERATURE,

  NUMBER_OF_YVT_X002_AUXILIARY_DATA,
};

double yvt_sensor_ax_data_timestamps[MAXIMUM_CAPTURING_COUNT * MAXIMUM_RECORD_TIMES];
double yvt_sensor_scaled_ax_data[MAXIMUM_CAPTURING_COUNT * MAXIMUM_RECORD_TIMES][NUMBER_OF_YVT_X002_AUXILIARY_DATA];

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
      p.spot = spot;
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

static void clear_yvt_sensor_ax_data_buffer(void) {
  int data_index = 0;
  int data_type = 0;
  for (data_index = 0; data_index < MAXIMUM_CAPTURING_COUNT * MAXIMUM_RECORD_TIMES; ++data_index) {
    yvt_sensor_ax_data_timestamps[data_index] = INVALID_TIMESTAMP;

    for (data_type = 0; data_type < NUMBER_OF_YVT_X002_AUXILIARY_DATA; ++data_type) {
      yvt_sensor_scaled_ax_data[data_index][data_type] = 0.0;
    }
  }

  return;
}
static int is_yvt_x002_valid_auxiliary_data(const urg3d_auxiliary_data_t *raw_data) {
  if ((raw_data->type & URG3D_GYRO_DATA) &&
      (raw_data->type & URG3D_ACCEL_DATA) &&
      (raw_data->type & URG3D_TEMPERATURE_DATA)) {
    return 1;
  }
  return 0;
}
static void set_value_unit_of_yvt_x002(const urg3d_auxiliary_record_t *raw_data,
    double *timestamp_msec,
    double yvt_scaled_data[NUMBER_OF_YVT_X002_AUXILIARY_DATA]) {
  *timestamp_msec = (double)raw_data->timestamp_ms;
  yvt_scaled_data[ROTATION_ANGULAR_VELOCITY_ABOUT_X_AXIS] =
    YVT_X002_ANGULAR_VELOCITY_SCALING_FACTOR * (double)raw_data->gyro_x;
  yvt_scaled_data[ROTATION_ANGULAR_VELOCITY_ABOUT_Y_AXIS] =
    YVT_X002_ANGULAR_VELOCITY_SCALING_FACTOR * (double)raw_data->gyro_y;
  yvt_scaled_data[ROTATION_ANGULAR_VELOCITY_ABOUT_Z_AXIS] =
    YVT_X002_ANGULAR_VELOCITY_SCALING_FACTOR * (double)raw_data->gyro_z;

  yvt_scaled_data[ACCELERATION_IN_X_DIRECTION] =
    YVT_X002_ACCELERATION_SCALING_FACTOR * (double)raw_data->accel_x;
  yvt_scaled_data[ACCELERATION_IN_Y_DIRECTION] =
    YVT_X002_ACCELERATION_SCALING_FACTOR * (double)raw_data->accel_y;
  yvt_scaled_data[ACCELERATION_IN_Z_DIRECTION] =
    YVT_X002_ACCELERATION_SCALING_FACTOR * (double)raw_data->accel_z;
  yvt_scaled_data[TEMPERATURE] =
    YVT_X002_TEMPERATURE_SCALING_FACTOR * (double)raw_data->temperature + YVT_X002_TEMPERATURE_SHIFT_VALUE;
  return;
}
static void print_yvt_sensor_ax_data(const double* scaled_ax_data) {
#if 0
  printf("rotation angular velocity about x axis %3.1f [deg/msec]\n", scaled_ax_data[ROTATION_ANGULAR_VELOCITY_ABOUT_X_AXIS]);
  printf("rotation angular velocity about y axis %3.1f [deg/msec]\n", scaled_ax_data[ROTATION_ANGULAR_VELOCITY_ABOUT_Y_AXIS]);
  printf("rotation angular velocity about z axis %3.1f [deg/msec]\n", scaled_ax_data[ROTATION_ANGULAR_VELOCITY_ABOUT_Z_AXIS]);
  printf("temperature %3.1f [degree]\n", scaled_ax_data[TEMPERATURE]);
#endif
  double ax = scaled_ax_data[ACCELERATION_IN_Z_DIRECTION];
  double ay = scaled_ax_data[ACCELERATION_IN_X_DIRECTION];
  double az = scaled_ax_data[ACCELERATION_IN_Y_DIRECTION];
  printf("ax %3.3f [G]\t", ax);
  printf("ay %3.3f [G]\t", ay);
  printf("az %3.3f [G]\t", az);
  printf("theta %3.2f [deg]\t", atan2(ax, sqrt(ay*ay + az*az)) * 180/M_PI);
  printf("psi %3.2f [deg]\t",   atan2(ay, sqrt(ax*ax + az*az)) * 180/M_PI);
  printf("phi %3.2f [deg]\n",   atan2(sqrt(ax*ax + ay*ay), az) * 180/M_PI);

  std::ofstream ax_fout("ax_data.txt", std::ios_base::app);
  //ax_fout << "# Accel_X Accel_Y Accel_Z on coyomi local coordinate\n";
  ax_fout << ax << " " << ay << " " << az << " ";
  ax_fout
    << atan2(ax, sqrt(ay*ay + az*az)) << " "
    << atan2(ay, sqrt(ax*ax + az*az)) << " "
    << atan2(sqrt(ax*ax + ay*ay), az) << "\n";
  ax_fout.close();
  return;
}

static void print_valid_yvt_sensor_ax_data(void) {
  int data_index = 0;
  int data_type = 0;
  double timestamp;
  const double *scaled_ax_data = NULL;

  for (data_index = 0; data_index < MAXIMUM_CAPTURING_COUNT * MAXIMUM_RECORD_TIMES; ++data_index) {
    timestamp = yvt_sensor_ax_data_timestamps[data_index];
    scaled_ax_data = yvt_sensor_scaled_ax_data[data_index];
    if (timestamp != INVALID_TIMESTAMP) {
      //printf("%lf [sec]\n", timestamp / 1000.0);
      print_yvt_sensor_ax_data(scaled_ax_data);
    }
  }
  return;
}

void GetUrg3d::getAXData() {
  std::cout << "Get AX Data\n";
  int ret = 0;
  urg3d_auxiliary_data_t auxiliary_data;
  int record_index = 0;
  int capturing_record_count = 0;

  int capturing_count = 0;
  int capturing_data_index = 0;
  clear_yvt_sensor_ax_data_buffer();
  // start the acquisition mode URG3D_AUXILIARY
  if((ret = urg3d_high_start_data(&urg, URG3D_AUXILIARY)) < 0) {
    printf("error urg3d_high_start_data %d\n", ret);
    ret = urg3d_close(&urg);
#if defined(URG3D_MSC)
    getchar();
#endif
    return;
  }
  while(1) {
    if (capturing_count == MAXIMUM_CAPTURING_COUNT / 2) {
      //printf("%d capturing end\n", capturing_count);
    }
    if (capturing_count > MAXIMUM_CAPTURING_COUNT) {
      break;
    }
    // check receiving data
    if(urg3d_next_receive_ready(&urg)) {
      // parse ax data from receiving data
      if(urg3d_high_get_auxiliary_data(&urg, &auxiliary_data) > 0) {
        if (is_yvt_x002_valid_auxiliary_data(&auxiliary_data) > 0) {
          ++capturing_count;
          if (capturing_count > MAXIMUM_CAPTURING_COUNT) {
            break;
          }
          capturing_record_count = auxiliary_data.record_count;
          if (capturing_record_count > MAXIMUM_RECORD_TIMES) {
            capturing_record_count = MAXIMUM_RECORD_TIMES;
          }
          for (record_index = 0; record_index < capturing_record_count; ++record_index) {
            set_value_unit_of_yvt_x002(&auxiliary_data.records[record_index],
                &yvt_sensor_ax_data_timestamps[capturing_data_index],
                yvt_sensor_scaled_ax_data[capturing_data_index]);
            ++capturing_data_index;
          }
        }
      } else if(urg3d_low_get_binary(&urg, &header, data, &length_data) > 0) {
        // error check
        if(strncmp(header.type, "ERR", 3) == 0 || strncmp(header.type, "_er", 3) == 0) {
          printf("error %c%c%c %s", header.status[0], header.status[1], header.status[2], data);
          if(header.status[0] != '0'){
            break;
          }
        }
      }
    } else {
      usleep(10000);
    }
  }
  printf("capturing count is %d\n", capturing_data_index);
  print_valid_yvt_sensor_ax_data();
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
