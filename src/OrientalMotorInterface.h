#ifndef __ORIENTAL_MOTOR_INTERFACE_H__
#define __ORIENTAL_MOTOR_INTERFACE_H__
#include "query.h"
#include "Config.h"
#include "shm_board.h"

struct ODOMETORY {
  double dist_R;
  double dist_L;
  double travel;
  double rotation;
  double rx;
  double ry;
  double ra;
  ODOMETORY() {
    dist_R = 0.0;
    dist_L = 0.0;
    travel = 0.0;
    rotation = 0.0;
    rx = 0.0;
    ry = 0.0;
    ra = 0.0;
  }
};

void read_res(uint8_t *buf, int length);
void show_state(uint8_t *buf, const long long &ts);
void read_odo(uint8_t *buf, ODOMETORY &odo);

// CRC create
void calcBcc(uint8_t *sendData, int length) {
  unsigned int crcH, crcL;
  int crc=0xFFFF;
  for (int no = 0; no < length-2; no++) {
    crc = crc ^ sendData[no];
    for (int i = 0; i < 8; i++) {
      if (1 == crc % 2) {
        crc = crc >> 1;
        crc = 0xA001 ^ crc;
      } else {
        crc = crc >> 1;
      }
    }
  }
  crcL = (crc & 255);
  crcH = (crc >> 8 & 255);
  sendData[length-2] = static_cast<unsigned char>(crcL);
  sendData[length-1] = static_cast<unsigned char>(crcH);
}

void send_cmd(uint8_t *cmd, int length) {
  calcBcc(cmd, length);
#ifdef DEBUG_SENDRESP
  std::cerr << "[SEND]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd[i]) << " ";
  }
  std::cerr << "\n";
#endif
  int n = write(fd_motor, cmd, length);
  usleep(SERIAL_INTERVAL_SEND);
}

void simple_send_cmd(uint8_t *cmd, int length) {
  send_cmd(cmd, length);
  uint8_t res_buf[MAX_BUFFER_SIZE];
  read_res(res_buf, 8);
}

int ReadByte(uint8_t *buf) {
  return read(fd_motor, buf, sizeof(uint8_t));
}

void read_res(uint8_t *buf2, int length) {
  int tries = 3;
  int tmp_len = 0;
  uint8_t buf[1];
  while (tries) {
    if(ReadByte(buf)) {
      buf2[tmp_len++] = buf[0];
      if (tmp_len >= length) {
        break;
      }
    } else {
      tries++;
    }
  }
  usleep(SERIAL_INTERVAL_RESP);
}

void turn_on_motors() {
  //std::cerr << "\033[12;1H" << "Turn ON RL...";
  simple_send_cmd(Query_Write_Son_R, sizeof(Query_Write_Son_R));
  simple_send_cmd(Query_Write_Son_L, sizeof(Query_Write_Son_L));
  //std::cerr << "Done.\n";
}

void turn_off_motors() {
  //std::cerr << "\033[12;1H" << "Turn OFF RL...";
  simple_send_cmd(Query_Write_Soff_R, sizeof(Query_Write_Soff_R));
  simple_send_cmd(Query_Write_Soff_L, sizeof(Query_Write_Soff_L));
  //std::cerr << "Done.\n";
}

void free_motors() {
  //std::cerr << "\033[12;1H" << "FREE RL...";
  simple_send_cmd(Query_Write_FREE_R, sizeof(Query_Write_FREE_R));
  simple_send_cmd(Query_Write_FREE_L, sizeof(Query_Write_FREE_L));
  //std::cerr << "Done.\n";
}

inline int32_t circular_diff32(int32_t curr, int32_t prev) {
    return static_cast<int32_t>(static_cast<uint32_t>(curr) - static_cast<uint32_t>(prev));
}
void read_state(ODOMETORY &odo, const long long &ts) {
  uint8_t buf[MAX_BUFFER_SIZE];
  //send_cmd(Query_NET_ID_READ_ODO, sizeof(Query_NET_ID_READ_ODO));
  //read_res(buf, 17);
  send_cmd(Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(buf, 57);
#if 0
  std::cerr << "\033[11A" << "Read state\n";
  show_state(buf, ts);
#endif
  //read_odo(buf, odo);

  const int OFFSET = 26;
  int alarm_code_R     = static_cast<int>(buf[ 3] << 24 | buf[ 4] << 16 | buf[ 5] << 8 | buf[ 6]);
  double temp_driver_R = static_cast<int>(buf[ 7] << 24 | buf[ 8] << 16 | buf[ 9] << 8 | buf[10]) * 0.1;
  double temp_motor_R  = static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]) * 0.1;
  //int position_R       = static_cast<int>(buf[15] << 24 | buf[16] << 16 | buf[17] << 8 | buf[18]);
  int32_t position_R       = static_cast<int32_t>((static_cast<uint32_t>(buf[15]) << 24) |
                                          (static_cast<uint32_t>(buf[16]) << 16) |
                                          (static_cast<uint32_t>(buf[17]) << 8)  |
                                          (static_cast<uint32_t>(buf[18])));
  int power_R          = static_cast<int>(buf[19] << 24 | buf[20] << 16 | buf[21] << 8 | buf[22]);
  double voltage_R     = static_cast<int>(buf[23] << 24 | buf[24] << 16 | buf[25] << 8 | buf[26]) * 0.1;

  int alarm_code_L     = static_cast<int>(buf[ 3 + OFFSET] << 24 | buf[ 4 + OFFSET] << 16 | buf[ 5 + OFFSET] << 8 | buf[ 6 + OFFSET]);
  double temp_driver_L = static_cast<int>(buf[ 7 + OFFSET] << 24 | buf[ 8 + OFFSET] << 16 | buf[ 9 + OFFSET] << 8 | buf[10 + OFFSET]) * 0.1;
  double temp_motor_L  = static_cast<int>(buf[11 + OFFSET] << 24 | buf[12 + OFFSET] << 16 | buf[13 + OFFSET] << 8 | buf[14 + OFFSET]) * 0.1;
  //int position_L       = static_cast<uint32_t>(buf[15 + OFFSET] << 24 | buf[16 + OFFSET] << 16 | buf[17 + OFFSET] << 8 | buf[18 + OFFSET]);
  int32_t position_L       = static_cast<int32_t>((static_cast<uint32_t>(buf[15+OFFSET]) << 24) |
                                          (static_cast<uint32_t>(buf[16+OFFSET]) << 16) |
                                          (static_cast<uint32_t>(buf[17+OFFSET]) << 8)  |
                                          (static_cast<uint32_t>(buf[18+OFFSET])));
  int power_L          = static_cast<int>(buf[19 + OFFSET] << 24 | buf[20 + OFFSET] << 16 | buf[21 + OFFSET] << 8 | buf[22 + OFFSET]);
  double voltage_L     = static_cast<int>(buf[23 + OFFSET] << 24 | buf[24 + OFFSET] << 16 | buf[25 + OFFSET] << 8 | buf[26 + OFFSET]) * 0.1;

  double dist_L   =  position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R   = -position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel   = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;
  double voltage  = (voltage_L + voltage_R)/2.0;

  enc->total_travel = travel;
  enc->battery = voltage;
  bat->voltage = voltage;
  enc->temp_driver_R = temp_driver_R;
  enc->temp_motor_R  = temp_motor_R;
  enc->temp_driver_L = temp_driver_L;
  enc->temp_motor_L  = temp_motor_L;

  double dl = travel - odo.travel;
  double dth = rotation - odo.rotation;
  odo.rx += dl * cos(odo.ra);
  odo.ry += dl * sin(odo.ra);
  odo.ra += dth;
  if (odo.ra > M_PI) odo.ra -= 2*M_PI;
  else if (odo.ra < -M_PI) odo.ra += 2*M_PI;
  odo.dist_R = dist_R;
  odo.dist_L = dist_L;
  odo.travel = travel;
  odo.rotation = rotation;
}

void calc_vw2hex(uint8_t *Query_NET_ID_WRITE, double v, double w) {
  double wr = v/(WHEEL_D/2) + w*WHEEL_T/(1.0*WHEEL_D);
  double wl = v/(WHEEL_D/2) - w*WHEEL_T/(1.0*WHEEL_D);
  double motor_wr_rpm =-wr / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  double motor_wl_rpm = wl / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  Query_NET_ID_WRITE[15] = (static_cast<int>(motor_wr_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[16] = (static_cast<int>(motor_wr_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[17] = (static_cast<int>(motor_wr_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[18] =  static_cast<int>(motor_wr_rpm)        & 0xFF;
  Query_NET_ID_WRITE[39] = (static_cast<int>(motor_wl_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[40] = (static_cast<int>(motor_wl_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[41] = (static_cast<int>(motor_wl_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[42] =  static_cast<int>(motor_wl_rpm)        & 0xFF;
#if 0
  for (int i = 15; i < 19; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(Query_NET_ID_WRITE[i]) << " ";
  }
  std::cerr << "\n";
  for (int i = 39; i < 43; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(Query_NET_ID_WRITE[i]) << " ";
  }
  std::cerr << "\n";
#endif
}

void show_state(uint8_t *buf, const long long &ts) {
  int OFFSET = 26;
  int alarm_code_R     = static_cast<int>(buf[ 3] << 24 | buf[ 4] << 16 | buf[ 5] << 8 | buf[ 6]);
  double temp_driver_R = static_cast<int>(buf[ 7] << 24 | buf[ 8] << 16 | buf[ 9] << 8 | buf[10]) * 0.1;
  double temp_motor_R  = static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]) * 0.1;
  int position_R       = static_cast<int>(buf[15] << 24 | buf[16] << 16 | buf[17] << 8 | buf[18]);
  int power_R          = static_cast<int>(buf[19] << 24 | buf[20] << 16 | buf[21] << 8 | buf[22]);
  double voltage_R     = static_cast<int>(buf[23] << 24 | buf[24] << 16 | buf[25] << 8 | buf[26]) * 0.1;

  int alarm_code_L     = static_cast<int>(buf[ 3 + OFFSET] << 24 | buf[ 4 + OFFSET] << 16 | buf[ 5 + OFFSET] << 8 | buf[ 6 + OFFSET]);
  double temp_driver_L = static_cast<int>(buf[ 7 + OFFSET] << 24 | buf[ 8 + OFFSET] << 16 | buf[ 9 + OFFSET] << 8 | buf[10 + OFFSET]) * 0.1;
  double temp_motor_L  = static_cast<int>(buf[11 + OFFSET] << 24 | buf[12 + OFFSET] << 16 | buf[13 + OFFSET] << 8 | buf[14 + OFFSET]) * 0.1;
  int position_L       = static_cast<int>(buf[15 + OFFSET] << 24 | buf[16 + OFFSET] << 16 | buf[17 + OFFSET] << 8 | buf[18 + OFFSET]);
  int power_L          = static_cast<int>(buf[19 + OFFSET] << 24 | buf[20 + OFFSET] << 16 | buf[21 + OFFSET] << 8 | buf[22 + OFFSET]);
  double voltage_L     = static_cast<int>(buf[23 + OFFSET] << 24 | buf[24 + OFFSET] << 16 | buf[25 + OFFSET] << 8 | buf[26 + OFFSET]) * 0.1;

  double dist_L = position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R = position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;

  bat->ts = ts;
  bat->voltage = (voltage_L + voltage_R)/2.0;

  std::cerr << "\033[1;1H" << "-------------";
  std::cerr << "\033[2;1H" << "Alarm_L:" << alarm_code_L;
  std::cerr << "\033[3;1H" << "Driver_L temp:" << std::dec << temp_driver_L;
  std::cerr << "\033[4;1H" << "Motor_L  temp:" << std::dec << temp_motor_L;
  std::cerr << "\033[5;1H" << "Position_L:" << position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  std::cerr << "\033[6;1H" << "Power_L:" << power_L;
  std::cerr << "\033[7;1H" << "Voltage_L:" << voltage_R;

  std::cerr << "\033[2;40H" <<  "Alarm_R:" << alarm_code_R;
  std::cerr << "\033[3;40H" <<  "Driver_R temp:" << std::dec << temp_driver_R;
  std::cerr << "\033[4;40H" <<  "Motor_R  temp:" << std::dec << temp_motor_R;
  std::cerr << "\033[5;40H" <<  "Position_R:" << position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  std::cerr << "\033[6;40H" <<  "Power_R:" << power_R;
  std::cerr << "\033[7;40H" <<  "Voltage_R:" << voltage_R;
  std::cerr << "\033[8;1H\033[2K" << travel << " " << rotation * 180.0/M_PI
    << " " << loc->x << " " << loc->y << " " << loc->a * 180/M_PI;
  std::cerr << "\033[9;1H" << "-------------\n";
}

void read_odo(uint8_t *buf, ODOMETORY &odo) {
#if 0
  int OFFSET = 6;
  int START = 3;
#else
  int START = 15;
  int OFFSET = 26;
#endif
  int position_R = static_cast<int>(buf[START] << 24 | buf[START+1] << 16 | buf[START+2] << 8 | buf[START+3]);
  int position_L = static_cast<int>(buf[START + OFFSET] << 24 | buf[START+1 + OFFSET] << 16 | buf[START+2 + OFFSET] << 8 | buf[START+3 + OFFSET]);

  double dist_L = position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R =-position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;
  double dl = travel - odo.travel;
  double dth = rotation - odo.rotation;
  odo.rx += dl * cos(odo.ra);
  odo.ry += dl * sin(odo.ra);
  odo.ra += dth;
  if (odo.ra > M_PI) odo.ra -= 2*M_PI;
  else if (odo.ra < -M_PI) odo.ra += 2*M_PI;
  odo.dist_R = dist_R;
  odo.dist_L = dist_L;
  odo.travel = travel;
  odo.rotation = rotation;
}

#endif
