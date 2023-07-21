#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "query.h"

#define MAX_BUFFER_SIZE 256
#define BAUDRATE B115200

#define SERIAL_PORT   "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ035HQB-if00-port0"

#define DEBUG_SENDRESP

int fd;

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
#ifdef DEBUG_SENDRESP
  std::cerr << "[SEND]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd[i]) << " ";
  }
  std::cerr << "\n";
#endif
  int n = write(fd, cmd, length);
}

void read_res(uint8_t *buf, int length) {
  int len = read(fd, buf, length);
#ifdef DEBUG_SENDRESP
  std::cerr << "[RESP]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]) << " ";
  }
  std::cerr << "\n";
#endif
}

void read_temperature() {
  // read temperatures on driver and motor
  double temp_driver_R;
  double temp_driver_L;
  double temp_motor_R;
  double temp_motor_L;
  uint8_t buf[MAX_BUFFER_SIZE];

  calcBcc(Query_Read_Temperature_R, 8);
  send_cmd(Query_Read_Temperature_R, 8);
  usleep(10000);
  read_res(buf, 13);
  temp_driver_R = 0.1 * static_cast<int>(buf[3] << 24 | buf[4] << 16 | buf[5] << 8 | buf[6]);
  temp_motor_R  = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);

  calcBcc(Query_Read_Temperature_L, 8);
  send_cmd(Query_Read_Temperature_L, 8);
  usleep(10000);
  read_res(buf, 13);
  temp_driver_L = 0.1 * static_cast<int>(buf[3] << 24 | buf[4] << 16 | buf[5] << 8 | buf[6]);
  temp_motor_L  = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);

  std::cerr << "Driver_R temp:" << std::dec << temp_driver_R << "\n";
  std::cerr << "Motor_R  temp:" << std::dec << temp_motor_R  << "\n";
  std::cerr << "Driver_L temp:" << std::dec << temp_driver_L << "\n";
  std::cerr << "Motor_L  temp:" << std::dec << temp_motor_L  << "\n";
  usleep(10000);
}

int main(int argc, char *argv[]) {
  std::cerr << "Hello, Coyomi2" << "\n";
  std::string devName = SERIAL_PORT;
  fd = open(devName.c_str(), O_RDWR | O_NOCTTY);
  if(fd < 0) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd: " << fd << "\n";
  }
  
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd, TCSANOW, &tio);

  uint8_t buf[MAX_BUFFER_SIZE];

  // ID Share Config.
  std::cerr << "Shared ID CONFIG\n";
  calcBcc(Query_IDshare_R, 21);
  send_cmd(Query_IDshare_R, 21);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_IDshare_L, 21);
  send_cmd(Query_IDshare_L, 21);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_READ_R, 21);
  send_cmd(Query_READ_R, 21);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_READ_L, 21);
  send_cmd(Query_READ_L, 21);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_WRITE_R, 33);
  send_cmd(Query_WRITE_R, 33);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_WRITE_L, 33);
  send_cmd(Query_WRITE_L, 33);
  usleep(10000);
  read_res(buf, 8);

#if 1
  //trun on exitation on RL motor
  std::cerr << "Turn ON RL\n";
  calcBcc(Query_Write_Son_R, 13);
  send_cmd(Query_Write_Son_R, 13);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_Write_Son_L, 13);
  send_cmd(Query_Write_Son_L, 13);
  usleep(10000);
  read_res(buf, 8);
  getchar();
#endif

// Read temperature using share id
  std::cerr << "Read temperature\n";
  calcBcc(Query_NET_ID_READ, 8);
  send_cmd(Query_NET_ID_READ, 8);
  usleep(10000);
  read_res(buf, 33);
  usleep(10000);
  double temp_driver_R = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);
  double temp_motor_R  = 0.1 * static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]);
  double temp_driver_L = 0.1 * static_cast<int>(buf[21] << 24 | buf[22] << 16 | buf[23] << 8 | buf[24]);
  double temp_motor_L  = 0.1 * static_cast<int>(buf[25] << 24 | buf[26] << 16 | buf[27] << 8 | buf[28]);
  std::cerr << "---\n";
  std::cerr << "Driver_R temp:" << std::dec << temp_driver_R << "\t";
  std::cerr << "Motor_R  temp:" << std::dec << temp_motor_R  << "\n";
  std::cerr << "Driver_L temp:" << std::dec << temp_driver_L << "\t";
  std::cerr << "Motor_L  temp:" << std::dec << temp_motor_L  << "\n";

  std::cerr << "Start rotation\n";
  calcBcc(Query_NET_ID_WRITE, 57);
  send_cmd(Query_NET_ID_WRITE, 57);
  usleep(10000);
  read_res(buf, 8);
  usleep(10000);
  int DRIVE_TIME = 10; // sec
  sleep(DRIVE_TIME);
  Query_NET_ID_WRITE[17] = 0;
  Query_NET_ID_WRITE[18] = 0;
  Query_NET_ID_WRITE[41] = 0;
  Query_NET_ID_WRITE[42] = 0;
  calcBcc(Query_NET_ID_WRITE, 57);
  send_cmd(Query_NET_ID_WRITE, 57);
  usleep(10000);
  read_res(buf, 8);
  usleep(10000);
  sleep(1);

  std::cerr << "Read temperature\n";
  calcBcc(Query_NET_ID_READ, 8);
  send_cmd(Query_NET_ID_READ, 8);
  usleep(10000);
  read_res(buf, 33);
  usleep(10000);
  temp_driver_R = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);
  temp_motor_R  = 0.1 * static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]);
  temp_driver_L = 0.1 * static_cast<int>(buf[21] << 24 | buf[22] << 16 | buf[23] << 8 | buf[24]);
  temp_motor_L  = 0.1 * static_cast<int>(buf[25] << 24 | buf[26] << 16 | buf[27] << 8 | buf[28]);
  std::cerr << "---\n";
  std::cerr << "Driver_R temp:" << std::dec << temp_driver_R << "\t";
  std::cerr << "Motor_R  temp:" << std::dec << temp_motor_R  << "\n";
  std::cerr << "Driver_L temp:" << std::dec << temp_driver_L << "\t";
  std::cerr << "Motor_L  temp:" << std::dec << temp_motor_L  << "\n";

  // turn off exitation on RL motor
  std::cerr << "Turn OFF RL\n";
  calcBcc(Query_Write_Soff_R, 13);
  send_cmd(Query_Write_Soff_R, 13);
  usleep(10000);
  read_res(buf, 8);
  calcBcc(Query_Write_Soff_L, 13);
  send_cmd(Query_Write_Soff_L, 13);
  usleep(10000);
  read_res(buf, 8);
  usleep(10000);
  getchar();

  return 0;
}

