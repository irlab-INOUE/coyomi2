#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "query.h"

#define MAX_BUFFER_SIZE 256
#define BAUDRATE B115200

#define SERIAL_PORT   "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ035HQB-if00-port0"
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
  std::cerr << "[SEND]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd[i]) << " ";
  }
  std::cerr << "\n";
  int n = write(fd, cmd, length);
}

void read_res(uint8_t *buf, int length) {
  std::cerr << "[RESP]";
  int len = read(fd, buf, length);
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]) << " ";
  }
  std::cerr << "\n";
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
  usleep(5000);
  read_res(buf, 13);
  temp_driver_R = 0.1 * static_cast<int>(buf[3] << 24 | buf[4] << 16 | buf[5] << 8 | buf[6]);
  temp_motor_R  = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);

  calcBcc(Query_Read_Temperature_L, 8);
  send_cmd(Query_Read_Temperature_L, 8);
  usleep(5000);
  read_res(buf, 13);
  temp_driver_L = 0.1 * static_cast<int>(buf[3] << 24 | buf[4] << 16 | buf[5] << 8 | buf[6]);
  temp_motor_L  = 0.1 * static_cast<int>(buf[7] << 24 | buf[8] << 16 | buf[9] << 8 | buf[10]);

  std::cerr << "Driver_R temp:" << std::dec << temp_driver_R << "\n";
  std::cerr << "Motor_R  temp:" << std::dec << temp_motor_R  << "\n";
  std::cerr << "Driver_L temp:" << std::dec << temp_driver_L << "\n";
  std::cerr << "Motor_L  temp:" << std::dec << temp_motor_L  << "\n";
  usleep(5000);
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

  //trun on exitation on L motor
  std::cerr << "Turn ON\n";
  calcBcc(Query_Write_Son_R, 13);
  send_cmd(Query_Write_Son_R, 13);
  usleep(5000);
  uint8_t buf[MAX_BUFFER_SIZE];
  read_res(buf, 8);
  calcBcc(Query_Write_Son_L, 13);
  send_cmd(Query_Write_Son_L, 13);
  usleep(5000);
  read_res(buf, 8);
  getchar();

  std::cerr << "Read temperature\n";
  read_temperature();
  getchar();

  // turn off exitation
  std::cerr << "Turn OFF\n";
  calcBcc(Query_Write_Soff_R, 13);
  send_cmd(Query_Write_Soff_R, 13);
  usleep(5000);
  read_res(buf, 8);
  calcBcc(Query_Write_Soff_L, 13);
  send_cmd(Query_Write_Soff_L, 13);
  usleep(5000);
  read_res(buf, 8);
  getchar();

  return 0;
}

