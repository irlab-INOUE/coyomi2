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
#define SERIAL_INTERVAL 30000

#define SERIAL_PORT   "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ035HQB-if00-port0"

//#define DEBUG_SENDRESP

void read_res(uint8_t *buf, int length);

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
  calcBcc(cmd, length);
#ifdef DEBUG_SENDRESP
  std::cerr << "[SEND]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd[i]) << " ";
  }
  std::cerr << "\n";
#endif
  int n = write(fd, cmd, length);
  usleep(SERIAL_INTERVAL);
}

void simple_send_cmd(uint8_t *cmd, int length) {
  send_cmd(cmd, length);
  uint8_t res_buf[MAX_BUFFER_SIZE];
  read_res(res_buf, 8);
  usleep(SERIAL_INTERVAL);
}

void read_res(uint8_t *buf, int length) {
  int len = read(fd, buf, length);
  usleep(SERIAL_INTERVAL);
#ifdef DEBUG_SENDRESP
  std::cerr << "[RESP]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]) << " ";
  }
  std::cerr << "\n";
#endif
}

void show_state(uint8_t *buf) {
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

  std::cerr << "\033[1;1H" << "---";
  std::cerr << "\033[2;1H" << "Alarm_L:" << alarm_code_L;
  std::cerr << "\033[3;1H" << "Driver_L temp:" << std::dec << temp_driver_L;
  std::cerr << "\033[4;1H" << "Motor_L  temp:" << std::dec << temp_motor_L;
  std::cerr << "\033[5;1H" << "Position_L:" << position_L;
  std::cerr << "\033[6;1H" << "Power_L:" << power_L;
  std::cerr << "\033[7;1H" << "Voltage_L:" << voltage_R;

  std::cerr << "\033[2;40H" <<  "Alarm_R:" << alarm_code_R;
  std::cerr << "\033[3;40H" <<  "Driver_R temp:" << std::dec << temp_driver_R;
  std::cerr << "\033[4;40H" <<  "Motor_R  temp:" << std::dec << temp_motor_R;
  std::cerr << "\033[5;40H" <<  "Position_R:" << position_R;
  std::cerr << "\033[6;40H" <<  "Power_R:" << power_R;
  std::cerr << "\033[7;40H" <<  "Voltage_R:" << voltage_R;
  std::cerr << "\033[8;1H" << "---";
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
  std::cerr << "ID Share configration...";
  simple_send_cmd(Query_IDshare_R, sizeof(Query_IDshare_R));
  simple_send_cmd(Query_IDshare_L, sizeof(Query_IDshare_L));
  simple_send_cmd(Query_READ_R,    sizeof(Query_READ_R));
  simple_send_cmd(Query_READ_L,    sizeof(Query_READ_L));
  simple_send_cmd(Query_WRITE_R,   sizeof(Query_WRITE_R));
  simple_send_cmd(Query_WRITE_L,   sizeof(Query_WRITE_L));
  std::cerr << "Done.\n";

  //trun on exitation on RL motor
  std::cerr << "Turn ON RL...";
  simple_send_cmd(Query_Write_Son_R, sizeof(Query_Write_Son_R));
  simple_send_cmd(Query_Write_Son_L, sizeof(Query_Write_Son_L));
  std::cerr << "Done.\n";

  // Read temperature using share id
  std::cerr << "Read state\n";
  send_cmd(Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(buf, 57);
  show_state(buf);

  // Start drive 
  std::cerr << "Start rotation. Please hit Enter key.\n";
  getchar();
  Query_NET_ID_WRITE[17] = 0x0B;  // sample speed command for R
  Query_NET_ID_WRITE[18] = 0xB8;
  Query_NET_ID_WRITE[41] = 0x0B;  // sample speed command for L
  Query_NET_ID_WRITE[42] = 0xB8;
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  int DRIVE_TIME = 6; // cycle
  std::cerr << "\033[2J" << "\033[1;1H";
  for (int i = 0; i < DRIVE_TIME; i++) {
    send_cmd(Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
    read_res(buf, 57);
    std::cerr << "\033[8A";
    show_state(buf);
    sleep(1);
  }
  Query_NET_ID_WRITE[17] = 0;     // stop speed command for R
  Query_NET_ID_WRITE[18] = 0;
  Query_NET_ID_WRITE[41] = 0;     // stop speed command for L
  Query_NET_ID_WRITE[42] = 0;
  simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
  sleep(1);

  // turn off exitation on RL motor
  std::cerr << "\nTurn OFF RL...";
  simple_send_cmd(Query_Write_Soff_R, sizeof(Query_Write_Soff_R));
  simple_send_cmd(Query_Write_Soff_L, sizeof(Query_Write_Soff_L));
  std::cerr << "Done.\n";

  return 0;
}

