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
#include <linux/joystick.h>

#include "query.h"

#define MAX_BUFFER_SIZE 256
#define BAUDRATE B115200
#define SERIAL_INTERVAL 30000

#define SERIAL_PORT   "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ035HQB-if00-port0"
#define JS_PORT       "/dev/input/js0"

#define WHEEL_D 0.311
#define WHEEL_T 0.461
#define GEAR_RATIO 50 

#define FORWARD_MAX_SPEED   1.5
#define ROTATION_MAX_SPEED  1.0


//#define DEBUG_SENDRESP

void read_res(uint8_t *buf, int length);

int fd;
int fd_js;    // file descriptor to joystick

int *axis = NULL, num_of_axis = 0, num_of_buttons = 0;
char *button = NULL, name_of_joystick[80];

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

  std::cerr << "\033[1;1H" << "-------------";
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
  std::cerr << "\033[8;1H" << "-------------\n";
}

void turn_on_motors() {
  std::cerr << "Turn ON RL...";
  simple_send_cmd(Query_Write_Son_R, sizeof(Query_Write_Son_R));
  simple_send_cmd(Query_Write_Son_L, sizeof(Query_Write_Son_L));
  std::cerr << "Done.\n";
}

void turn_off_motors() {
  std::cerr << "\nTurn OFF RL...";
  simple_send_cmd(Query_Write_Soff_R, sizeof(Query_Write_Soff_R));
  simple_send_cmd(Query_Write_Soff_L, sizeof(Query_Write_Soff_L));
  std::cerr << "Done.\n";
}

void read_state() {
  uint8_t buf[MAX_BUFFER_SIZE];
  send_cmd(Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(buf, 57);
  std::cerr << "\033[10A";
  std::cerr << "Read state\n";
  show_state(buf);
}

void calc_vw2hex(uint8_t *Query_NET_ID_WRITE, double v, double w) {
  double wr = v/(WHEEL_D/2) + w*WHEEL_T/(1.0*WHEEL_D);
  double wl = v/(WHEEL_D/2) - w*WHEEL_T/(1.0*WHEEL_D);
  double motor_wr_rpm = wr / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  double motor_wl_rpm = wl / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  Query_NET_ID_WRITE[15] = (static_cast<int>(motor_wr_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[16] = (static_cast<int>(motor_wr_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[17] = (static_cast<int>(motor_wr_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[18] =  static_cast<int>(motor_wr_rpm)        & 0xFF;
  Query_NET_ID_WRITE[39] = (static_cast<int>(motor_wl_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[40] = (static_cast<int>(motor_wl_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[41] = (static_cast<int>(motor_wl_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[42] =  static_cast<int>(motor_wl_rpm)        & 0xFF;
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
  //sleep(5);
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

  if ((fd_js = open(JS_PORT, O_RDONLY)) == -1) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd_js: " << fd_js << "\n";
  }

  struct js_event js;
  ioctl(fd_js, JSIOCGAXES, &num_of_axis);
	ioctl(fd_js, JSIOCGBUTTONS, &num_of_buttons);
	ioctl(fd_js, JSIOCGNAME(80), &name_of_joystick);

	axis = (int *)calloc(num_of_axis, sizeof(int));
	button = (char *)calloc(num_of_buttons, sizeof(char));

	std::cerr << "Joystick detected:" << name_of_joystick << std::endl;
	std::cerr << num_of_axis << " axis" << std::endl;
	std::cerr << num_of_buttons << " buttons" << std::endl << std::endl;

	fcntl(fd_js, F_SETFL, O_NONBLOCK);   /* use non-blocking mode */
	js.number = 0;
	std::cerr << "Joypad ready completed" << std::endl;

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd, TCSANOW, &tio);


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
  turn_on_motors();

  // Start drive 
  std::cerr << "Start rotation. Please hit Enter key.\n";
  getchar();
  std::cerr << "\033[2J" << "\033[1;1H";
  double v = 0.0;
  double w = 0.0;
  while(1) {
    std::cerr <<"Hello\n";
    /* read the joystick state */
		ssize_t a = read(fd_js, &js, sizeof(struct js_event));

		/* see what to do with the event */
		switch (js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				if(js.value){
					while (js.value) {
						ssize_t a = read(fd_js, &js, sizeof(struct js_event));
						usleep(10000);
					}

					switch(js.number) {
						case 0:
							std::cerr << "No." << (int)js.number << "\tTurn Left" << std::endl;
              v = 0.5;
              w = 1.0;
							break;
						case 1:
							std::cerr << "No." << (int)js.number << "\tFoward" << std::endl;
              v = 1.0;
              w = 0.0;
							break;
						case 2:
							std::cerr << "No." << (int)js.number << "\tStop" << std::endl;
              v = 0.0;
              w = 0.0;
							break;
						case 3:
							std::cerr << "No." << (int)js.number << "\tRight" << std::endl;
              v = 0.5;
              w = -0.5;
							break;
						case 4:
							std::cerr << "No." << (int)js.number << "\tSpeed Down" << std::endl;
							break;
						case 5:
							std::cerr << "No." << (int)js.number << "\tSpeed Up" << std::endl;
							break;
						case 6:
              std::cerr << "End\n";
              close(fd_js);
              turn_off_motors();
              return 0;
							break;
						default:
							break;
					}
				}
				break;
		}
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    read_state();
		usleep(100000);
	}
	//=====<<MAIN LOOP : END>>=====

  // turn off exitation on RL motor
  turn_off_motors();

  close(fd_js);

  return 0;
}

