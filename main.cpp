#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <fstream>
#include <chrono>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/wait.h>
#include <linux/joystick.h>

#include "query.h"
#include "Urg2d.h"
#include "shm_board.h"

#define MAX_BUFFER_SIZE 512 
#define BAUDRATE B230400
#define SERIAL_INTERVAL_SEND 4000
#define SERIAL_INTERVAL_RESP 4000

#define SERIAL_PORT   "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ035HQB-if00-port0"
#define JS_PORT       "/dev/input/js0"

#define WHEEL_D 0.311
#define WHEEL_T 0.461
#define GEAR_RATIO 50.0 
#define STEP_RESOLUTION 0.01/180*M_PI

#define FORWARD_MAX_SPEED   1.5
#define ROTATION_MAX_SPEED  1.0

#define TEXT_RED "\e[31m"
#define TEXT_GREEN "\e[32m"
#define TEXT_BLUE "\e[34m"
#define TEXT_WHITE "\e[37m"
#define TEXT_COLOR_RESET "\e[0m"

//#define DEBUG_SENDRESP

using namespace std::chrono;

void sigcatch( int );
void read_res(uint8_t *buf, int length);

int fd;
int fd_js;    // file descriptor to joystick

std::ofstream enc_log;
std::ofstream urg2d_log;

int *axis = NULL, num_of_axis = 0, num_of_buttons = 0;
char *button = NULL, name_of_joystick[80];

// 共有したい構造体毎にアドレスを割り当てる
BAT *shm_bat        = nullptr;

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
  usleep(SERIAL_INTERVAL_SEND);
}

void simple_send_cmd(uint8_t *cmd, int length) {
  send_cmd(cmd, length);
  uint8_t res_buf[MAX_BUFFER_SIZE];
  read_res(res_buf, 8);
}

int ReadByte(uint8_t *buf) {
  return read(fd, buf, sizeof(uint8_t));
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

  shm_bat->ts = ts;
  shm_bat->voltage = (voltage_L + voltage_R)/2.0;

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
  std::cerr << "\033[8;1H" << travel << " " << rotation * 180.0/M_PI;
  std::cerr << "\033[9;1H" << "-------------\n";
}

void turn_on_motors() {
  std::cerr << "\033[12;1H" << "Turn ON RL...";
  simple_send_cmd(Query_Write_Son_R, sizeof(Query_Write_Son_R));
  simple_send_cmd(Query_Write_Son_L, sizeof(Query_Write_Son_L));
  std::cerr << "Done.\n";
}

void turn_off_motors() {
  std::cerr << "\033[12;1H" << "Turn OFF RL...";
  simple_send_cmd(Query_Write_Soff_R, sizeof(Query_Write_Soff_R));
  simple_send_cmd(Query_Write_Soff_L, sizeof(Query_Write_Soff_L));
  std::cerr << "Done.\n";
}

void free_motors() {
  std::cerr << "\033[12;1H" << "FREE RL...";
  simple_send_cmd(Query_Write_FREE_R, sizeof(Query_Write_FREE_R));
  simple_send_cmd(Query_Write_FREE_L, sizeof(Query_Write_FREE_L));
  std::cerr << "Done.\n";
}

void read_odo(uint8_t *buf, ODOMETORY &odo) {
#if 0
  int OFFSET = 6;
  int START = 3;
#else
  int START = 15;
  int OFFSET = 26;
#endif
  int position_R       = static_cast<int>(buf[START] << 24 | buf[START+1] << 16 | buf[START+2] << 8 | buf[START+3]);
  int position_L       = static_cast<int>(buf[START + OFFSET] << 24 | buf[START+1 + OFFSET] << 16 | buf[START+2 + OFFSET] << 8 | buf[START+3 + OFFSET]);

  double dist_L = position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R = position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;
  double dl = travel - odo.travel;
  double dth = rotation - odo.rotation;
  odo.rx += dl * cos(odo.ra);
  odo.ry += dl * sin(odo.ra);
  odo.ra += dth;
  odo.dist_R = dist_R;
  odo.dist_L = dist_L;
  odo.travel = travel;
  odo.rotation = rotation;
}

void read_state(ODOMETORY &odo, const long long &ts) {
  uint8_t buf[MAX_BUFFER_SIZE];
  //send_cmd(Query_NET_ID_READ_ODO, sizeof(Query_NET_ID_READ_ODO));
  //read_res(buf, 17);
  send_cmd(Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(buf, 57);
#if 1
  std::cerr << "\033[11A";
  std::cerr << "Read state\n";
  show_state(buf, ts);
#endif
  read_odo(buf, odo);
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

std::vector<pid_t> p_list;
int main(int argc, char *argv[]) {
	/* Ctrl+c 対応 */
	if (SIG_ERR == signal( SIGINT, sigcatch )) {
		std::printf("failed to set signal handler\n");
		exit(1);
	}

  std::cerr << "Hello, Coyomi2" << "\n";

	/***************************************************************************
		共有メモリの確保
	 ***************************************************************************/
	// 共有したい構造体毎にアドレスを割り当てる
	shm_bat        =        (BAT *)shmAt(KEY_BAT, sizeof(BAT));
  std::cerr << TEXT_GREEN << "Completed shared memory allocation\n" << TEXT_COLOR_RESET;

  //Urg2d urg2d;
  //urg2d_log.open("urglog");
  enc_log.open("enclog");

  if((fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY)) == -1) {
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
  bool isFREE = false;

//#define URGDEBUG
#ifdef URGDEBUG
  while(1) {
    std::vector<LSP> result = urg2d.getData();
    int key = urg2d.view(5);
    if (key == 'q') break;
  }
  turn_off_motors();
  return 0;  
#endif
  
  pid_t c_pid = fork();
  if (c_pid == -1) {
    perror("fork");
    exit(EXIT_FAILURE);
  } else if (c_pid > 0) {
    std::cerr << "Start battery log: " << c_pid << "\n";
    p_list.emplace_back(c_pid);
  } else {
    while(1) {
      std::ofstream bat_log("batlog", std::ios_base::app);
      bat_log << shm_bat->ts << " " << shm_bat->voltage << "\n";
      sleep(1);
      bat_log.close();
    }
    exit(EXIT_SUCCESS);
  }

  // Start drive 
  std::cerr << "Start rotation. Please hit Enter key.\n";
  getchar();
  std::cerr << "\033[2J" << "\033[1;1H";
  double v = 0.0;
  double w = 0.0;
  ODOMETORY odo;
  int number_of_lidar_view_count = 1;
  int lidar_view_countdown = number_of_lidar_view_count;
  tcflush(fd, TCIOFLUSH);
  while(1) {
    auto time_now0 = high_resolution_clock::now();
    long long ts0 = duration_cast<milliseconds>(time_now0.time_since_epoch()).count();
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
            usleep(1000);
          }

          switch(js.number) {
            case 0:
              std::cerr << "No." << (int)js.number << "\tTurn Left" << std::endl;
              w = 0.5;
              break;
            case 1:
              std::cerr << "No." << (int)js.number << "\tFoward" << std::endl;
              w = 0.0;
              if (v < 0.01) v = 0.1;
              break;
            case 2:
              std::cerr << "No." << (int)js.number << "\tStop" << std::endl;
              v = 0.0;
              w = 0.0;
              break;
            case 3:
              std::cerr << "No." << (int)js.number << "\tRight" << std::endl;
              w = -0.5;
              break;
            case 4:
              std::cerr << "No." << (int)js.number << "\tSpeed Down" << std::endl;
              v -= 0.1;
              if (v < 0.0) v = 0.0;
              break;
            case 5:
              std::cerr << "No." << (int)js.number << "\tSpeed Up" << std::endl;
              v += 0.1;
              if (v > FORWARD_MAX_SPEED) v = FORWARD_MAX_SPEED;
              break;
            case 6:
              std::cerr << "End\n";
              v = 0.0;
              w = 0.0;
              calc_vw2hex(Query_NET_ID_WRITE, v, w);
              simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
              usleep(1500000);
              goto CLEANUP;
              break;
            case 7:
              std::cerr << "FREE\n";
              v = 0.0;
              w = 0.0;
              calc_vw2hex(Query_NET_ID_WRITE, v, w);
              simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
              usleep(1000000);
              isFREE = !isFREE;
              if (isFREE) 
                free_motors();
              else 
                turn_on_motors();
              break;
            default:
              break;
          }
        }
        break;
    }
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    auto time_now = high_resolution_clock::now();
    long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
    read_state(odo, ts);
    //std::vector<LSP> result = urg2d.getData();
    //urg2d.view(5);
    //std::cout << odo.rx << " " << odo.ry << "\n";

    // enc_log << ts << " " << odo.rx << " " << odo.ry << " " << odo.ra << " " << odo.travel << " " << odo.rotation * 180.0/M_PI << " " << odo.dist_R << " " << odo.dist_L << "\n";
    enc_log 
      << ts << " " 
      << odo.rx << " " << odo.ry << " " << odo.ra << " "
      << "end" << "\n";

#if 0
    urg2d_log << "LASERSCANRT" << " " 
      << ts << " "
      << result.size() * 3 << " " 
      << "-135" << " " << "135" << " " 
      << "0.25" << " " << "3" << " ";
    for (auto d: result) {
      urg2d_log << d.data << " " << "0" << " " << "0" << " ";
    }
    urg2d_log << ts << "\n";
#endif
  }
  //=====<<MAIN LOOP : END>>=====


CLEANUP:
  // turn off exitation on RL motor
  turn_off_motors();

  close(fd_js);

	for (auto pid: p_list) {
		kill(pid, SIGKILL);
	}
	pid_t wait_pid;
	while ((wait_pid = wait(nullptr)) > 0)
		std::cout << "wait:" << wait_pid << "\n";

	// 共有メモリのクリア
	shmdt(shm_bat);
	int keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;

  return 0;
}

void sigcatch(int sig) {
  std::cerr << TEXT_RED;
	std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;

  if (p_list.size() > 0) { // 以降の停止処理は親プロセスだけが行う
    // safe stop
    double v = 0.0;
    double w = 0.0;
    calc_vw2hex(Query_NET_ID_WRITE, v, w);
    simple_send_cmd(Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
    usleep(1500000);
    turn_off_motors();
    std::cerr << TEXT_BLUE << "Motors safe stop\n" << TEXT_COLOR_RESET;

    close(fd_js);
    close(fd);

    for (auto pid: p_list) {
      kill(pid, SIGKILL);
    }
    pid_t wait_pid;
    while ((wait_pid = wait(nullptr)) > 0)
      std::cout << "wait:" << wait_pid << "\n";

    // 共有メモリのクリア
    shmdt(shm_bat);
    int keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
    shmctl(keyID, IPC_RMID, nullptr);

    std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;
  }
	exit(1);
}
