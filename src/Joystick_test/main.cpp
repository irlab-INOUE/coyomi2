#include <iostream>
#include <linux/joystick.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <cmath>
#include <iomanip>

#include <cstring>
#include <termios.h>

#define JOY_DEV "/dev/input/js0"

int joy_fd, *axis = NULL, num_of_axis = 0, num_of_buttons = 0;
char *button = NULL, name_of_joystick[80];

// The max (min) value for each axis based measurement value
struct joy_calib {
  int max;
  int min;
  int zero;

  joy_calib() {
    max = 0;
    min = 0;
    zero = 0;
  };

  void set_val(int val) {
    if (max < val) {
      max = val;
    }

    if (min > val) {
      min = val;
    }
  };

  void set_zero(int val) {
    zero = val;
  };
};

int main(int argc, char *argv[]) {
  std::cout << "Hello, joy-con\n";

  // setup joystick
  struct js_event js;
  if ((joy_fd = open(JOY_DEV, O_RDONLY)) == -1) {
    std::cerr << "Couldn't open joystick" << std::endl;
    return -1;
  }

  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  axis = (int *)calloc(num_of_axis, sizeof(int));
  button = (char *)calloc(num_of_buttons, sizeof(char));

  std::cerr << "Joystick detected:" << name_of_joystick << std::endl;
  std::cerr << num_of_axis << " axis" << std::endl;
  std::cerr << num_of_buttons << " buttons" << std::endl << std::endl;

  fcntl(joy_fd, F_SETFL, O_NONBLOCK);   /* use non-blocking mode */

  js.number = 0;
  js.value = 0;

  std::cerr << "Joypad ready completed" << std::endl;

  // calibrate axis
  // until JS_EVENT_BUTTON pressed
  bool loop_out_flag = false;
  std::vector<joy_calib> j_calib(6);
  while (1) {
    /* read the joystick state */
    ssize_t a = read(joy_fd, &js, sizeof(struct js_event));
    switch (js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_AXIS:
        axis[js.number] = js.value;
        j_calib[js.number].set_val(js.value);
        j_calib[js.number].set_zero(js.value);
        break;
      case JS_EVENT_BUTTON:
        if (js.value) {
          std::cout << "Pressed JS_EVENT_BUTTON\n";
          loop_out_flag = true;
        }
        break;
      default:
        break;
    }
    if (loop_out_flag) {
      break;
    }
    usleep(10000);
  }
  for (auto j: j_calib) {
    std::cout << j.min << " " << j.max << " " << j.zero << "\n";
  }

  //=====<<MAIN LOOP>>=====
  while (1) {
    /* read the joystick state */
    ssize_t a = read(joy_fd, &js, sizeof(struct js_event));
    std::cout << axis[0] << " " << axis[1];

    // convert axis 0 (1) to -1 ~ 1
    double axis0 = 2.0 * axis[0] / (j_calib[0].max - j_calib[0].min + 10);
    double axis1 = 2.0 * axis[1] / (j_calib[1].max - j_calib[1].min + 10);
    double angle = atan2(-axis1, axis0) * 180/M_PI;
    std::cout << "\e[2K";   // 現在行を消去
    std::cout << std::fixed << std::setprecision(2);  // 表示桁数を設定
    std::cout << angle << " Axis 0:" << axis0 << " Axis 1:" << axis1 << "\n";
    std::cout << std::defaultfloat;

    switch (js.type & ~JS_EVENT_INIT) {
      case JS_EVENT_AXIS:
        axis[js.number] = js.value;
        break;
      case JS_EVENT_BUTTON:
        button[js.number] = js.value;
        if(js.value){
          while (js.value) {
            ssize_t a = read(joy_fd, &js, sizeof(struct js_event));
            usleep(10000);
          }
          switch(js.number) {
            case 0:
              std::cerr << "No." << (int)js.number << "\tTurn Left" << std::endl;
              break;
            case 1:
              std::cerr << "No." << (int)js.number << "\tFoward" << std::endl;
              break;
            case 2:
              std::cerr << "No." << (int)js.number << "\tStop" << std::endl;
              break;
            case 3:
              std::cerr << "No." << (int)js.number << "\tRight" << std::endl;
              break;
            case 4:
              std::cerr << "No." << (int)js.number << "\tSpeed Down" << std::endl;
              break;
            case 5:
              std::cerr << "No." << (int)js.number << "\tSpeed Up" << std::endl;
              break;
            case 6:
              std::cerr << "No." << (int)js.number << std::endl;
              break;
            case 7:
              std::cerr << "No." << (int)js.number << std::endl;
              break;
            case 11:
              std::cerr << "No." << (int)js.number << std::endl;
              break;
            default:
              std::cerr << "No." << (int)js.number << std::endl;
              break;
          }
        }
        break;
    }
    usleep(10000);
  }
  //=====<<MAIN LOOP : END>>=====

  close(joy_fd);
  return 0;
}
