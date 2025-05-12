#include "shared_struct.h"
#include "global_variable.h"

void thread_display(std::shared_ptr<LOGDIR_PATH> log_path, std::shared_ptr<LOG_DATA> log_data,
                    std::shared_ptr<DisplayContents> disp, std::shared_ptr<ENC> enc, std::shared_ptr<LOC> loc) {

  // Ncurses setup
  WINDOW *win = initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  curs_set(0);
  start_color();
  timeout(0);
  init_pair(1, COLOR_BLUE, COLOR_BLACK);
  clear();

  // 下部10行分のサブウィンドウを作成
  int screen_height, screen_width;
  getmaxyx(stdscr, screen_height, screen_width); // 端末サイズ取得
  int log_height = 10;
  int log_width = screen_width;
  int log_starty = screen_height - log_height;
  int log_startx = 0;
  WINDOW* log_win = newwin(log_height, log_width, log_starty, log_startx);

  add_log(log_data, "LOG START");

  int ROW_MCL = 0;
  int ROW_MOTOR = 7;
  int ROW_TOTAL_TRAVEL = 14;
  int ROW_PATH = 15;
  int ROW_CURRENT_MAP_PATH_INDEX = 16;
  mvprintw(ROW_MCL,     0, "MCL Information");
  mvprintw(ROW_MCL+1,   0, "X[m]     Y[m]      A[deg]");
  mvprintw(ROW_MOTOR,   0, "Motor Information");
  mvprintw(ROW_MOTOR+1, 0, "X[m]     Y[m]      A[deg]");
  while (running.load()) {
    // update status window
    disp->temp_driver_L = enc->temp_driver_L;
    disp->temp_driver_R = enc->temp_driver_R;
    disp->temp_motor_L  = enc->temp_motor_L;
    disp->temp_motor_R  = enc->temp_motor_R;

    move(ROW_MCL+2, 0); clrtoeol();
    mvprintw(ROW_MCL+2, 0, "%.3f", disp->loc_x);
    mvprintw(ROW_MCL+2, 9, "%.3f", disp->loc_y);
    mvprintw(ROW_MCL+2,19, "%.1f", disp->loc_a*180/M_PI);

    move(ROW_MCL+3, 0); clrtoeol();
    printw("Current WP Index: %d", disp->current_wp_index);

    move(ROW_MCL+4, 0); clrtoeol();
    printw("v: %.2f  w: %.2f", disp->v, disp->w);

    move(ROW_MCL+5, 0); clrtoeol();
    printw("obx: %.3f  oby: %.3f  ang: %.1f", disp->min_obstacle_x, disp->min_obstacle_y,
           atan2(disp->min_obstacle_y, disp->min_obstacle_x) * 180/M_PI);

    move(ROW_MOTOR+2, 0); clrtoeol();
    mvprintw(ROW_MOTOR+2, 0, "%.3f", disp->enc_x);
    mvprintw(ROW_MOTOR+2, 9, "%.3f", disp->enc_y);
    mvprintw(ROW_MOTOR+2,19, "%.1f", disp->enc_a*180/M_PI);

    move(ROW_MOTOR+3, 0); clrtoeol();
    printw("Voltage: %.1f", disp->battery);

    move(ROW_MOTOR+4, 0); clrtoeol();
    printw("TmpL_D %.1f  TmpR_D %.1f", disp->temp_driver_L, disp->temp_driver_R);

    move(ROW_MOTOR+5, 0); clrtoeol();
    printw("TmpL_M %.1f  TmpR_M %.1f", disp->temp_motor_L, disp->temp_motor_R);

    move(ROW_TOTAL_TRAVEL, 0); clrtoeol();
    printw("Total %.1f", disp->total_travel);

    move(ROW_PATH, 0); clrtoeol();
    printw("%s", loc->path_to_map_dir);

    move(ROW_CURRENT_MAP_PATH_INDEX, 0); clrtoeol();
    printw("CURRENT_MAP_PATH_INDEX %d", loc->CURRENT_MAP_PATH_INDEX);

    // update Log window
    draw_log_window(log_win, log_data, log_width, log_height);

    refresh();
    sleep_for(milliseconds(100));
  }
  endwin();   // ncurses end
  std::cout << "Display exit." << std::endl;
  exit(EXIT_SUCCESS);
}

