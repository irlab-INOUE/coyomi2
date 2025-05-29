#ifndef COYOMI_INCLUDE_SHM_BOARD_H
#define COYOMI_INCLUDE_SHM_BOARD_H

#include <iostream>
#include <cstring>
#include <vector>
//#include <sys/shm.h>
#include <stdlib.h>
#include <string_view>
//#include <semaphore.h>
//#include "Urg2d.h"

// for LOG Display
const int LOG_HEIGHT = 10;
#define LOG_SIZE 1024
#define NUM_LOGS 1000
struct LOG_DATA {
    size_t current_index; // 現在のログインデックス
    std::vector<std::string> logs;

    LOG_DATA() {
      current_index = 0;
      logs.resize(NUM_LOGS);
    }
};

// ログの追加
void add_log(std::shared_ptr<LOG_DATA> shared, std::string_view log) {
  {
    LockGuard lock(mtx);
    if (shared->current_index < NUM_LOGS) {
      shared->logs[shared->current_index] = log;
      shared->current_index++;
    } else {
      //std::cerr << "ログがいっぱいです!" << std::endl;
      shared->current_index = 0;
      shared->logs[shared->current_index] = log;
    }
  }
}

void draw_log_window(WINDOW* win, std::shared_ptr<LOG_DATA> shared, int width, int height) {
    werase(win);
    box(win, 0, 0);

    // タイトル表示（カラー対応）
    if (has_colors()) {
        wattron(win, COLOR_PAIR(1));
        mvwprintw(win, 0, 2, " Log Window ");
        wattroff(win, COLOR_PAIR(1));
    } else {
        mvwprintw(win, 0, 2, " Log Window ");
    }

    int max_lines = height - 2;
    int start = std::max(0, (int)shared->current_index - max_lines);

    try{
      for (int i = 0; i < max_lines; ++i) {
        if (i + start < (int)shared->current_index) {
          mvwprintw(win, i + 1, 2, "%-*s", width - 4, shared->logs[i + start].c_str());
        } else {
          mvwprintw(win, i + 1, 2, "%-*s", width - 4, "");  // 空行でクリア
        }
      }
    } catch (const std::exception& e) {
      ;
    }

    wrefresh(win);
}

#endif // COYOMI_INCLUDE_SHM_BOARD_H
