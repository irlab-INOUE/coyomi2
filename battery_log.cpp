#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shm_board.h"

#define TEXT_RED "\e[31m"
#define TEXT_GREEN "\e[32m"
#define TEXT_BLUE "\e[34m"
#define TEXT_WHITE "\e[37m"
#define TEXT_COLOR_RESET "\e[0m"

void sigcatch( int );
std::ofstream bat_log;

// 共有したい構造体毎にアドレスを割り当てる
BAT *shm_bat        = nullptr;

int main(int argc, char *argv[]) {
	/* Ctrl+c 対応 */
	if (SIG_ERR == signal( SIGINT, sigcatch )) {
		std::printf("failed to set signal handler\n");
		exit(1);
	}

  std::cerr << "Hello, Battery logger" << "\n";

	/***************************************************************************
		共有メモリの確保
	 ***************************************************************************/
	// 共有したい構造体毎にアドレスを割り当てる
	shm_bat        =        (BAT *)shmAt(KEY_BAT, sizeof(BAT));
  std::cerr << TEXT_GREEN << "Completed shared memory allocation\n" << TEXT_COLOR_RESET;

  bat_log.open("batlog");

  while(1) {
    std::cout << shm_bat->ts << " " << shm_bat->voltage << "\n";
    bat_log << shm_bat->ts << " " << shm_bat->voltage << "\n";
    sleep(1);
  }

#if 0
	// 共有メモリのクリア
	shmdt(shm_bat);
	int keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);
	std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;
#endif

  return 0;
}

void sigcatch(int sig) {
  std::cerr << TEXT_RED;
	std::printf("Catch signal %d\n", sig);
  std::cerr << TEXT_COLOR_RESET;

#if 0
	// 共有メモリのクリア
	shmdt(shm_bat);
	int keyID = shmget(KEY_BAT, sizeof(BAT), 0666 | IPC_CREAT);
	shmctl(keyID, IPC_RMID, nullptr);

	std::cerr << TEXT_BLUE << "shm all clear, Bye!\n" << TEXT_COLOR_RESET;
#endif
	exit(1);
}
