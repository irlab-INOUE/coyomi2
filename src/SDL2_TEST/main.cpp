#include <SDL.h>
#include <iostream>
#include <cmath> // for std::abs

const int DEAD_ZONE = 8000; // デッドゾーン（感度しきい値）

int main() {
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0) {
    std::cerr << "SDLの初期化に失敗: " << SDL_GetError() << std::endl;
    return 1;
  }

  int count = SDL_NumJoysticks();
  std::cout << "ジョイスティック数: " << count << std::endl;

  if (count > 0) {
    SDL_Joystick* joystick = SDL_JoystickOpen(0);
    if (joystick) {
      std::cout << "名前: " << SDL_JoystickName(joystick) << std::endl;
      std::cout << "軸の数: " << SDL_JoystickNumAxes(joystick) << std::endl;
      std::cout << "ボタンの数: " << SDL_JoystickNumButtons(joystick) << std::endl;

      SDL_Event e;
      bool running = true;

      std::cout << "ボタンやスティックを操作してください。終了は Ctrl+C。\n";

      while (running) {
        while (SDL_PollEvent(&e)) {
          switch (e.type) {
            case SDL_JOYBUTTONDOWN:
              std::cout << "ボタン " << static_cast<int>(e.jbutton.button) << " が押されました。" << std::endl;
              break;

            case SDL_JOYBUTTONUP:
              std::cout << "ボタン " << static_cast<int>(e.jbutton.button) << " が離されました。" << std::endl;
              break;

            case SDL_JOYAXISMOTION:
              if (std::abs(e.jaxis.value) > DEAD_ZONE) {
                std::cout << "軸 " << static_cast<int>(e.jaxis.axis)
                  << " が動きました。値: " << e.jaxis.value << std::endl;
              }
              break;

            case SDL_QUIT:
              running = false;
              break;

            default:
              break;
          }
        }
        SDL_Delay(10);
      }

      SDL_JoystickClose(joystick);
    } else {
      std::cerr << "ジョイスティックのオープンに失敗: " << SDL_GetError() << std::endl;
    }
  }

  SDL_Quit();
  return 0;
}
