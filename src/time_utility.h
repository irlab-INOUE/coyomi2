#include <chrono>

using namespace std::chrono;

long long get_current_time() {
  auto time_now = high_resolution_clock::now();
  long long ts = duration_cast<milliseconds>(time_now.time_since_epoch()).count();
  return ts;
}

