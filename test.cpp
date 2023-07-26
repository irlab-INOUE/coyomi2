#include <iostream>
#include <fstream>
#include <cmath>

int main() {
  std::ifstream fin("./urglog");
  std::string tmp;
  long long ts;
  int data_size;
  double start;
  double end;
  double step;
  int echo;
  long d;
  fin >> tmp;
  int count = 0;
  while (!fin.eof()) {
    std::string fname = "lsp" + std::to_string(count);
    std::ofstream fout(fname);
    fin >> ts >> data_size >> start >> end >> step >> echo;
    for (int i = 0; i < data_size/echo; i++) {
      fin >> d >> tmp >> tmp;
      if (d > 35000) continue;
      fout << d/1000.0 * cos((static_cast<double>(step) * i + static_cast<double>(start))*M_PI/180.0) << " "
          << d/1000.0 * sin((static_cast<double>(step) * i + static_cast<double>(start))*M_PI/180.0) << "\n";
    }
    fin >> tmp;
    count++;
    fin >> tmp;
  }
  return 0;
}
