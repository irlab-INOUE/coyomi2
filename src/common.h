#ifndef __COMMON_H__
#define __COMMON_H__

#include "yaml-cpp/yaml.h"

YAML::Node yamlRead(std::string path) {
  try {
    return YAML::LoadFile(path);
  } catch(YAML::BadFile &e) {
    std::cerr << "read error! yaml is not exist."<< std::endl;
    exit(EXIT_FAILURE);
  }
}

// log file
std::ofstream enc_log;
std::ofstream fout_urg2d;
std::ofstream mcl_log;
std::ofstream de_log;
std::ofstream sound_log;

#endif
