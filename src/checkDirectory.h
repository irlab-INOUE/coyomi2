#ifndef __CHECK_DIRECTORY_H__
#define __CHECK_DIRECTORY_H__

#include <string>
#include <fstream>
#include <cstdio>

void checkDir(std::string storeDir)
{
	std::ifstream dir(storeDir);
	if (!dir.is_open()) {
		std::string cmd;
		char* ccmd;
		cmd = "mkdir -p " + std::string(storeDir.data()) + "\n";
		ccmd = new char[cmd.size() + 1];
		std::char_traits<char>::copy(ccmd, cmd.c_str(), cmd.size() + 1);
		int ret = system(ccmd);
	}
}

#endif
