#ifndef __GLOBAL_VARIABLE_H__
#define __GLOBAL_VARIABLE_H__
std::atomic<bool> running(true);
std::atomic<bool> get3DLidarData(false);
std::atomic<bool> gotoEnd(false);
std::atomic<bool> isFREE(true);

#endif
