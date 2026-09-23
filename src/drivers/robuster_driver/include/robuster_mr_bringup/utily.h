#ifndef __UTILY_H__
#define __UTILY_H__

#include <time.h>

inline long int getSysTime() {
  struct timespec tm;
  clock_gettime(CLOCK_MONOTONIC, &tm);

  return (long int)(tm.tv_sec * 1000 + tm.tv_nsec / 1000000); // ms
}

#endif