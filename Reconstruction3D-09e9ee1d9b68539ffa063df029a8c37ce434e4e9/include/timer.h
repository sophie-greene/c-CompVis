#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

class Timer
{
public:
  Timer() { restart(); }
  void restart()
  {
    gettimeofday(&start_time, 0);
  }

  double elapsed()
  {
    timeval end_time;
    gettimeofday(&end_time, 0);
    return 1000.0*(end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec)/1000.0;
  }

  double u_elapsed()
  {
    timeval end_time;
    gettimeofday(&end_time, 0);
    return 1000000.0*(end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec);
  }

  static long int time(){
    timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec;
  }
  static long int u_time(){
    timeval t;
    gettimeofday(&t, 0);
    return 1000*t.tv_sec + t.tv_usec;
  }

private:
  timeval start_time;
};

#endif // TIMER_H
