#ifndef __SLIDING
#define __SLIDING

#include "stm32f4xx.h"

class SlidingMedian
{
public:
  SlidingMedian(uint16_t s = 7);
  SlidingMedian(const SlidingMedian&)             = delete;
  SlidingMedian(SlidingMedian&&)                  = delete;
  SlidingMedian& operator= (const SlidingMedian&) = delete;
  SlidingMedian& operator= (SlidingMedian&&)      = delete;
  
  void push(const uint16_t);
  uint16_t get();
  
  ~SlidingMedian() { delete[] array; }
private:
  uint16_t size, mediane;
  bool is_change;
  uint16_t* array;
};

#endif /* __SLIDING */