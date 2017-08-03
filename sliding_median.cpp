#include "sliding_median.h"

SlidingMedian::SlidingMedian(uint16_t s)
{
  size = s;
  array = new uint16_t[size];
  for(short i = 0; i < size; array[i] = 0, i++);
  is_change = false;
}

void SlidingMedian::push(const uint16_t z)
{
  for(short i = 1; i < size; array[i-1] = array[i], i++);
  array[size-1] = z;
  is_change = true;
}

uint16_t SlidingMedian::get()
{
  uint16_t mediane = 0;
  
  if(is_change)
  {
    uint16_t* arr = new uint16_t[size];
    for(short i = 0; i < size; arr[i] = array[i], i++);
    
    for(short i = 0; i < size - 1; ++i)
      for(short j = 0; j < size - i; ++j)
        if(arr[j] > arr[j+1])
        {
          uint16_t temp = arr[j];
          arr[j] = arr[j + 1];
          arr[j + 1] = temp;
        }
      
    mediane = arr[size / 2];
    delete[] arr;
    is_change = false;
  }
  
  return mediane;
}