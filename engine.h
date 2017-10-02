#ifndef __ENGINE
#define __ENGINE

#include "stm32f4xx.h"

class Engine
{
public:
  Engine()                          = default;
  Engine(const Engine&)             = delete;
  Engine(Engine&&)                  = delete;
  Engine& operator= (const Engine&) = delete;
  Engine& operator= (Engine&&)      = delete;
  ~Engine()                         = default;
  
  void RequestRpm() const;
  
  inline void SetRpm(const uint16_t x) { rpm = x; };
  inline uint16_t GetRpm() const       { return rpm; };
  //inline void SetLoad(const uint8_t x) { load = x; };
  //inline uint8_t GetLoad() const       { return load; };
private:
  const static uint8_t resol = 8;
  //uint8_t load = 0;
  uint16_t rpm = 0;
};

#endif /* __ENGINE */