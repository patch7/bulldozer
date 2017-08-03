#ifndef __KPP
#define __KPP

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "sliding_median.h"
#include "engine.h"

struct DigitalSignals
{
  uint8_t direction    : 2;//00 - N,      01 - F,        10 - R,             11 - Not available
  uint8_t clutch_state : 2;//00 - none,   01 - '+',      10 - '-',           11 - Not available
  uint8_t parking      : 2;//00 - off,    01 - on,       10 - reserved,      11 - Don't care
  uint8_t auto_reverse : 2;//00 - off,    01 - on,       10 - reserved,      11 - Don't care
  uint8_t clutch       : 2;//00 - 0,      01 - 1,        10 - 2,             11 - 3
  uint8_t old_direct   : 2;//00 - N,      01 - F,        10 - R,             11 - Not available
  uint8_t start_eng    : 1;//true/false
  uint8_t parking_ch   : 1;//true/false
  uint8_t clutch_ch    : 1;//true/false
  uint8_t direct_ch    : 1;//true/false
  //uint8_t KPPState     : 2;//00 - manual, 01 - auto low, 10 - auto high/low, 11 - Not available
  //uint8_t hold_KPP     : 2;//00 - off,    01 - on,       10 - reserved,      11 - Don't care
  //uint8_t load_condit  : 2;//00 - off,    01 - on,       10 - reserved,      11 - Don't care
  void Set(const uint8_t);
  DigitalSignals() : direction(0), clutch_state(0), parking(0), auto_reverse(0), start_eng(0), clutch(0) {}
};

struct AnalogSignals
{
  SlidingMedian SMleft, SMright, SMthrottle, SMbrake, SMdeceler, SMtemp;
  void SendMsg(const DigitalSignals);
  void Set(const uint16_t*);
};

class KPP
{
public:
  KPP()                       = default;
  KPP(const KPP&)             = delete;
  KPP(KPP&&)                  = delete;
  KPP& operator= (const KPP&) = delete;
  KPP& operator= (KPP&&)      = delete;
  ~KPP()                      = default;
  
  void Forward();
  void Reverse();
  void ResetAllValve();
  void SetAllOt();
  void SetAllBf();
  void SetClutch(const uint8_t);
  void ResetClutch(const uint8_t);
  void SetDirection(const uint8_t);
  void ResetDirection(const uint8_t);
private:
  //void SetClutch(const uint8_t, const uint8_t) const;
  void SetOtL();
  void ResetOtL();
  
  void SetOtR();
  void ResetOtR();
  
  void SetBfL();
  void ResetBfL();
  
  void SetBfR();
  void ResetBfR();
  
  void SetFirst();
  void ResetFirst();
  
  void SetSecond();
  void ResetSecond();
  
  void SetThird();
  void ResetThird();
  
  void SetForward();
  void ResetForward();
  
  void SetReverse();
  void ResetReverse();
};

#endif /* __KPP */