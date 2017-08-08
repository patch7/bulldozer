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
  DigitalSignals() : clutch_state(0), old_direct(0), direction(0), direct_ch(0), parking(0),
                     auto_reverse(0), parking_ch(0), start_eng(0), clutch_ch(0), clutch(0) {}
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
  
  void SetAllOt()                    const;
  void ResetAllOt()                  const;
  void Brake(const uint8_t)          const;
  void ResetAllValve()               const;
  void SetAllBf()                    const;
  void SetClutch(const uint8_t)      const;
  void ResetClutch(const uint8_t)    const;
  void SetDirection(const uint8_t)   const;
  void ResetDirection(const uint8_t) const;
private:
  //void SetClutch(const uint8_t, const uint8_t) const;
  void ResetOtL()     const;
  void ResetOtR()     const;
  void ResetFirst()   const;
  void ResetSecond()  const;
  void ResetThird()   const;
  void ResetForward() const;
  void ResetReverse() const;
  
  //особой необходимости в этих методах нет, используются только по 1 разу.
  void ResetBfL()     const;
  void ResetBfR()     const;
  void SetOtL()       const;
  void SetOtR()       const;
  void SetBfL()       const;
  void SetBfR()       const;
  void SetFirst()     const;
  void SetSecond()    const;
  void SetThird()     const;
  void SetForward()   const;
  void SetReverse()   const;
};

#endif /* __KPP */