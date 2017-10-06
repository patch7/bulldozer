#ifndef __KPP
#define __KPP

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "sliding_median.h"
#include "engine.h"

const static uint8_t N = 0;
const static uint8_t F = 1;
const static uint8_t R = 2;

const static uint8_t PLUS  = 1;
const static uint8_t MINUS = 2;

const static uint8_t OFF = 0;
const static uint8_t ON  = 1;

//все методы класса необходимо пересмотреть после создания общей структуры!!!
class KPP
{
public:
  KPP(const uint8_t size_filter = 9) : 
  SMleft(size_filter),
  SMright(size_filter),
  SMthrottle(size_filter),
  SMbrake(size_filter),
  SMdeceler(size_filter),
  SMtemp(size_filter),
  direction(0),
  clutch_state(0),
  parking(1),
  auto_reverse(0),
  clutch(0),
  old_direct(0),
  oil_filter(0),
  d_generator(0),
  start_eng(0),
  parking_ch(0),
  clutch_ch(0),
  direct_ch(0)
  {}
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
  void ResetAllClutch()              const;
  void SetDirection(const uint8_t)   const;
  void ResetDirection(const uint8_t) const;
  void ResetAllDirect()              const;

  void BrakeRotate();
  void SwitchDirection(Engine&);
  void Parking(const uint16_t);
  void SetClutch(const uint16_t);
  void DigitalSet(const uint16_t);
  void AnalogSet(const uint16_t*);
  void SendMsg();
private:
  void PropBrakeR(const uint8_t) const;
  void PropBrakeL(const uint8_t) const;
  void PropSetOtL(const uint8_t) const;
  void PropSetOtR(const uint8_t) const;

  void OnClutch()     const;
  void OffClutch()    const;

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

  CanTxMsg TxMessage;

  SlidingMedian SMleft, SMright, SMthrottle, SMbrake, SMdeceler, SMtemp;

  uint8_t direction    : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  uint8_t clutch_state : 2;//00 - none, 01 - '+', 10 - '-',  11 - Not available
  uint8_t parking      : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t auto_reverse : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t clutch       : 2;//00 - 0,    01 - 1,   10 - 2,    11 - 3
  uint8_t old_direct   : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  
  uint8_t oil_filter   : 1;//true/false
  uint8_t d_generator  : 1;//true/false

  uint8_t start_eng    : 1;//true/false
  uint8_t parking_ch   : 1;//true/false
  uint8_t clutch_ch    : 1;//true/false
  uint8_t direct_ch    : 1;//true/false
};

//inline методы должны быть включены в каждую трансляцию, так что лучше их определять в заголовке.

inline void KPP::SetOtL() const       { TIM_SetCompare1(TIM4, 500); }
inline void KPP::ResetOtL() const     { TIM_SetCompare1(TIM4, 0); }

inline void KPP::SetOtR() const       { TIM_SetCompare2(TIM4, 500); }
inline void KPP::ResetOtR() const     { TIM_SetCompare2(TIM4, 0); }

inline void KPP::SetBfL() const       { TIM_SetCompare3(TIM4, 500); }
inline void KPP::ResetBfL() const     { TIM_SetCompare3(TIM4, 0); }

inline void KPP::SetBfR() const       { TIM_SetCompare4(TIM4, 500); }
inline void KPP::ResetBfR() const     { TIM_SetCompare4(TIM4, 0); }

inline void KPP::SetFirst() const     { TIM_SetCompare1(TIM3, 500); }
inline void KPP::ResetFirst() const   { TIM_SetCompare1(TIM3, 0); }

inline void KPP::SetSecond() const    { TIM_SetCompare2(TIM3, 500); }
inline void KPP::ResetSecond() const  { TIM_SetCompare2(TIM3, 0); }

inline void KPP::SetThird() const     { TIM_SetCompare3(TIM3, 500); }
inline void KPP::ResetThird() const   { TIM_SetCompare3(TIM3, 0); }

inline void KPP::SetForward() const   { TIM_SetCompare4(TIM3, 500); }
inline void KPP::ResetForward() const { TIM_SetCompare4(TIM3, 0); }

inline void KPP::SetReverse() const   { TIM_SetCompare2(TIM1, 500); }
inline void KPP::ResetReverse() const { TIM_SetCompare2(TIM1, 0); }

inline void KPP::PropSetOtL(const uint8_t data) const
{
  TIM_SetCompare1(TIM4, 500 - data * 5);//500 - data * 500 / 100
}
inline void KPP::PropSetOtR(const uint8_t data) const
{
  TIM_SetCompare2(TIM4, 500 - data * 5);//500 - data * 500 / 100
}
#endif /* __KPP */