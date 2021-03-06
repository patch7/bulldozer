#ifndef __KPP
#define __KPP

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_flash.h"
#include <utility>
#include <array>
#include "sliding_median.h"

const static uint8_t N     = 0;
const static uint8_t F     = 1;
const static uint8_t R     = 2;
const static uint8_t PLUS  = 1;
const static uint8_t MINUS = 2;
const static uint8_t OFF   = 0;
const static uint8_t ON    = 1;

union Pressure{ uint32_t i; float f; };

bool CanTxMailBox_IsEmpty(CAN_TypeDef*);

class Calibrate
{
public:
  friend class KPP;
  typedef SlidingMedian<uint16_t> SM;

  enum State{Not, OtLeftV, OtRightV, BfLeftV, BfRightV, ForwardV, ReverseV, OneV, TwoV, ThreeV};

  explicit Calibrate(const uint8_t sizef = 9):
    Left(sizef),
    Right(sizef),
    Throt(sizef),
    Brake(sizef),
    Decel(sizef),
    Temp(sizef),
    OtL(sizef),
    OtR(sizef),
    BfL(sizef),
    BfR(sizef),
    F(sizef),
    R(sizef),
    One(sizef),
    Two(sizef),
    Three(sizef),
    PresFilter(7),
    d_generator(0),
    old_direct(0),
    oil_filter(0),
    parking_ch(0),
    direction(0), 
    clutch_st(0),
    reverse(0),  
    start_eng(0),
    clutch_ch(0), 
    parking(1),  
    direct_ch(0),
    clutch(0) { FlashRead(); }
  Calibrate(const Calibrate&)            = delete;
  Calibrate(Calibrate&&)                 = delete;
  Calibrate& operator=(const Calibrate&) = delete;
  Calibrate& operator=(Calibrate&&)      = delete;

  void RemoteCtrlAndRPM(uint8_t, uint16_t);
  void OtLeftTime(CanRxMsg&);
  void OtLeftPres(CanRxMsg&);
  void OtRightTime(CanRxMsg&);
  void OtRightPres(CanRxMsg&);
  void BfLeftTime(CanRxMsg&);
  void BfLeftPres(CanRxMsg&);
  void BfRightTime(CanRxMsg&);
  void BfRightPres(CanRxMsg&);
  void ForwardTime(CanRxMsg&);
  void ForwardPres(CanRxMsg&);
  void ReverseTime(CanRxMsg&);
  void ReversePres(CanRxMsg&);
  void OneTime(CanRxMsg&);
  void OnePres(CanRxMsg&);
  void TwoTime(CanRxMsg&);
  void TwoPres(CanRxMsg&);
  void ThreeTime(CanRxMsg&);
  void ThreePres(CanRxMsg&);
  void Valve(State&, Pressure);
  void Save();
  ~Calibrate()                           = default;
private:
  void FlashWrite();
  void FlashRead();

  static const uint32_t address = 0x08060000;
  
  struct Data
  {
    std::pair<uint16_t, uint16_t> OtLeftTimePres[8];
    std::pair<uint16_t, uint16_t> OtRightTimePres[8];
    std::pair<uint16_t, uint16_t> BfLeftTimePres[8];
    std::pair<uint16_t, uint16_t> BfRightTimePres[8];
    std::pair<uint16_t, uint16_t> ForwardTimePres[8];
    std::pair<uint16_t, uint16_t> ReverseTimePres[8];
    std::pair<uint16_t, uint16_t> OneTimePres[8];
    std::pair<uint16_t, uint16_t> TwoTimePres[8];
    std::pair<uint16_t, uint16_t> ThreeTimePres[8];
    std::pair<uint16_t, uint16_t> AnalogRemoteCtrlAndRPM[6];// Rud; Left; Right; Brake; Decl; RPM;
    //номер каждого элемента массива соответствует значению регистра таймера умноженного на 4, в каждом массиве храним значение давления умноженное на 10.
    std::array<std::array<uint16_t, 125>, 9> Valve;//OTl, OTr, BFl, BFr, F, R, One, Two, Three
  }d;

  SM Left, Right, Throt, Brake, Decel, Temp, OtL, OtR, BfL, BfR, F, R, One, Two, Three, PresFilter;

  const uint8_t koef = 4;

  uint8_t direction  : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  uint8_t clutch_st  : 2;//00 - none, 01 - '+', 10 - '-',  11 - Not available
  uint8_t parking    : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t reverse    : 2;//00 - off,  01 - on,  10 - res,  11 - Don't care
  uint8_t clutch     : 2;//00 - 0,    01 - 1,   10 - 2,    11 - 3
  uint8_t old_direct : 2;//00 - N,    01 - F,   10 - R,    11 - Not available
  uint8_t oil_filter : 1;//true/false
  uint8_t d_generator: 1;//true/false
  uint8_t start_eng  : 1;//true/false
  uint8_t parking_ch : 1;//true/false
  uint8_t clutch_ch  : 1;//true/false
  uint8_t direct_ch  : 1;//true/false
};

inline void Calibrate::Save() { FlashWrite(); }
inline void Calibrate::OtLeftTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtLeftTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OtLeftPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtLeftTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::OtRightTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtRightTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OtRightPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OtRightTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfLeftTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfLeftTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfLeftPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfLeftTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfRightTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfRightTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::BfRightPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.BfRightTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::ForwardTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ForwardTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ForwardPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ForwardTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::ReverseTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ReverseTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ReversePres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ReverseTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::OneTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OneTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::OnePres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.OneTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::TwoTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.TwoTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::TwoPres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.TwoTimePres[i].second = RxMsg.Data[i] * koef;
}
inline void Calibrate::ThreeTime(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ThreeTimePres[i].first = RxMsg.Data[i] * koef;
}
inline void Calibrate::ThreePres(CanRxMsg& RxMsg)
{
  for(uint8_t i = 0; i < RxMsg.DLC; ++i)
    d.ThreeTimePres[i].second = RxMsg.Data[i] * koef;
}

//все методы класса необходимо пересмотреть после создания общей структуры!!!
class KPP
{
public:
  KPP()                       = default;
  KPP(const KPP&)             = delete;
  KPP(KPP&&)                  = delete;
  KPP& operator= (const KPP&) = delete;
  KPP& operator= (KPP&&)      = delete;
  ~KPP()                      = default;
  //logic control
  void SetAllOt()                    const;
  void ResetAllOt()                  const;
  void Brake(const uint8_t)          const;
  void ResetAllValve()               const;
  void SetAllBf()                    const;
  void ResetAllClutch()              const;
  void SetDirection(const uint8_t)   const;
  void ResetDirection(const uint8_t) const;
  void ResetAllDirect()              const;
  void BrakeRotate(Calibrate&);
  void SwitchDirection(Calibrate&);
  void Parking(Calibrate&);
  void SetClutch(Calibrate&);
  //Work with the data
  void DigitalSet(const uint16_t, Calibrate&);
  void AnalogSet(const uint16_t*, Calibrate&);
  void CurrentSet(const uint16_t*, Calibrate&);
  void Send(Calibrate&);
  void SendData(Calibrate&);
  void RequestRpm(const uint16_t) const;
  void SetRpm(const uint16_t);
  //test
  void SendDataValve(Calibrate&);
private:
  void Send(CanTxMsg&, std::pair<uint16_t, uint16_t>*, Calibrate&);
  void PropBrakeR(const uint8_t)                   const;
  void PropBrakeL(const uint8_t)                   const;
  void PropSetOtL(const uint8_t)                   const;
  void PropSetOtR(const uint8_t)                   const;
  void OnClutch(Calibrate& cal)                    const;
  void OffClutch(Calibrate& cal)                   const;
  void RightUp(uint8_t, uint8_t, uint8_t, uint8_t) const;
  void RightDown(uint8_t, uint8_t)                 const;
  void LeftUp(uint8_t, uint8_t, uint8_t, uint8_t)  const;
  void LeftDown(uint8_t, uint8_t)                  const;
  void SetOtL(const uint8_t)                       const;
  void SetOtR(const uint8_t)                       const;
  void ResetOtL()                                  const;
  void ResetOtR()                                  const;
  void ResetFirst()                                const;
  void ResetSecond()                               const;
  void ResetThird()                                const;
  void ResetForward()                              const;
  void ResetReverse()                              const;
  void ResetBfL()                                  const;
  void ResetBfR()                                  const;
  void SetOtL()                                    const;
  void SetOtR()                                    const;
  void SetBfL()                                    const;
  void SetBfR()                                    const;
  void SetFirst()                                  const;
  void SetSecond()                                 const;
  void SetThird()                                  const;
  void SetForward()                                const;
  void SetReverse()                                const;

  uint16_t       rpm    = 0;
  const uint16_t maxpwm = 500;
  const uint8_t  minpwm = 0;
  const uint8_t  resol  = 8;
};

//inline методы должны быть включены в каждую трансляцию, так что лучше их определять в заголовке.
inline void KPP::ResetOtL() const     { TIM_SetCompare1(TIM4, maxpwm); }//ОТ выключен
inline void KPP::SetOtL() const       { TIM_SetCompare1(TIM4, minpwm); }//ОТ включен
inline void KPP::ResetOtR() const     { TIM_SetCompare2(TIM4, maxpwm); }//ОТ выключен
inline void KPP::SetOtR() const       { TIM_SetCompare2(TIM4, minpwm); }//ОТ включен
// 500 - data * 500 / 100
inline void KPP::SetOtL(const uint8_t d) const { TIM_SetCompare1(TIM4, 500 - d * 5); }
inline void KPP::SetOtR(const uint8_t d) const { TIM_SetCompare2(TIM4, 500 - d * 5); }

inline void KPP::SetBfL() const       { TIM_SetCompare3(TIM4, maxpwm); }
inline void KPP::ResetBfL() const     { TIM_SetCompare3(TIM4, minpwm); }
inline void KPP::SetBfR() const       { TIM_SetCompare4(TIM4, maxpwm); }
inline void KPP::ResetBfR() const     { TIM_SetCompare4(TIM4, minpwm); }

inline void KPP::SetFirst() const     { TIM_SetCompare1(TIM3, maxpwm); }
inline void KPP::ResetFirst() const   { TIM_SetCompare1(TIM3, minpwm); }
inline void KPP::SetSecond() const    { TIM_SetCompare2(TIM3, maxpwm); }
inline void KPP::ResetSecond() const  { TIM_SetCompare2(TIM3, minpwm); }
inline void KPP::SetThird() const     { TIM_SetCompare3(TIM3, maxpwm); }
inline void KPP::ResetThird() const   { TIM_SetCompare3(TIM3, minpwm); }

inline void KPP::SetForward() const   { TIM_SetCompare4(TIM3, maxpwm); }
inline void KPP::ResetForward() const { TIM_SetCompare4(TIM3, minpwm); }
inline void KPP::SetReverse() const   { TIM_SetCompare2(TIM1, maxpwm); }
inline void KPP::ResetReverse() const { TIM_SetCompare2(TIM1, minpwm); }

inline void KPP::SetRpm(const uint16_t x) { rpm = x / resol; };
#endif /* __KPP */