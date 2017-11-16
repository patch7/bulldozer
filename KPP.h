#ifndef __KPP
#define __KPP

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_flash.h"
#include <utility>
#include "sliding_median.h"
#include "engine.h"

const static uint8_t N = 0;
const static uint8_t F = 1;
const static uint8_t R = 2;

const static uint8_t PLUS  = 1;
const static uint8_t MINUS = 2;

const static uint8_t OFF = 0;
const static uint8_t ON  = 1;

bool CanTxMailBox_IsEmpty(CAN_TypeDef*);

class Calibrate//Singleton by transform Scott Meyers.
{
public:
  friend class KPP;
  typedef SlidingMedian SM;

  enum State{Not, OtLeftV, OtRightV, BfLeftV, BfRightV, ForwardV, ReverseV, OneV, TwoV, ThreeV};

  Calibrate(const Calibrate&)            = delete;
  Calibrate(Calibrate&&)                 = delete;
  Calibrate& operator=(const Calibrate&) = delete;
  Calibrate& operator=(Calibrate&&)      = delete;
  static Calibrate& getInstance();

  void RemoteCtrl(uint8_t, SM&, SM&, SM&, SM&, SM&);
  void SetOtLeftTime(CanRxMsg&);
  void SetOtLeftCur(CanRxMsg&);
  void SetOtRightTime(CanRxMsg&);
  void SetOtRightCur(CanRxMsg&);
  void SetBfLeftTime(CanRxMsg&);
  void SetBfLeftCur(CanRxMsg&);
  void SetBfRightTime(CanRxMsg&);
  void SetBfRightCur(CanRxMsg&);
  void SetForwardTime(CanRxMsg&);
  void SetForwardCur(CanRxMsg&);
  void SetReverseTime(CanRxMsg&);
  void SetReverseCur(CanRxMsg&);
  void SetOneTime(CanRxMsg&);
  void SetOneCur(CanRxMsg&);
  void SetTwoTime(CanRxMsg&);
  void SetTwoCur(CanRxMsg&);
  void SetThreeTime(CanRxMsg&);
  void SetThreeCur(CanRxMsg&);
  void SendData();
  void Save(const Calibrate& inst);
  void SetOtLeftValve(State&);
  void SetOtRightValve(State&);
  void SetBfLeftValve(State&);
  void SetBfRightValve(State&);
  void SetForwardValve(State&);
  void SetReverseValve(State&);
  void SetOneValve(State&);
  void SetTwoValve(State&);
  void SetThreeValve(State&);
  void CurrentSet(const uint16_t*);
  ~Calibrate()                           = default;
private:
  Calibrate()                            = default;
  void Send(CanTxMsg&, std::pair<uint16_t, uint16_t>*);

  static void FlashWrite(const Calibrate&);
  static void FlashRead(Calibrate&);

  static const uint32_t address = 0x08060000;
  const uint8_t koef            = 4;
  
  struct Data
  {
    std::pair<uint16_t, uint16_t> OtLeftTimeCur[8];
    std::pair<uint16_t, uint16_t> OtRightTimeCur[8];
    std::pair<uint16_t, uint16_t> BfLeftTimeCur[8];
    std::pair<uint16_t, uint16_t> BfRightTimeCur[8];
    std::pair<uint16_t, uint16_t> ForwardTimeCur[8];
    std::pair<uint16_t, uint16_t> ReverseTimeCur[8];
    std::pair<uint16_t, uint16_t> OneTimeCur[8];
    std::pair<uint16_t, uint16_t> TwoTimeCur[8];
    std::pair<uint16_t, uint16_t> ThreeTimeCur[8];

    //номер каждого элемента массива соответствует значению регистра таймера умноженного на 2.
    std::pair<uint16_t, uint16_t> OtLeftValve[250];
    std::pair<uint16_t, uint16_t> OtRightValve[250];
    std::pair<uint16_t, uint16_t> BfLeftValve[250];
    std::pair<uint16_t, uint16_t> BfRightValve[250];
    std::pair<uint16_t, uint16_t> FValve[250];
    std::pair<uint16_t, uint16_t> RValve[250];
    std::pair<uint16_t, uint16_t> OneValve[250];
    std::pair<uint16_t, uint16_t> TwoValve[250];
    std::pair<uint16_t, uint16_t> ThreeValve[250];

    std::pair<uint16_t, uint16_t> AnalogRemoteCtrl[5];// Rud; Left; Right; Brake; Decl;
  }d;

  SlidingMedian OtL, OtR, BfL, BfR, F, R, One, Two, Three;
};

inline void Calibrate::Save(const Calibrate& inst) { FlashWrite(inst); }
inline void Calibrate::SetOtLeftTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OtLeftTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetOtLeftCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OtLeftTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetOtRightTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OtRightTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetOtRightCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OtRightTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetBfLeftTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.BfLeftTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetBfLeftCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.BfLeftTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetBfRightTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.BfRightTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetBfRightCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.BfRightTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetForwardTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ForwardTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetForwardCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ForwardTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetReverseTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ReverseTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetReverseCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ReverseTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetOneTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OneTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetOneCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.OneTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetTwoTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.TwoTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetTwoCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.TwoTimeCur[i].second = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetThreeTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ThreeTimeCur[i].first = RxMessage.Data[i] * koef;
}
inline void Calibrate::SetThreeCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    d.ThreeTimeCur[i].second = RxMessage.Data[i] * koef;
}

//все методы класса необходимо пересмотреть после создания общей структуры!!!
class KPP
{
public:
  KPP(const uint8_t size_filter = 9) : SMleft(size_filter),     SMright(size_filter),
                                       SMthrottle(size_filter), SMbrake(size_filter),
                                       SMdeceler(size_filter),  SMtemp(size_filter),
                                       direction(0),            clutch_state(0),
                                       parking(1),              auto_reverse(0),
                                       clutch(0),               old_direct(0),
                                       oil_filter(0),           d_generator(0),
                                       start_eng(0),            parking_ch(0),
                                       clutch_ch(0),            direct_ch(0) {}
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

  SlidingMedian& GetT();
  SlidingMedian& GetL();
  SlidingMedian& GetR();
  SlidingMedian& GetB();
  SlidingMedian& GetD();
private:
  void PropBrakeR(const uint8_t)                   const;
  void PropBrakeL(const uint8_t)                   const;
  void PropSetOtL(const uint8_t)                   const;
  void PropSetOtR(const uint8_t)                   const;
  void OnClutch()                                  const;
  void OffClutch()                                 const;
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

  CanTxMsg TxMessage;

  SlidingMedian SMleft, SMright, SMthrottle, SMbrake, SMdeceler, SMtemp;

  uint8_t BfLcount = 0;
  uint8_t BfRcount = 0;

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

  //нужно для пропорционального управления клапаном по графику!!!
  //bool SetBfL = false;
  //bool SetBfR = false;
};

//inline методы должны быть включены в каждую трансляцию, так что лучше их определять в заголовке.
inline SlidingMedian& KPP::GetT() { return SMthrottle; }
inline SlidingMedian& KPP::GetL() { return SMleft; }
inline SlidingMedian& KPP::GetR() { return SMright; }
inline SlidingMedian& KPP::GetB() { return SMbrake; }
inline SlidingMedian& KPP::GetD() { return SMdeceler; }

inline void KPP::ResetOtL() const     { TIM_SetCompare1(TIM4, 500); }
inline void KPP::SetOtL() const       { TIM_SetCompare1(TIM4, 0); }
inline void KPP::ResetOtR() const     { TIM_SetCompare2(TIM4, 500); }
inline void KPP::SetOtR() const       { TIM_SetCompare2(TIM4, 0); }
//500 - data * 500 / 100
inline void KPP::SetOtL(const uint8_t d) const {TIM_SetCompare1(TIM4,500-d*5);}
inline void KPP::SetOtR(const uint8_t d) const {TIM_SetCompare2(TIM4,500-d*5);}

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
#endif /* __KPP */