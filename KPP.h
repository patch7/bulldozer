#ifndef __KPP
#define __KPP

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_flash.h"
#include "sliding_median.h"
#include "engine.h"

const static uint8_t N = 0;
const static uint8_t F = 1;
const static uint8_t R = 2;

const static uint8_t PLUS  = 1;
const static uint8_t MINUS = 2;

const static uint8_t OFF = 0;
const static uint8_t ON  = 1;

class Calibrate
{
public:
  friend class KPP;
  typedef SlidingMedian SM;
  Calibrate()                            = delete;
  Calibrate(const Calibrate&)            = delete;
  Calibrate(Calibrate&&)                 = delete;
  Calibrate& operator=(const Calibrate&) = delete;
  Calibrate& operator=(Calibrate&&)      = delete;
  static void instanse();

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
  void Save();
  ~Calibrate()                           = default;
private:
  void FlashWrite();
  void FlashRead();
  //надо сделать определение указателя на память с калибровками в одном месте. Тут!
  uint16_t OtLeftTime[8]  = {0};
  uint16_t OtLeftCur[8]   = {0};
  uint16_t OtRightTime[8] = {0};
  uint16_t OtRightCur[8]  = {0};
  uint16_t BfLeftTime[8]  = {0};
  uint16_t BfLeftCur[8]   = {0};
  uint16_t BfRightTime[8] = {0};
  uint16_t BfRightCur[8]  = {0};
  uint16_t ForwardTime[8] = {0};
  uint16_t ForwardCur[8]  = {0};
  uint16_t ReverseTime[8] = {0};
  uint16_t ReverseCur[8]  = {0};
  uint16_t OneTime[8]     = {0};
  uint16_t OneCur[8]      = {0};
  uint16_t TwoTime[8]     = {0};
  uint16_t TwoCur[8]      = {0};
  uint16_t ThreeTime[8]   = {0};
  uint16_t ThreeCur[8]    = {0};

  uint16_t RudMin     = 0;
  uint16_t RudMax     = 0;
  uint16_t LeftMin    = 0;
  uint16_t LeftMax    = 0;
  uint16_t RightMin   = 0;
  uint16_t RightMax   = 0;
  uint16_t BrakeMin   = 0;
  uint16_t BrakeMax   = 0;
  uint16_t DecelerMin = 0;
  uint16_t DecelerMax = 0;
};

inline void SetOtLeftTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OtLeftTime[i] = RxMessage.Data[i]
}
inline void SetOtLeftCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OtLeftCur[i] = RxMessage.Data[i]
}
inline void SetOtRightTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OtRightTime[i] = RxMessage.Data[i]
}
inline void SetOtRightCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OtRightCur[i] = RxMessage.Data[i]
}
inline void SetBfLeftTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    BfLeftTime[i] = RxMessage.Data[i]
}
inline void SetBfLeftCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    BfLeftCur[i] = RxMessage.Data[i]
}
inline void SetBfRightTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    BfRightTime[i] = RxMessage.Data[i]
}
inline void SetBfRightCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    BfRightCur[i] = RxMessage.Data[i]
}
inline void SetForwardTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ForwardTime[i] = RxMessage.Data[i]
}
inline void SetForwardCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ForwardCur[i] = RxMessage.Data[i]
}
inline void SetReverseTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ReverseTime[i] = RxMessage.Data[i]
}
inline void SetReverseCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ReverseCur[i] = RxMessage.Data[i]
}
inline void SetOneTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OneTime[i] = RxMessage.Data[i]
}
inline void SetOneCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    OneCur[i] = RxMessage.Data[i]
}
inline void SetTwoTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    TwoTime[i] = RxMessage.Data[i]
}
inline void SetTwoCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    TwoCur[i] = RxMessage.Data[i]
}
inline void SetThreeTime(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ThreeTime[i] = RxMessage.Data[i]
}
inline void SetThreeCur(CanRxMsg& RxMessage)
{
  for(uint8_t i = 0; i < RxMessage.DLC; ++i)
    ThreeCur[i] = RxMessage.Data[i]
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