#include "KPP.h"

void DigitalSignals::Set(const uint8_t d)
{
  if(parking != (0x03 & (d >> 2)))
    parking_ch = true;
  else
    parking_ch = false;
  
  old_direct   = direction
  direction    = 0x03 & (d >> 6);
  clutch_state = 0x03 & (d >> 4);
  parking      = 0x03 & (d >> 2);
  auto_reverse = 0x03 & d;
  
  if(old_direct != direction)
    direct_ch = true;
  
  switch(clutch_state)
  {
  case 0://none
    clutch_ch = false;
    break;
  case 1://'+'
    if(clutch < 3)
    {
      ++clutch;
      clutch_ch = true;
    }
    break;
  case 2://'-'
    if(clutch > 1)
    {
      --clutch;
      clutch_ch = true;
    }
    break;
  }
}

void AnalogSignals::SendMsg(const DigitalSignals d)
{
  CanTxMsg TxMessage;
  
  TxMessage.RTR   = CAN_RTR_DATA;
  TxMessage.IDE   = CAN_ID_STD;
  
  TxMessage.StdId   = 0x002;
  TxMessage.DLC     = 7;
  TxMessage.Data[0] = SMleft.get()     * 100 / 4095;
  TxMessage.Data[1] = SMright.get()    * 100 / 4095;
  TxMessage.Data[2] = SMthrottle.get() * 100 / 4095;
  TxMessage.Data[3] = SMbrake.get()    * 100 / 4095;
  TxMessage.Data[4] = SMdeceler.get()  * 100 / 4095;
  TxMessage.Data[5] = SMtemp.get()     * 100 / 4095;
  TxMessage.Data[6] = d.clutch << 2 | d.start_eng;
  CAN_Transmit(CAN2, &TxMessage);
}

void AnalogSignals::Set(const uint16_t *data)
{
  SMleft.push(data[0]);
  SMright.push(data[1]);
  SMthrottle.push(data[2]);
  SMbrake.push(data[3]);
  SMdeceler.push(data[4]);
  SMtemp.push(data[5]);
}

void KPP::Forward()
{

}

void KPP::ResetAllValve()
{
  ResetOtL();
  ResetOtR();
  ResetBfL();
  ResetBfR();
  ResetFirst();
  ResetSecond();
  ResetThird();
  ResetForward();
  ResetReverse();
}

inline void KPP::SetAllOt()    { SetOtL(); SetOtR(); }
inline void KPP::SetAllBf()    { SetBfL(); SetBfR(); }

inline void KPP::SetClutch(const uint8_t d)
{
  switch(n)
  {
  case 1:
    SetFirst();
    break;
  case 2:
    SetSecond();
    break;
  case 3:
    SetThird();
    break;
  }
}

inline void KPP::ResetClutch(const uint8_t n)
{
  switch(n)
  {
  case 1:
    ResetFirst();
    break;
  case 2:
    ResetSecond();
    break;
  case 3:
    ResetThird();
    break;
  }
}

inline void KPP::SetDirection(const uint8_t d)
{
  if(d == 1)
    SetForward();
  else if(d == 2)
    SetReverse();
}

inline void KPP::ResetDirection(const uint8_t d)
{
  if(d == 1)
    ResetForward();
  else if(d == 2)
    ResetReverse();
}

inline void KPP::SetOtL()      { TIM_SetCompare1(TIM4, 500); }
inline void KPP::ResetOtL()    { TIM_SetCompare1(TIM4, 0); }

inline void KPP::SetOtR()      { TIM_SetCompare2(TIM4, 500); }
inline void KPP::ResetOtR()    { TIM_SetCompare2(TIM4, 0); }

inline void KPP::SetBfL()      { TIM_SetCompare3(TIM4, 500); }
inline void KPP::ResetBfL()    { TIM_SetCompare3(TIM4, 0); }

inline void KPP::SetBfR()      { TIM_SetCompare4(TIM4, 500); }
inline void KPP::ResetBfR()    { TIM_SetCompare4(TIM4, 0); }

inline void KPP::SetFirst()    { TIM_SetCompare1(TIM3, 500); }
inline void KPP::ResetFirst()  { TIM_SetCompare1(TIM3, 0); }

inline void KPP::SetSecond()   { TIM_SetCompare2(TIM3, 500); }
inline void KPP::ResetSecond() { TIM_SetCompare2(TIM3, 0); }

inline void KPP::SetThird()    { TIM_SetCompare3(TIM3, 500); }
inline void KPP::ResetThird()  { TIM_SetCompare3(TIM3, 0); }

inline void KPP::SetForward()  { TIM_SetCompare4(TIM3, 500); }
inline void KPP::ResetForward(){ TIM_SetCompare4(TIM3, 0); }

inline void KPP::SetReverse()  { TIM_SetCompare1(TIM1, 500); }
inline void KPP::ResetReverse(){ TIM_SetCompare1(TIM1, 0); }
