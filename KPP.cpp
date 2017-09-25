#include "KPP.h"

void DigitalSignals::Set(const uint8_t d)
{
  if(parking != (0x03 & (d >> 2)))
    parking_ch = true;
  else
    parking_ch = false;
  
  old_direct   = direction;
  direction    = 0x03 & (d >> 6);//надо пменять местами, сначало меньшие сдвиги
  clutch_state = 0x03 & (d >> 4);
  parking      = 0x03 & (d >> 2);
  auto_reverse = 0x03 & d;//надо переставить до условия со сдвигом
  
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

void KPP::Brake(const uint8_t d) const
{
   TIM_SetCompare1(TIM4, 500 - d * 5);//500 - d * 500 / 100  OT левое
   TIM_SetCompare2(TIM4, 500 - d * 5);//500 - d * 500 / 100  ОТ правое
}

void KPP::ResetAllValve() const
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

void KPP::SetAllOt() const    { SetOtL(); SetOtR(); }
void KPP::ResetAllOt() const  { ResetOtL(); ResetOtR(); }
void KPP::SetAllBf() const    { SetBfL(); SetBfR(); }

void KPP::SetClutch(const uint8_t d) const
{
  switch(d)
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

void KPP::ResetClutch(const uint8_t d) const
{
  switch(d)
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

void KPP::SetDirection(const uint8_t d) const
{
  if(d == 1)
    SetForward();
  else if(d == 2)
    SetReverse();
}

void KPP::ResetDirection(const uint8_t d) const
{
  if(d == 1)
    ResetForward();
  else if(d == 2)
    ResetReverse();
}
