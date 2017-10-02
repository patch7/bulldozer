#include "KPP.h"

void DigitalSignals::Set(const uint16_t d)
{
  if(parking != (0x0003 & (d >> 4)))
    parking_ch = true;
  else
    parking_ch = false;
  
  direction    = 0x0003 & d;
  clutch_state = 0x0003 & (d >> 2);
  parking      = 0x0003 & (d >> 4);
  auto_reverse = 0x0003 & (d >> 6);
  oil_filter   = 0x0001 & (d >> 8);
  d_generator  = 0x0001 & (d >> 9);

  if(old_direct != direction)
  {
    direct_ch  = true;
    old_direct = direction;
  }
  else
    direct_ch = false;

  if(clutch_state == 0)
    clutch_ch = false;
  else
    clutch_ch = true;

  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.StdId   = 0x010;
  TxMessage.DLC     = 8;
  TxMessage.Data[0] = direct_ch;
  TxMessage.Data[1] = old_direct;
  TxMessage.Data[2] = direction;
  TxMessage.Data[3] = parking_ch;
  TxMessage.Data[4] = parking;
  TxMessage.Data[5] = clutch_ch;
  TxMessage.Data[6] = clutch_state;
  TxMessage.Data[7] = clutch;
  CAN_Transmit(CAN2, &TxMessage);
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
  TxMessage.Data[6] = (uint8_t)(d.clutch << 1) | d.start_eng;
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
   TIM_SetCompare1(TIM4, 500 - d * 5);//500 - d * 500 / 100  OTл
   TIM_SetCompare2(TIM4, 500 - d * 5);//500 - d * 500 / 100  ОТп
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

void KPP::SetAllOt() const  
{
  SetOtL(); 
  SetOtR(); 
}
void KPP::ResetAllOt() const
{
  ResetOtL();
  ResetOtR(); 
}
void KPP::SetAllBf() const  
{
  SetBfL(); 
  SetBfR(); 
}
void KPP::ResetAllClutch() const
{
  ResetFirst();
  ResetSecond();
  ResetThird();
}

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

void KPP::ResetAllDirect() const
{
  ResetForward();
  ResetReverse();
}
