#include "KPP.h"

void KPP::DigitalSet(const uint16_t data)
{
  if(parking != (0x0003 & (data >> 4)))
    parking_ch = true;
  else
    parking_ch = false;

  if(direction != (0x0003 & data))
    direct_ch  = true;
  else
    direct_ch  = false;
  
  direction    = 0x0003 & data;
  clutch_state = 0x0003 & (data >> 2);
  parking      = 0x0003 & (data >> 4);
  auto_reverse = 0x0003 & (data >> 6);
  oil_filter   = 0x0001 & (data >> 8);
  d_generator  = 0x0001 & (data >> 9);

  //debug
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
void KPP::AnalogSet(const uint16_t* data)
{
  SMleft.push(data[0]);
  SMright.push(data[1]);
  SMthrottle.push(data[2]);
  SMbrake.push(data[3]);
  SMdeceler.push(data[4]);
  SMtemp.push(data[5]);
}
void KPP::SendMsg()
{
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.StdId   = 0x002;
  TxMessage.DLC     = 7;
  TxMessage.Data[0] = SMleft.get()     * 100 / 4095;
  TxMessage.Data[1] = SMright.get()    * 100 / 4095;
  TxMessage.Data[2] = SMthrottle.get() * 100 / 4095;
  TxMessage.Data[3] = SMbrake.get()    * 100 / 4095;
  TxMessage.Data[4] = SMdeceler.get()  * 100 / 4095;
  TxMessage.Data[5] = SMtemp.get()     * 100 / 4095;
  TxMessage.Data[6] = (uint8_t)(clutch << 1) | start_eng;
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::Brake(const uint8_t data) const
{
  PropSetOtL(data);
  PropSetOtR(data);
}
void KPP::BrakeParking(const uint16_t rpm)
{
  if(parking == ON && parking_ch)
  {
    ResetAllValve();
    clutch = 0;
  }
  else if(parking == OFF)
  {//возможно торможение джойстиками надо будет перенести к поворотам!!!
    if(SMbrake.get() * 100 / 4095 <= 5)
      SetAllOt();
    else if(SMbrake.get() * 100 / 4095 >= 95)
      ResetAllOt();
    else
      Brake(SMbrake.get() * 100 / 4095);

    if(parking_ch)
    {
      SetAllBf();
      //ResetAllClutch();
      if(rpm > 350)
        SetClutch(clutch = 1);
    }
  }
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
void KPP::SetClutch(const uint16_t rpm)
{
  if(parking == OFF && clutch_state == PLUS && clutch < 3 && rpm > 350)
  {
    OffClutch();
    ++clutch;
    OnClutch();
    clutch_state = false;
  }
  else if(parking == OFF && clutch_state == MINUS && clutch > 1 && rpm > 350)
  {
    OffClutch();
    --clutch;
    OnClutch();
    clutch_state = false;
  }
  //debug
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.StdId   = 0x011;
  TxMessage.DLC     = 1;
  TxMessage.Data[0] = clutch;
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::OnClutch() const
{
  switch(clutch)
  {
  case 1: SetFirst();
    break;
  case 2: SetSecond();
    break;
  case 3: SetThird();
    break;
  }
}
void KPP::OffClutch() const
{
  switch(clutch)
  {
  case 1: ResetFirst();
    break;
  case 2: ResetSecond();
    break;
  case 3: ResetThird();
    break;
  }
}
void KPP::SetDirection(const uint8_t dir) const
{
  if(dir == F)
    SetForward();
  else if(dir == R)
    SetReverse();
}
void KPP::ResetDirection(const uint8_t dir) const
{
  if(dir == F)
    ResetForward();
  else if(dir == R)
    ResetReverse();
}
void KPP::ResetAllDirect() const
{
  ResetForward();
  ResetReverse();
}
void KPP::SwitchDirection(Engine& eng)
{
  if(direct_ch && parking == OFF)
  {
    if(direction == N)
      ResetAllDirect();
    else if(direction == F)
      ResetDirection(R);
    else if(direction == R)
      ResetDirection(F);

    if(direction)
    {
      uint16_t rpm = eng.GetRpm();
      if(rpm > 810)
      {
        eng.SetRpm(800);
        eng.RequestRpm();
      }
      SetDirection(direction);
      eng.SetRpm(rpm);
      eng.RequestRpm();
    }
  }
          
  if(direction == N && parking == ON && eng.GetRpm() < 350)
    start_eng = true;
  else
    start_eng = false;
}
void KPP::BrakeRotate()
{
  uint8_t left  = SMleft.get()  * 100 / 4095;
  uint8_t right = SMright.get() * 100 / 4095;
  uint8_t brake = SMbrake.get() * 100 / 4095;

  if(parking == OFF)
  {
    if(left > 5 && left < 95)
    {
      ResetBfL();
      PropSetOtL(left);
      PropBrake(brake);
    }
    else if(left >= 95)
    {
      ResetBfL();
      PropSetOtL(100);
      PropBrake(brake);
    }
    else if(brake <= 5)
      SetAllOt();
    else if(brake > 5 && brake < 95)
      Brake(brake);
    else if(brake >= 95)
      ResetAllOt();
  }
}
void KPP::PropBrake(const uint8_t brake) const
{
  if(brake <= 5)
    PropSetOtR(0);
  else if(brake > 5 && brake < 95)
    PropSetOtR(brake);
  else if(brake >= 95)
    PropSetOtR(100);
}