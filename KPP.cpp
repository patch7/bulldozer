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
void KPP::Parking(const uint16_t rpm)
{
  if(parking == ON && parking_ch)
  {
    ResetAllValve();
    clutch = 0;
  }
  else if(parking == OFF && parking_ch)
  {
    SetBfL();
    SetBfR();
    if(rpm > 350)
    {
      clutch = 1;
      OnClutch();
    }
  }
}
void KPP::ResetAllValve() const
{
  SetOtL();
  SetOtR();
  ResetBfL();
  ResetBfR();
  ResetFirst();
  ResetSecond();
  ResetThird();
  ResetForward();
  ResetReverse();
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
  {//за счет времени спада давления в бустере передачи, алгоритм будет соответствовать ТТ, т.е. включается необходимая передача и через 100 мс выключается предыдущая.
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
void KPP::SwitchDirection(Engine& eng)
{
  if(direct_ch && parking == OFF)
  {
    if(direction == N)
    {
      ResetForward();
      ResetReverse();
    }
    else if(direction == F)
      ResetReverse();
    else if(direction == R)
      ResetForward();

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
void KPP::RightUp(uint8_t begin, uint8_t end,uint8_t right,uint8_t brake) const
{
  ResetBfR();//нужна задержка чтобы масло успело слиться

  if(right > begin && right < end)
  {
    if(brake <= right)
      SetOtR(right);
    else if(brake > right && brake < end)
      SetOtR(brake);
    else if(brake >= end)
      SetOtR();
  }
  else if(right >= end)
    SetOtR();
}
void KPP::RightDown(uint8_t begin, uint8_t brake) const
{
  if(brake <= begin)
    ResetOtR();
  else
    SetOtR(brake);

  SetBfR();//должно меняться пропорционально по графику вкл. SetBfR(right)
}
void KPP::LeftUp(uint8_t begin, uint8_t end, uint8_t left, uint8_t brake) const
{
  ResetBfL();//нужна задержка чтобы масло успело слиться

  if(left > begin && left < end)
  {//приоритет у органа управления, который сильнее тормозит
    if(brake <= left)
      SetOtL(left);
    else if(brake > left && brake < end)
      SetOtL(brake);
    else if(brake >= end)
      SetOtL();
  }
  else if(left >= end)
    SetOtL();
}
void KPP::LeftDown(uint8_t begin, uint8_t brake) const
{
  if(brake <= begin)
    ResetOtL();
  else
    SetOtL(brake);

  SetBfL();//должно меняться пропорционально по графику вкл. SetBfR(right)
}
void KPP::BrakeRotate()
{
  static uint8_t old_left  = 0;
  static uint8_t old_right = 0;
  
  bool left_up    = false;
  bool left_down  = false;
  bool right_up   = false;
  bool right_down = false;

  uint8_t left  = SMleft.get()  * 100 / 4095;
  uint8_t right = SMright.get() * 100 / 4095;
  uint8_t brake = SMbrake.get() * 100 / 4095;

  if(old_left + 1 < left)//левый джойстик
  {// +1 для избежания постоянного переключения из-за дрожания руки.
    left_up   = true;
    left_down = false;
  }
  else if(old_left - 1 > left)
  {// -1 для избежания постоянного переключения из-за дрожания руки.
    left_down = true;
    left_up   = false;
  }

  if(old_right + 1 < right)//правый джойстик
  {// +1 для избежания постоянного переключения из-за дрожания руки.
    right_up   = true;
    right_down = false;
  }
  else if(old_right - 1 > right)
  {// -1 для избежания постоянного переключения из-за дрожания руки.
    right_down = true;
    right_up   = false;
  }

  old_left  = left;
  old_right = right;

  if(parking == OFF)
  {
    const uint8_t begin = 10;
    const uint8_t end   = 90;

    if(left <= begin)//приоритет у органа управления, который сильнее тормозит
    {
      if(brake <= begin)
        ResetOtL();//нужна задержка чтобы масло успело слиться
      else if(brake > begin && brake < end)
        SetOtL(brake);
      else if(brake >= end)
        SetOtL();
    }
    if(right <= begin)
    {
      if(brake <= begin)
        ResetOtR();//нужна задержка чтобы масло успело слиться
      else if(brake > begin && brake < end)
        SetOtR(brake);
      else if(brake >= end)
        SetOtR();
    }

    if(left_up)
    {
      LeftUp(begin, end, left, brake);
      if(right_up)
        RightUp(begin, end, right, brake);
      else if(right_down)
        RightDown(begin, brake);
    }
    else if(left_down)
    {
      LeftDown(begin, brake);
      if(right_up)
        RightUp(begin, end, right, brake);
      else if(right_down)
        RightDown(begin, brake);
    }
    else// if(!left_up && !left_down)
      if(right_up)
        RightUp(begin, end, right, brake);
      else if(right_down)
        RightDown(begin, brake);
  }
}
void KPP::FlashWrite()//проверить, указатель может увеличиваться до бесконечности!!!
{
  const uint32_t* source_address = (uint32_t*)&calib;
  FLASH_Unlock();
  for (uint16_t i = 0; i < sizeof(calib); i += 4)
    FLASH_ProgramWord((uint32_t)flash_address++, *source_address++);
  FLASH_Lock();
}
void KPP::FlashRead()//проверить, указатель может увеличиваться до бесконечности!!!
{
  const uint32_t* source_address = (uint32_t*)&calib;
  for (int i = 0; i < sizeof(calib); i += 4)
    *source_address++ = *(__IO uint32_t*)flash_address++;
}
void KPP::Calibrate(CanRxMsg& RxMessage)
{
  if(RxMessage.Data[0] & 0x01)//Rud
  {
    if(RxMessage.Data[0] & 0x20)
      calib.RudMin = SMthrottle.get() - 5;
    else if(RxMessage.Data[0] & 0x40)
      calib.RudMax = SMthrottle.get() + 5;
  }
  else if(RxMessage.Data[0] & 0x02)//Left
  {
    if(RxMessage.Data[0] & 0x20)
      calib.LeftMin = SMleft.get() - 5;
    else if(RxMessage.Data[0] & 0x40)
      calib.LeftMax = SMleft.get() + 5;
  }
  else if(RxMessage.Data[0] & 0x04)//Right
  {
    if(RxMessage.Data[0] & 0x20)
      calib.RightMin = SMright.get() - 5;
    else if(RxMessage.Data[0] & 0x40)
      calib.RightMax = SMright.get() + 5;
  }
  else if(RxMessage.Data[0] & 0x08)//Brake
  {
    if(RxMessage.Data[0] & 0x20)
      calib.BrakeMin = SMbrake.get() - 5;
    else if(RxMessage.Data[0] & 0x40)
      calib.BrakeMax = SMbrake.get() + 5;
  }
  else if(RxMessage.Data[0] & 0x10)//Decel
  {
    if(RxMessage.Data[0] & 0x20)
      calib.DecelerateMin = SMdeceler.get() - 5;
    else if(RxMessage.Data[0] & 0x40)
      calib.DecelerateMax = SMdeceler.get() + 5;
  }
  else if(RxMessage.Data[1] & 0x01)//FlashWrite
    kpp.FlashWrite();
}