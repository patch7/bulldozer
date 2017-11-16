#include "KPP.h"

bool CanTxMailBox_IsEmpty(CAN_TypeDef* CANx)
{
  if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0 || (CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1 ||
     (CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    return true;
  else
    return false;
}

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

void Calibrate::FlashWrite(const Calibrate& inst)//надо проверить
{
  const uint32_t* flash_address  = (uint32_t*)address;
  const uint32_t* source_address = (uint32_t*)&inst.d;
  FLASH_Unlock();
  FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3);
  for (uint16_t i = 0; i < sizeof(inst.d); i += 4)
    FLASH_ProgramWord((uint32_t)flash_address++, *source_address++);
  FLASH_Lock();
}
void Calibrate::FlashRead(Calibrate& inst)//надо проверить
{
  const uint32_t* flash_address = (uint32_t*)address;
  uint32_t* source_address = (uint32_t*)&inst.d;
  for (uint16_t i = 0; i < sizeof(inst.d); i += 4)
    *source_address++ = *(__IO uint32_t*)flash_address++;
}
void Calibrate::RemoteCtrl(uint8_t state, SM& throt, SM& l, SM& r, SM& brake, SM& dec)
{
  switch(state)
  {
    case 0x21: d.AnalogRemoteCtrl[0].first  = throt.get(); break;//min
    case 0x41: d.AnalogRemoteCtrl[0].second = throt.get(); break;//max
    case 0x22: d.AnalogRemoteCtrl[1].first  = l.get();     break;//min
    case 0x42: d.AnalogRemoteCtrl[1].second = l.get();     break;//max
    case 0x23: d.AnalogRemoteCtrl[2].first  = r.get();     break;//min
    case 0x43: d.AnalogRemoteCtrl[2].second = r.get();     break;//max
    case 0x24: d.AnalogRemoteCtrl[3].first  = brake.get(); break;//min
    case 0x44: d.AnalogRemoteCtrl[3].second = brake.get(); break;//max
    case 0x25: d.AnalogRemoteCtrl[4].first  = dec.get();   break;//min
    case 0x45: d.AnalogRemoteCtrl[4].second = dec.get();   break;//max
  }
}
void Calibrate::Send(CanTxMsg& TxMessage, std::pair<uint16_t, uint16_t>* data)
{
  TxMessage.Data[0] = (uint8_t)(data[0].first / koef);
  TxMessage.Data[1] = (uint8_t)(data[1].first / koef);
  TxMessage.Data[2] = (uint8_t)(data[2].first / koef);
  TxMessage.Data[3] = (uint8_t)(data[3].first / koef);
  TxMessage.Data[4] = (uint8_t)(data[4].first / koef);
  TxMessage.Data[5] = (uint8_t)(data[5].first / koef);
  TxMessage.Data[6] = (uint8_t)(data[6].first / koef);
  TxMessage.Data[7] = (uint8_t)(data[7].first / koef);
  while(!CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);

  ++TxMessage.StdId;

  TxMessage.Data[0] = (uint8_t)(data[0].second / koef);
  TxMessage.Data[1] = (uint8_t)(data[1].second / koef);
  TxMessage.Data[2] = (uint8_t)(data[2].second / koef);
  TxMessage.Data[3] = (uint8_t)(data[3].second / koef);
  TxMessage.Data[4] = (uint8_t)(data[4].second / koef);
  TxMessage.Data[5] = (uint8_t)(data[5].second / koef);
  TxMessage.Data[6] = (uint8_t)(data[6].second / koef);
  TxMessage.Data[7] = (uint8_t)(data[7].second / koef);
  while(!CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void Calibrate::SendData()
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;
  TxMessage.StdId = 0x1FD;

  for(uint8_t i = 0; i < 5; ++i, ++TxMessage.StdId)
  {
    TxMessage.Data[0] = (uint8_t)(d.AnalogRemoteCtrl[i].first);
    TxMessage.Data[1] = (uint8_t)(d.AnalogRemoteCtrl[i].first  >> 8);
    TxMessage.Data[2] = (uint8_t)(d.AnalogRemoteCtrl[i].second);
    TxMessage.Data[3] = (uint8_t)(d.AnalogRemoteCtrl[i].second  >> 8);
    if(++i < 5)
    {
      TxMessage.Data[4] = (uint8_t)(d.AnalogRemoteCtrl[i].first);
      TxMessage.Data[5] = (uint8_t)(d.AnalogRemoteCtrl[i].first >> 8);
      TxMessage.Data[6] = (uint8_t)(d.AnalogRemoteCtrl[i].second);
      TxMessage.Data[7] = (uint8_t)(d.AnalogRemoteCtrl[i].second >> 8);
    }
    else
    {
      TxMessage.Data[4] = 0;
      TxMessage.Data[5] = 0;
      TxMessage.Data[6] = 0;
      TxMessage.Data[7] = 0;
    }
    while(!CanTxMailBox_IsEmpty(CAN2));
    CAN_Transmit(CAN2, &TxMessage);
  }

  TxMessage.StdId = 0x200;
  Send(TxMessage, d.OtLeftTimeCur);
  TxMessage.StdId = 0x202;
  Send(TxMessage, d.OtRightTimeCur);
  TxMessage.StdId = 0x204;
  Send(TxMessage, d.BfLeftTimeCur);
  TxMessage.StdId = 0x206;
  Send(TxMessage, d.BfRightTimeCur);
  TxMessage.StdId = 0x208;
  Send(TxMessage, d.ForwardTimeCur);
  TxMessage.StdId = 0x20A;
  Send(TxMessage, d.ReverseTimeCur);
  TxMessage.StdId = 0x20C;
  Send(TxMessage, d.OneTimeCur);
  TxMessage.StdId = 0x20E;
  Send(TxMessage, d.TwoTimeCur);
  TxMessage.StdId = 0x210;
  Send(TxMessage, d.ThreeTimeCur);
}
Calibrate& Calibrate::getInstance()
{
  static Calibrate instance;
  FlashRead(instance);
  return instance;
}
void Calibrate::CurrentSet(const uint16_t* data)
{
  OtL.push(data[0]);
  OtR.push(data[1]);
  BfL.push(data[2]);
  BfR.push(data[3]);
  F.push(data[4]);
  R.push(data[5]);
  One.push(data[6]);
  Two.push(data[7]);
  Three.push(data[8]);
}
void Calibrate::SetOtLeftValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.OtLeftValve[count - 1].first  = OtL.get();
    d.OtLeftValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      state = Not;
      TIM_SetCompare1(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare1(TIM4, 2 + count * 2);
  ++count;
}
void Calibrate::SetOtRightValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.OtRightValve[count - 1].first  = OtR.get();
    d.OtRightValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare2(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM4, 2 + count * 2);
  ++count;
}
void Calibrate::SetBfLeftValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.BfLeftValve[count - 1].first  = BfL.get();
    d.BfLeftValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare3(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare3(TIM4, 2 + count * 2);
  ++count;
}
void Calibrate::SetBfRightValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.BfRightValve[count - 1].first  = BfR.get();
    d.BfRightValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare4(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare4(TIM4, 2 + count * 2);
  ++count;
}
void Calibrate::SetForwardValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.FValve[count - 1].first  = F.get();
    d.FValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare4(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare4(TIM3, 2 + count * 2);
  ++count;
}
void Calibrate::SetReverseValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.RValve[count - 1].first  = F.get();
    d.RValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare2(TIM1, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM1, 2 + count * 2);
  ++count;
}
void Calibrate::SetOneValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.OneValve[count - 1].first  = One.get();
    d.OneValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare1(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare1(TIM3, 2 + count * 2);
  ++count;
}
void Calibrate::SetTwoValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.TwoValve[count - 1].first  = Two.get();
    d.TwoValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare2(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM3, 2 + count * 2);
  ++count;
}
void Calibrate::SetThreeValve(State& state)
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)
  {
    d.ThreeValve[count - 1].first  = Three.get();
    d.ThreeValve[count - 1].second = 0;//текущее давление
    if(count == 250)
    {
      //state = false;
      TIM_SetCompare3(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare3(TIM3, 2 + count * 2);
  ++count;
}