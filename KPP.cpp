#include "KPP.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
// \\            //  ////////  ////////  //    //       ///////       /\     ////////     /\     //
//  \\    /\    //      //        //     //    //       //    //     //\\       //       //\\    //
//   \\  //\\  //       //        //     ////////       //     //   //  \\      //      //  \\   //
//    \\//  \\//        //        //     //    //       //    //   //////\\     //     //////\\  //
//     \/    \/      ////////     //     //    //       //////    //      \\    //    //      \\ //
///////////////////////////////////////////////////////////////////////////////////////////////////
void KPP::DigitalSet(const uint16_t data, Calibrate& cal)//Good
{
  if(cal.parking != (0x0003 & (data >> 4)))
    cal.parking_ch = true;
  else
    cal.parking_ch = false;

  if(cal.direction != (0x0003 & data))
    cal.direct_ch  = true;
  else
    cal.direct_ch  = false;
  
  cal.direction   = 0x0003 & data;
  cal.clutch_st   = 0x0003 & (data >> 2);
  cal.parking     = 0x0003 & (data >> 4);
  cal.reverse     = 0x0003 & (data >> 6);
  cal.oil_filter  = 0x0001 & (data >> 8);
  cal.d_generator = 0x0001 & (data >> 9);
}
void KPP::AnalogSet(const uint16_t* data, Calibrate& cal)//Good
{
  cal.Left.push(data[0]);
  cal.Right.push(data[1]);
  cal.Throt.push(data[2]);
  cal.Brake.push(data[3]);
  cal.Decel.push(data[4]);
  cal.Temp.push(data[5]);
}
void KPP::CurrentSet(const uint16_t* data, Calibrate& cal)//Good
{
  cal.OtL.push(data[0]);
  cal.OtR.push(data[1]);
  cal.BfL.push(data[2]);
  cal.BfR.push(data[3]);
  cal.F.push(data[4]);
  cal.R.push(data[5]);
  cal.One.push(data[6]);
  cal.Two.push(data[7]);
  cal.Three.push(data[8]);
}
void KPP::Send(Calibrate& cal)//Good, надо исправить в соответствии с коментариями.
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  TxMessage.StdId   = 0x001;
  TxMessage.Data[0] = (uint8_t)(cal.Left.get());
  TxMessage.Data[1] = (uint8_t)(cal.Left.get() >> 8);
  TxMessage.Data[2] = (uint8_t)(cal.Right.get());
  TxMessage.Data[3] = (uint8_t)(cal.Right.get() >> 8);
  TxMessage.Data[4] = (uint8_t)(cal.Throt.get());
  TxMessage.Data[5] = (uint8_t)(cal.Throt.get() >> 8);
  TxMessage.Data[6] = (uint8_t)(cal.Brake.get());
  TxMessage.Data[7] = (uint8_t)(cal.Brake.get() >> 8);
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);

  TxMessage.StdId   = 0x002;
  TxMessage.Data[0] = (uint8_t)(cal.Decel.get());
  TxMessage.Data[1] = (uint8_t)(cal.Decel.get() >> 8);
  TxMessage.Data[2] = (uint8_t)(cal.clutch << 6|cal.reverse << 4|cal.parking << 2|cal.direction);
  TxMessage.Data[3] = (uint8_t)(cal.start_eng);
  TxMessage.Data[4] = (uint8_t)(cal.Temp.get());//надо привести к таблице температур!!!
  TxMessage.Data[5] = 0;//Скорость трактора!
  TxMessage.Data[6] = 0;//Reserved!
  TxMessage.Data[7] = 0;//Reserved!
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);

  TxMessage.StdId   = 0x003;
  TxMessage.Data[0] = (uint8_t)(cal.OtL.get() / 16);
  TxMessage.Data[1] = (uint8_t)(cal.OtR.get() / 16);
  TxMessage.Data[2] = (uint8_t)(cal.BfL.get() / 16);
  TxMessage.Data[3] = (uint8_t)(cal.BfR.get() / 16);
  TxMessage.Data[4] = (uint8_t)(cal.F.get()   / 16);
  TxMessage.Data[5] = (uint8_t)(cal.R.get()   / 16);
  TxMessage.Data[6] = (uint8_t)(cal.One.get() / 16);
  TxMessage.Data[7] = (uint8_t)(cal.Two.get() / 16);
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);

  TxMessage.StdId   = 0x004;
  TxMessage.Data[0] = (uint8_t)(cal.Three.get() / 16);
  TxMessage.Data[1] = 0;
  TxMessage.Data[2] = 0;
  TxMessage.Data[3] = 0;
  TxMessage.Data[4] = 0;
  TxMessage.Data[5] = 0;
  TxMessage.Data[6] = 0;
  TxMessage.Data[7] = 0;
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::Send(CanTxMsg& TxMessage, std::pair<uint16_t, uint16_t>* data, Calibrate& cal)//Good
{
  TxMessage.Data[0] = (uint8_t)(data[0].first / cal.koef);
  TxMessage.Data[1] = (uint8_t)(data[1].first / cal.koef);
  TxMessage.Data[2] = (uint8_t)(data[2].first / cal.koef);
  TxMessage.Data[3] = (uint8_t)(data[3].first / cal.koef);
  TxMessage.Data[4] = (uint8_t)(data[4].first / cal.koef);
  TxMessage.Data[5] = (uint8_t)(data[5].first / cal.koef);
  TxMessage.Data[6] = (uint8_t)(data[6].first / cal.koef);
  TxMessage.Data[7] = (uint8_t)(data[7].first / cal.koef);
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);

  ++TxMessage.StdId;
  TxMessage.Data[0] = (uint8_t)(data[0].second / cal.koef);
  TxMessage.Data[1] = (uint8_t)(data[1].second / cal.koef);
  TxMessage.Data[2] = (uint8_t)(data[2].second / cal.koef);
  TxMessage.Data[3] = (uint8_t)(data[3].second / cal.koef);
  TxMessage.Data[4] = (uint8_t)(data[4].second / cal.koef);
  TxMessage.Data[5] = (uint8_t)(data[5].second / cal.koef);
  TxMessage.Data[6] = (uint8_t)(data[6].second / cal.koef);
  TxMessage.Data[7] = (uint8_t)(data[7].second / cal.koef);
  while(!cal.CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::SendData(Calibrate& cal)//Good
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  TxMessage.StdId = 0x200;
  Send(TxMessage, cal.d.OtLeftTimeCur, cal);
  TxMessage.StdId = 0x202;
  Send(TxMessage, cal.d.OtRightTimeCur, cal);
  TxMessage.StdId = 0x204;
  Send(TxMessage, cal.d.BfLeftTimeCur, cal);
  TxMessage.StdId = 0x206;
  Send(TxMessage, cal.d.BfRightTimeCur, cal);
  TxMessage.StdId = 0x208;
  Send(TxMessage, cal.d.ForwardTimeCur, cal);
  TxMessage.StdId = 0x20A;
  Send(TxMessage, cal.d.ReverseTimeCur, cal);
  TxMessage.StdId = 0x20C;
  Send(TxMessage, cal.d.OneTimeCur, cal);
  TxMessage.StdId = 0x20E;
  Send(TxMessage, cal.d.TwoTimeCur, cal);
  TxMessage.StdId = 0x210;
  Send(TxMessage, cal.d.ThreeTimeCur, cal);

  TxMessage.StdId = 0x212;
  for(uint8_t i = 0; i < 5; ++i, ++TxMessage.StdId)
  {
    TxMessage.Data[0] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].first);
    TxMessage.Data[1] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].first  >> 8);
    TxMessage.Data[2] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].second);
    TxMessage.Data[3] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].second  >> 8);
    if(++i < 5)
    {
      TxMessage.Data[4] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].first);
      TxMessage.Data[5] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].first >> 8);
      TxMessage.Data[6] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].second);
      TxMessage.Data[7] = (uint8_t)(cal.d.AnalogRemoteCtrl[i].second >> 8);
    }
    else
    {
      TxMessage.Data[4] = 0;
      TxMessage.Data[5] = 0;
      TxMessage.Data[6] = 0;
      TxMessage.Data[7] = 0;
    }
    while(!cal.CanTxMailBox_IsEmpty(CAN2));
    CAN_Transmit(CAN2, &TxMessage);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////       ///////   //////   | \   ||  ////////  //////    //////   //            //////////
//////////       //       //    //  ||\\  ||     //     //   //  //    //  //            //////////
//////////       //       //    //  || \\ ||     //     //////   //    //  //            //////////
//////////       //       //    //  ||  \\||     //     // //    //    //  //            //////////
//////////       ///////   //////   ||   \ |     //     //   //   //////   ///////       //////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void KPP::Parking(const uint16_t rpm, Calibrate& cal)//Проверить на соответствие с логикой пропорционального управл
{
  if(cal.parking == ON && cal.parking_ch)
  {
    ResetAllValve();
    cal.clutch = 0;
  }
  else if(cal.parking == OFF && cal.parking_ch)//Проверить, возможно не соответствует ТТ
  {
    SetBfL();
    SetBfR();
    if(rpm > 350)//Убрать магическое число
    {
      cal.clutch = 1;
      OnClutch(cal);
    }
  }
}
void KPP::ResetAllValve() const//Проверить на логику проп. управ. может надо выключать все дискретн
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
void KPP::ResetAllClutch() const//Проверить на логику проп. управ. может надо выключать все дискрет
{
  ResetFirst();
  ResetSecond();
  ResetThird();
}
void KPP::SetClutch(const uint16_t rpm, Calibrate& cal)//Проверить на логику пропорционального управления
{
  if(cal.parking == OFF && cal.clutch_st == PLUS && cal.clutch < 3 && rpm > 350)//магическое число
  {//за счет времени спада давления в бустере передачи, алгоритм будет соответствовать ТТ, т.е. включается необходимая передача и через 100 мс выключается предыдущая. Так ли надо управлять???
    OffClutch(cal);
    ++cal.clutch;
    OnClutch(cal);
    cal.clutch_st = false;
  }
  else if(cal.parking == OFF && cal.clutch_st == MINUS && cal.clutch > 1 && rpm > 350)//магическое число
  {
    OffClutch(cal);
    --cal.clutch;
    OnClutch(cal);
    cal.clutch_st = false;
  }
}
void KPP::OnClutch(Calibrate& cal) const//Good
{
  switch(cal.clutch)
  {
  case 1: SetFirst();
    break;
  case 2: SetSecond();
    break;
  case 3: SetThird();
    break;
  }
}
void KPP::OffClutch(Calibrate& cal) const//Good
{
  switch(cal.clutch)
  {
  case 1: ResetFirst();
    break;
  case 2: ResetSecond();
    break;
  case 3: ResetThird();
    break;
  }
}
void KPP::SetDirection(const uint8_t dir) const//Проверить нужен ли этот метод
{
  if(dir == F)
    SetForward();
  else if(dir == R)
    SetReverse();
}
void KPP::SwitchDirection(Engine& eng, Calibrate& cal)//Good исправить в соответствии с коментариями
{
  if(cal.direct_ch && cal.parking == OFF)
  {
    if(cal.direction == N)
    {
      ResetForward();
      ResetReverse();
    }
    else if(cal.direction == F)
      ResetReverse();
    else if(cal.direction == R)
      ResetForward();

    if(cal.direction)
    {
      uint16_t rpm = eng.GetRpm();
      if(rpm > 810)//магическое число
      {
        eng.SetRpm(800);//магическое число
        eng.RequestRpm();
      }
      SetDirection(cal.direction);
      eng.SetRpm(rpm);
      eng.RequestRpm();
    }
  }
          
  if(cal.direction == N && cal.parking == ON && eng.GetRpm() < 350)//магическое число
    cal.start_eng = true;
  else
    cal.start_eng = false;
}
void KPP::RightUp(uint8_t begin, uint8_t end,uint8_t right,uint8_t brake) const//Привести в соответ
{
  ResetBfR();//Дискретно, нужна задержка чтобы масло успело слиться!

  if(right > begin && right < end)//приоритет у того кто сильнее тормозит
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
void KPP::RightDown(uint8_t begin, uint8_t brake) const//Привести в соответствие с коментариями
{
  if(brake <= begin)
    ResetOtR();
  else
    SetOtR(brake);
  SetBfR();//должно меняться пропорционально по графику вкл. SetBfR(right)
}
void KPP::LeftUp(uint8_t begin, uint8_t end, uint8_t left, uint8_t brake) const//Привести в соответ
{
  ResetBfL();//Дискретно, нужна задержка чтобы масло успело слиться!

  if(left > begin && left < end)//приоритет у того кто сильнее тормозит
  {
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
void KPP::LeftDown(uint8_t begin, uint8_t brake) const//Привести в соответствие с коментариями
{
  if(brake <= begin)
    ResetOtL();
  else
    SetOtL(brake);
  SetBfL();//должно меняться пропорционально по графику вкл. SetBfL(left)
}
void KPP::BrakeRotate(Calibrate& cal)//Good
{
  static uint8_t old_left  = 0;
  static uint8_t old_right = 0;
  
  bool left_up    = false;
  bool left_down  = false;
  bool right_up   = false;
  bool right_down = false;

  uint8_t left  = 100 - cal.Left.get()  * 100 / 4095;
  uint8_t right = 100 - cal.Right.get() * 100 / 4095;
  uint8_t brake = cal.Brake.get() * 100 / 4095;

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

  if(cal.parking == OFF)
  {
    const uint8_t begin = 10;
    const uint8_t end   = 90;

    if(left <= begin)//приоритет у того, кто сильнее тормозит
    {
      if(brake <= begin)
        ResetOtL();//Дискретно, нужна задержка чтобы масло успело слиться!
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
///////////////////////////////////////////////////////////////////////////////////////////////////
////    ///////      /\      //       //////  //////   //////       /\     ////////  //////    ////
////    //          //\\     //         //    //   //  //   //     //\\       //     //        ////
////    //         //  \\    //         //    //////   //////     //  \\      //     //////    ////
////    //        //////\\   //         //    //   //  // //     //////\\     //     //        ////
////    ///////  //      \\  ///////  //////  //////   //   //  //      \\    //     //////    ////
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Calibrate::CanTxMailBox_IsEmpty(CAN_TypeDef* CANx)//Good
{
  if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0 || (CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1 ||
     (CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    return true;
  else
    return false;
}
void Calibrate::FlashWrite()//Good привести в соответствие с коментариями
{
  const uint32_t* flash_address  = (uint32_t*)address;
  const uint32_t* source_address = (uint32_t*)&d;
  FLASH_Unlock();
  FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3);
  for (uint16_t i = 0; i < sizeof(d); i += 4)//магическое число поменять на sizeof(uint32_t)
    FLASH_ProgramWord((uint32_t)flash_address++, *source_address++);
  FLASH_Lock();
}
void Calibrate::FlashRead()//Good
{
  const uint32_t* flash_address = (uint32_t*)address;
  uint32_t* source_address = (uint32_t*)&d;
  for (uint16_t i = 0; i < sizeof(d); i += 4)//магическое число поменять на sizeof(uint32_t)
    *source_address++ = *(__IO uint32_t*)flash_address++;
}
void Calibrate::RemoteCtrl(uint8_t state)//Good
{
  switch(state)
  {
    case 0x21: d.AnalogRemoteCtrl[0].first  = Throt.get(); break;//min
    case 0x41: d.AnalogRemoteCtrl[0].second = Throt.get(); break;//max
    case 0x22: d.AnalogRemoteCtrl[1].first  = Left.get();  break;//min
    case 0x42: d.AnalogRemoteCtrl[1].second = Left.get();  break;//max
    case 0x23: d.AnalogRemoteCtrl[2].first  = Right.get(); break;//min
    case 0x43: d.AnalogRemoteCtrl[2].second = Right.get(); break;//max
    case 0x24: d.AnalogRemoteCtrl[3].first  = Brake.get(); break;//min
    case 0x44: d.AnalogRemoteCtrl[3].second = Brake.get(); break;//max
    case 0x25: d.AnalogRemoteCtrl[4].first  = Decel.get(); break;//min
    case 0x45: d.AnalogRemoteCtrl[4].second = Decel.get(); break;//max
  }
}
void Calibrate::OtLeftValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.OtLeftValve[count - 1].first  = OtL.get();
    d.OtLeftValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      state = Not;
      TIM_SetCompare1(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare1(TIM4, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::OtRightValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.OtRightValve[count - 1].first  = OtR.get();
    d.OtRightValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare2(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM4, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::BfLeftValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.BfLeftValve[count - 1].first  = BfL.get();
    d.BfLeftValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare3(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare3(TIM4, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::BfRightValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.BfRightValve[count - 1].first  = BfR.get();
    d.BfRightValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare4(TIM4, count = 0);
      return;
    }
  }
  TIM_SetCompare4(TIM4, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::ForwardValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.FValve[count - 1].first  = F.get();
    d.FValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare4(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare4(TIM3, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::ReverseValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.RValve[count - 1].first  = F.get();
    d.RValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare2(TIM1, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM1, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::OneValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.OneValve[count - 1].first  = One.get();
    d.OneValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare1(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare1(TIM3, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::TwoValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.TwoValve[count - 1].first  = Two.get();
    d.TwoValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare2(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare2(TIM3, 2 + count * 2);//магическое число
  ++count;
}
void Calibrate::ThreeValve(State& state)//Заготовка для авто калибровки клапанов, доделать!!!
{
  static uint8_t count = 0;
  if(count != 0 && count <= 250)//магическое число
  {
    d.ThreeValve[count - 1].first  = Three.get();
    d.ThreeValve[count - 1].second = 0;//текущее давление
    if(count == 250)//магическое число
    {
      //state = false;
      TIM_SetCompare3(TIM3, count = 0);
      return;
    }
  }
  TIM_SetCompare3(TIM3, 2 + count * 2);//магическое число
  ++count;
}