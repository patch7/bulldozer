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
void KPP::RequestRpm(const uint16_t x) const
{
  CanTxMsg TxMessage;
  TxMessage.ExtId = 0x0C000000;
  TxMessage.RTR   = CAN_RTR_DATA;
  TxMessage.IDE   = CAN_ID_EXT;
  TxMessage.DLC   = 8;

  TxMessage.Data[0] = 0;
  TxMessage.Data[1] = 0;
  TxMessage.Data[2] = 0;
  TxMessage.Data[3] = x * resol % 256;
  TxMessage.Data[4] = x * resol / 256;
  TxMessage.Data[5] = 0;
  TxMessage.Data[6] = 0;
  TxMessage.Data[7] = 0;
  while(!CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::Send(Calibrate& cal)//Good, надо исправить в соответствии с коментариями.
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  for(TxMessage.StdId = 0x001; TxMessage.StdId < 0x005; ++TxMessage.StdId)
  {
    switch(TxMessage.StdId)
    {
      case 0x001:
        TxMessage.Data[0] = (uint8_t)(cal.Left.get());
        TxMessage.Data[1] = (uint8_t)(cal.Left.get() >> 8);
        TxMessage.Data[2] = (uint8_t)(cal.Right.get());
        TxMessage.Data[3] = (uint8_t)(cal.Right.get() >> 8);
        TxMessage.Data[4] = (uint8_t)(cal.Throt.get());
        TxMessage.Data[5] = (uint8_t)(cal.Throt.get() >> 8);
        TxMessage.Data[6] = (uint8_t)(cal.Brake.get());
        TxMessage.Data[7] = (uint8_t)(cal.Brake.get() >> 8);
        break;
      case 0x002:
        TxMessage.Data[0] = (uint8_t)(cal.Decel.get());
        TxMessage.Data[1] = (uint8_t)(cal.Decel.get() >> 8);
        TxMessage.Data[2] = (uint8_t)(cal.clutch<<6|cal.reverse <<4|cal.parking <<2|cal.direction);
        TxMessage.Data[3] = (uint8_t)(cal.start_eng);
        TxMessage.Data[4] = (uint8_t)(cal.Temp.get());//надо привести к таблице температур!!!
        TxMessage.Data[5] = 0;//Скорость трактора!
        TxMessage.Data[6] = 0;//Reserved!
        TxMessage.Data[7] = 0;//Reserved!
        break;
      case 0x003:
        TxMessage.Data[0] = (uint8_t)(cal.OtL.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[1] = (uint8_t)(cal.OtR.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[2] = (uint8_t)(cal.BfL.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[3] = (uint8_t)(cal.BfR.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[4] = (uint8_t)(cal.F.get()   / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[5] = (uint8_t)(cal.R.get()   / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[6] = (uint8_t)(cal.One.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[7] = (uint8_t)(cal.Two.get() / 16.25);//На дисплее надо * на (4.88..5)
        break;
      case 0x004:
        TxMessage.Data[0] = (uint8_t)(cal.Three.get() / 16.25);//На дисплее надо * на (4.88..5)
        TxMessage.Data[1] = 0;
        TxMessage.Data[2] = 0;
        TxMessage.Data[3] = 0;
        TxMessage.Data[4] = 0;
        TxMessage.Data[5] = 0;
        TxMessage.Data[6] = 0;
        TxMessage.Data[7] = 0;
        break;
    }
    while(!CanTxMailBox_IsEmpty(CAN2));
    CAN_Transmit(CAN2, &TxMessage);
  }
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
  while(!CanTxMailBox_IsEmpty(CAN2));
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
  while(!CanTxMailBox_IsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void KPP::SendData(Calibrate& cal)//Good
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 8;

  TxMessage.StdId = 0x200;
  Send(TxMessage, cal.d.OtLeftTimePres, cal);
  TxMessage.StdId = 0x202;
  Send(TxMessage, cal.d.OtRightTimePres, cal);
  TxMessage.StdId = 0x204;
  Send(TxMessage, cal.d.BfLeftTimePres, cal);
  TxMessage.StdId = 0x206;
  Send(TxMessage, cal.d.BfRightTimePres, cal);
  TxMessage.StdId = 0x208;
  Send(TxMessage, cal.d.ForwardTimePres, cal);
  TxMessage.StdId = 0x20A;
  Send(TxMessage, cal.d.ReverseTimePres, cal);
  TxMessage.StdId = 0x20C;
  Send(TxMessage, cal.d.OneTimePres, cal);
  TxMessage.StdId = 0x20E;
  Send(TxMessage, cal.d.TwoTimePres, cal);
  TxMessage.StdId = 0x210;
  Send(TxMessage, cal.d.ThreeTimePres, cal);

  TxMessage.StdId = 0x212;
  for(uint8_t i = 0; TxMessage.StdId < 0x215; ++TxMessage.StdId, ++i)
  {
    TxMessage.Data[0] = cal.d.AnalogRemoteCtrlAndRPM[i + i].first ;
    TxMessage.Data[1] = cal.d.AnalogRemoteCtrlAndRPM[i + i].first  >> 8;
    TxMessage.Data[2] = cal.d.AnalogRemoteCtrlAndRPM[i + i].second;
    TxMessage.Data[3] = cal.d.AnalogRemoteCtrlAndRPM[i + i].second >> 8;
    TxMessage.Data[4] = cal.d.AnalogRemoteCtrlAndRPM[i + i + 1].first ;
    TxMessage.Data[5] = cal.d.AnalogRemoteCtrlAndRPM[i + i + 1].first  >> 8;
    TxMessage.Data[6] = cal.d.AnalogRemoteCtrlAndRPM[i + i + 1].second;
    TxMessage.Data[7] = cal.d.AnalogRemoteCtrlAndRPM[i + i + 1].second >> 8;
    while(!CanTxMailBox_IsEmpty(CAN2));
    CAN_Transmit(CAN2, &TxMessage);
  }
}
//тест
void KPP::SendDataValve(Calibrate& cal)//Пока только ОТ левый
{
  CanTxMsg TxMessage;
  TxMessage.RTR   = CAN_RTR_DATA;
  TxMessage.IDE   = CAN_ID_STD;
  TxMessage.DLC   = 8;
  TxMessage.StdId = 0x300;

  for(uint8_t i = 1; i <= cal.d.Valve.begin()->size(); ++i)
    if(i % 9)
      TxMessage.Data[(i % 9) - 1] = (uint8_t)(cal.d.Valve[0][i - 1]);
    else
    {
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);
      ++TxMessage.StdId;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////       ///////   //////   | \   ||  ////////  //////    //////   //            //////////
//////////       //       //    //  ||\\  ||     //     //   //  //    //  //            //////////
//////////       //       //    //  || \\ ||     //     //////   //    //  //            //////////
//////////       //       //    //  ||  \\||     //     // //    //    //  //            //////////
//////////       ///////   //////   ||   \ |     //     //   //   //////   ///////       //////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void KPP::Parking(Calibrate& cal)//Проверить на пропорциональное управление
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
    ResetOtL();//ОТ выключен
    ResetOtR();//ОТ выключен
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
void KPP::SetClutch(Calibrate& cal)//Проверить на пропорциональное управление
{
  if(cal.parking == OFF && cal.clutch_st == PLUS && cal.clutch < 3 && rpm > 350)//магическое число
  {//за счет времени спада давления в бустере передачи, алгоритм будет соответствовать ТТ, т.е. включается необходимая передача и через 100 мс выключается предыдущая. Так ли надо управлять???
    OffClutch(cal);
    ++cal.clutch;
    OnClutch(cal);
    cal.clutch_st = false;
  }
  else if(cal.parking == OFF && cal.clutch_st == MINUS && cal.clutch > 1 && rpm > 350)//magic numb
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
void KPP::SwitchDirection(Calibrate& cal)//Good привести в соответствии с коментариями
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
      uint16_t temp = rpm;
      if(rpm > 810)//магическое число
        RequestRpm(800);//магическое число
      SetDirection(cal.direction);
      RequestRpm(temp);
    }
  }
          
  if(cal.direction == N && cal.parking == ON && rpm < 350)//магическое число
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

  uint8_t left  = (cal.d.AnalogRemoteCtrlAndRPM[1].second * 100 /
                   cal.d.AnalogRemoteCtrlAndRPM[1].second) - (cal.Left.get() * 100 /
                   cal.d.AnalogRemoteCtrlAndRPM[1].second);
  uint8_t right = (cal.d.AnalogRemoteCtrlAndRPM[2].second * 100 /
                   cal.d.AnalogRemoteCtrlAndRPM[2].second) - (cal.Right.get() * 100 /
                   cal.d.AnalogRemoteCtrlAndRPM[2].second);
  uint8_t brake =  cal.Brake.get() * 100 / cal.d.AnalogRemoteCtrlAndRPM[3].second;

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
    const uint8_t begin =  5;
    const uint8_t end   = 95;

    if(left <= begin)//приоритет у того, кто сильнее тормозит
    {
      if(brake <= begin)
        ResetOtL();//Дискретно, по графику или нет?
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
      LeftUp(begin, end, left, brake);
    else if(left_down)
      LeftDown(begin, brake);
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
void Calibrate::RemoteCtrlAndRPM(uint8_t state, uint16_t data)//Good
{
  switch(state)
  {
    case 0x21: d.AnalogRemoteCtrlAndRPM[0].first  = Throt.get(); break;//min
    case 0x22: d.AnalogRemoteCtrlAndRPM[1].first  = Left.get();  break;//min
    case 0x23: d.AnalogRemoteCtrlAndRPM[2].first  = Right.get(); break;//min
    case 0x24: d.AnalogRemoteCtrlAndRPM[3].first  = Brake.get(); break;//min
    case 0x25: d.AnalogRemoteCtrlAndRPM[4].first  = Decel.get(); break;//min
    case 0x26: d.AnalogRemoteCtrlAndRPM[5].first  = data;        break;//min

    case 0x41: d.AnalogRemoteCtrlAndRPM[0].second = Throt.get(); break;//max
    case 0x42: d.AnalogRemoteCtrlAndRPM[1].second = Left.get();  break;//max
    case 0x43: d.AnalogRemoteCtrlAndRPM[2].second = Right.get(); break;//max
    case 0x44: d.AnalogRemoteCtrlAndRPM[3].second = Brake.get(); break;//max
    case 0x45: d.AnalogRemoteCtrlAndRPM[4].second = Decel.get(); break;//max
    case 0x46: d.AnalogRemoteCtrlAndRPM[5].second = data;        break;//max
  }
}
//Функция вызывается раз в 10 мс, каждый 22-й (220 мс) вызов сохраняет давление и увеличивает ток клапана. После каждого увеличения тока на клапане, происходит ожидание задержки реакции электромагнита (100 мс, запас в 25 мс) и реакции клапана (стабилизации давления 50 мс, запас 25 мс). Далее в течении 70 мс (т.е. 7 раз) записывается текущее значение давления в фильтр. После заполнения фильтра записываем давление в таблицу соответствия тока давлению и увеличиваем ток.
void Calibrate::Valve(State& state, Pressure pres)
{
  const uint8_t cycle_time   = 22;// 22 * 10 = 220 мс
  const uint8_t current_step = 4; // 500 / 4 = 125 точек
  static uint16_t count      = 0;

  auto pValve = d.Valve.begin() + static_cast<uint8_t>(state) - 1;

  if(!count)
  {
    CanTxMsg TxMessage;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.DLC     = 1;
    TxMessage.StdId   = 0x114;
    TxMessage.Data[0] = 0x01;
    while(!CanTxMailBox_IsEmpty(CAN2));
    CAN_Transmit(CAN2, &TxMessage);//статус калибровки, идет калибровка клапана.
  }

  if(!(count % cycle_time) && count / cycle_time <= pValve->size())
  {
    if(count > 0)
      (*pValve)[count / cycle_time - 1] = PresFilter.get();//записываем давление
    if(count / cycle_time == pValve->size())//если последняя точка
    {
      state = Not;
      TIM_SetCompare1(TIM4, count = 0);

      CanTxMsg TxMessage;
      TxMessage.RTR     = CAN_RTR_DATA;
      TxMessage.IDE     = CAN_ID_STD;
      TxMessage.DLC     = 1;
      TxMessage.StdId   = 0x114;
      TxMessage.Data[0] = 0x00;
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);//статус калибровки, калибровка клапана окончена.
      return;
    }
    TIM_SetCompare1(TIM4, current_step + current_step * (count / cycle_time));
  }
  if(count >= 15 + cycle_time * (count / cycle_time))//магическое число
    PresFilter.push(static_cast<uint16_t>(pres.f * 10));
  ++count;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//////////         ////////   //         //////    //////       /\       //              //////////
//////////         //         //        //    //   //   //     //\\      //              //////////
//////////         //   ///   //        //    //   //////     //  \\     //              //////////
//////////         //    //   //        //    //   //   //   //////\\    //              //////////
//////////         ////////   ///////    //////    //////   //      \\   ///////         //////////
///////////////////////////////////////////////////////////////////////////////////////////////////
bool CanTxMailBox_IsEmpty(CAN_TypeDef* CANx)//Good
{
  if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0 ||
     (CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1 ||
     (CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    return true;
  else
    return false;
}