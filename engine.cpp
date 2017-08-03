#include "engine.h"

void Engine::RequestRpm(const uint16_t x) const
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

  CAN_Transmit(CAN2, &TxMessage);
}
