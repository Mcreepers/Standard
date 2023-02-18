#include "dev_can.h"
#include "app_motor.h"
#include "queue.h"
#include "Message_Task.h"

CANctrl CAN1_Ctrl(CAN1);
CANctrl CAN2_Ctrl(CAN2);

void CANctrl::attachInterrupt(CAN_CallbackFunction_t Function)
{
    this->CAN_Function = Function;
}

void CANctrl::ChangeID(uint16_t StdID)
{
    this->StdId = StdID;
}

void CANctrl::SendData(void *buf, uint8_t len)
{
    uint8_t *ch = (uint8_t *)buf;
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = len;
    do
    {
        TxMessage.Data[len] = ch[len];
    } while(len--);

    CAN_Transmit(CANx, &TxMessage);
}

void CANctrl::IRQHandler(void)
{
    if(CAN_GetITStatus(CANx, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CANx, CAN_IT_FMP0);
        CAN_Receive(CANx, CAN_FIFO0, &Rx_Message);
        if(CAN_Function)
        {
            CAN_Function(&Rx_Message);
        }
    }
}

extern"C"
{
    void CAN1_RX0_IRQHandler(void)
    {
        CAN1_Ctrl.IRQHandler();
    }
    void CAN2_RX0_IRQHandler(void)
    {
        CAN2_Ctrl.IRQHandler();
    }
}
