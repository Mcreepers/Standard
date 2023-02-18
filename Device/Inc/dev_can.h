#ifndef __DEVICE_CAN_H
#define __DEVICE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#include "drivers_can.h"

class CANctrl: public CANdev
{
public:
	CANctrl(CAN_TypeDef *CANx): CANdev(CANx) {}

	void attachInterrupt(CAN_CallbackFunction_t Function);

	void ChangeID(uint16_t StdID);

	void SendData(void *buf, uint8_t len);

	void IRQHandler(void);

	CanRxMsg Rx_Message;
private:
	uint32_t StdId;
};

extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

#endif /* __DEVICE_CAN_H */
