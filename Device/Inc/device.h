#ifndef __DEVICE_H
#define __DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

    //#include "platform.h"

#ifdef __cplusplus
}
#endif

#include "dev_can.h"
#include "dev_serial.h"

#include "drivers_buffer.h"
#include "drivers_serial.h"
#include "drivers_can.h"
#include "drivers_remote.h"
#include "drivers_led.h"
#include "drivers_dma.h"

/* extren CAN class */
extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

/* extren Serial class */
extern Serialctrl Serial3_Ctrl;
extern Serialctrl Serial6_Ctrl;
extern Serialctrl Serial7_Ctrl;
extern Serialctrl Serial8_Ctrl;

/* extren DMA class */
extern DMAdev usart3_TxDMA;
extern DMAdev usart3_RxDMA;
extern DMAdev usart6_TxDMA;
extern DMAdev usart6_RxDMA;
extern DMAdev usart7_TxDMA;
extern DMAdev usart7_RxDMA;
extern DMAdev usart8_TxDMA;
extern DMAdev usart8_RxDMA;


#endif /* __DEVICE_H */

