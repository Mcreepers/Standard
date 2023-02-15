#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#include "dev_serial.h"

class Serial_Ctrl : public Serialctrl
{
public:
    Serial_Ctrl(
        USART_TypeDef * _USARTx, uint32_t BufferSize, uint8_t header, uint8_t tail, uint8_t lenth);
    Serial_Ctrl(
        USART_TypeDef * _USARTx, uint32_t BufferSize, uint32_t USART_ITPending, uint8_t header,
        uint8_t tail, uint8_t lenth);

    void Hook(bool mode);
    uint8_t Get_Data(void * buf);
    void Send_to_Message();
    void IRQHandler();

    ~Serial_Ctrl() {
    }

private:
    uint8_t buffer_size;
    uint8_t Header;
    uint8_t Tail;
    uint8_t Len;
    uint8_t Lenth;
    uint8_t Temp;
    uint8_t * Data;
    void * buf;
};

void Serial_ALL_Init(void);

class Serial_Com
{
public:
    Serial_Com() {
        this->Head = 0xff;
        this->Addr = 0x01;
    }

    void SendData();

    void ReceiveData(void * buf, uint8_t len);
    void Check_Calc(uint8_t data);

    void SendHeartBeat();
    void SendChassisVelocity();

    ~Serial_Com() {
    }

private:
    uint8_t Head;
    uint8_t Addr;
    uint8_t ID;
    uint8_t len;
    const void * buf;
    uint8_t SumCheck;
    uint8_t AddCheck;

    struct Heart_Beat_t {
        bool heartbeat;
    };

    struct Chassis_Velocity_t {
        int8_t chassis_vx;
        int8_t chassis_vy;
        int32_t chassis_angle;  //·Å´ó100±¶
    };

    struct odom_position_t {
        int32_t odom_position_x;
        int32_t odom_position_y;
        int32_t odom_position_z;
        int32_t odom_orientation_x;
    };

    struct chassis_target_liner_t {
        int32_t chassis_target_linear_x;
        int32_t chassis_target_linear_y;
        int32_t chassis_target_linear_z;
    };

    struct gimbal_ctrl_t {
        int8_t gimbal_ctrl_mode;
        int32_t gimbal_yaw;
        int32_t gimbal_pitch;
        int32_t gimbal_roll;
    };

    Heart_Beat_t HeartBeat[2];
    Chassis_Velocity_t ChassisVelocity[2];
};

#endif
