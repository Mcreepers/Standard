#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#include "dev_system.h"

#define SERIAL3 USART3
#define SERIAL6 USART6
#define SERIAL7 UART7
#define SERIAL8 UART8

struct Serial_Data_t
{
    uint8_t Header;
    uint8_t Tail;
    uint8_t Lenth;
    uint8_t buffer_size;
    uint8_t Len;
    uint8_t Temp;
    uint8_t *Data;
    Serial_Data_t(uint8_t Header_, uint8_t Tail_, uint8_t Lenth_, uint8_t buffer_size_)
        :Header(Header_), Tail(Tail_), Lenth(Lenth_), buffer_size(buffer_size_)
    {
        Data = new uint8_t[buffer_size_];
    };
};

class Serial_Ctrl
{
public:
    Serial_Ctrl()
    {
        Serial3 = new Serial_Data_t(Serial3_Data_Header, Serial3_Data_Tail, Serial3_Data_Lenth, Serial3_Buffer_Size);
        Serial6 = new Serial_Data_t(Serial6_Data_Header, Serial6_Data_Tail, Serial6_Data_Lenth, Serial6_Buffer_Size);
        Serial7 = new Serial_Data_t(Serial7_Data_Header, Serial7_Data_Tail, Serial7_Data_Lenth, Serial7_Buffer_Size);
        Serial8 = new Serial_Data_t(Serial8_Data_Header, Serial8_Data_Tail, Serial8_Data_Lenth, Serial8_Buffer_Size);
    }

    void Hook(USART_TypeDef *SERIAL, bool mode);
    void Handle(Serialctrl *Serial, Serial_Data_t *Usart, bool mode);
    void Send_to_Message(Serialctrl *SerialCtrl);

    Serialctrl *Tran(USART_TypeDef *SERIAL);

    uint8_t Get_Data(Serial_Data_t *Serial, uint8_t *buf);

    void SendData(USART_TypeDef *SERIAL, uint8_t ch);
    void SendData(USART_TypeDef *SERIAL, const void *str);
    void SendData(USART_TypeDef *SERIAL, const void *buf, uint8_t len);
    void SendData(Serialctrl *Serial, uint8_t ch);
    void SendData(Serialctrl *Serial, const void *str);
    void SendData(Serialctrl *Serial, const void *buf, uint8_t len);

    ~Serial_Ctrl() {}

private:
    Serial_Data_t *Serial3;
    Serial_Data_t *Serial6;
    Serial_Data_t *Serial7;
    Serial_Data_t *Serial8;

    void *buf;
};

void Serial_ALL_Init(void);

//木鸢通讯协议
//https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%A8%E9%B8%A2%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.html
class Serial_Comm
{
public:
    Serial_Comm(Serialctrl *SerialCtrl_):SerialCtrl(SerialCtrl_)
    {
        this->Head = 0xff;
        this->Addr = 0x01;
    }

    void SendData();

    void ReceiveData(void *buf, uint8_t len);
    void Check_Calc(uint8_t data);

    void SendHeartBeat();
    void SendChassisVelocity();

    ~Serial_Comm()
    {}

private:
    Serialctrl *SerialCtrl;
    uint8_t Head;
    uint8_t Addr;
    uint8_t ID;
    uint8_t len;
    const void *buf;
    uint8_t SumCheck;
    uint8_t AddCheck;

    struct Heart_Beat_t
    {
        bool heartbeat;
    };

    struct Chassis_Velocity_t
    {
        int8_t chassis_vx;
        int8_t chassis_vy;
        int32_t chassis_angle;  //放大100倍
    };

    struct odom_position_t
    {
        int32_t odom_position_x;
        int32_t odom_position_y;
        int32_t odom_position_z;
        int32_t odom_orientation_x;
    };

    struct chassis_target_liner_t
    {
        int32_t chassis_target_linear_x;
        int32_t chassis_target_linear_y;
        int32_t chassis_target_linear_z;
    };

    struct gimbal_ctrl_t
    {
        int8_t gimbal_ctrl_mode;
        int32_t gimbal_yaw;
        int32_t gimbal_pitch;
        int32_t gimbal_roll;
    };

    Heart_Beat_t HeartBeat[2];
    Chassis_Velocity_t ChassisVelocity[2];
};

#endif
