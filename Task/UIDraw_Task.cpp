#include "UIDraw_Task.h"

#include "arm_math.h"
#include "protocol_ui.h"
#include "protocol_judgement.h"
#include "Chassis_Task.h"
#include "Message_Task.h"
//姿态角 单位度
fp32 angle_degree[3] = { 0.0f, 0.0f, 0.0f };
float sin_yaw_UI, cos_yaw_UI;
float viosion_x = 0, viosion_y = 0;

char null[] = { "   " };
char cap[] = { "   %   " };
char Ture[] = { "ON " };
char Flase[] = { "OFF" };
char Level[] = { "speed_gear  " };
char Speed_gears[4][2] = { "0","1","2","3" };
char Chassis_Mode[4][7] = { "STOP","FOLLOW","NORMAL","SPIN" };
char uisend = 10;
char Buff[3][2] = { "1","2","3" };
uint32_t x1, x2, x3, x4, y1, y2, y3, y4;
uint16_t standard_ID1, standard_ID2;

const Chassis_Ctrl_Flags_t *chassis_flag;
const chassis_mode_e *chassis_mode;
void UIDraw_Task(void *pvParameters)
{
    usart7_DMA_init();
    chassis_flag = get_chassis_flag_control_point();
    chassis_mode = get_chassis_mode_control_point();
    while (1)
    {
        standard_ID1 = game_robot_state_t.robot_id;
        standard_ID2 = 0x100 + uint32_t(game_robot_state_t.robot_id);
        
        while (uisend > 1)
        {
            //底盘状态
            Char_Painter("friesp", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 790, Type_Flag_Frie_Speed);
            Char_Painter("spin", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 590, Type_Flag_Spin);
            Char_Painter("level", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 25, 340, 780, Type_Flag_Level);
            Char_Painter("power", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 25, 340, 720, Type_Flag_Speed_up);

            //视觉状态
            Char_Painter("aim", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 750, Type_Flag_Auto_Aiming);
            Char_Painter("energy", Flase, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 670, Type_Flag_Energy);
            Char_Painter("predict", Flase, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 710, Type_Flag_Predict);

            //发射状态
            Char_Painter("shoot", null, UI_Graph_ADD, standard_ID1, standard_ID2, 2, Graphic_Color_White, 3, 20, 1500, 550, Type_Flag_Shoot_Mode);

            //电容状态
            Num_Painter("vo", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_White, 3, 1600, 410, 20, NULL, 0, NULL);
            Num_Painter("li", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_White, 3, 1600, 370, 20, NULL, 0, NULL);
            Graph_Painter(" ", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 500, 100, 500, 100, NULL, NULL, NULL);

            //敌我场地血量
            Num_Painter("outpost_red", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_White, 3, 10, 770, 20, NULL, game_robot_HP_t.red_outpost_HP, NULL);
            Num_Painter("outpost_blue", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_White, 3, 100, 770, 20, NULL, game_robot_HP_t.blue_outpost_HP, NULL);
            Num_Painter("sentry_red", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_White, 3, 10, 730, 20, NULL, game_robot_HP_t.red_7_robot_HP, NULL);
            Num_Painter("sentry_blue", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_White, 3, 100, 730, 20, NULL, game_robot_HP_t.blue_7_robot_HP, NULL);
            Num_Painter("base_red", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_White, 3, 10, 690, 20, NULL, game_robot_HP_t.red_base_HP, NULL);
            Num_Painter("base_blue", UI_Graph_ADD, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_White, 3, 100, 690, 20, NULL, game_robot_HP_t.blue_base_HP, NULL);

            //底盘相对角度
            Graph_Painter("Li1", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);
            Graph_Painter("Li2", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);
            Graph_Painter("Li3", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);
            Graph_Painter("Li4", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);
            Graph_Painter("Li5", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);

            //近战坐标线，弹道偏移
            Graph_Painter("LI6", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 940, 540, 940, 240, NULL, NULL, NULL);//960,540-240
            Graph_Painter("LI7", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 880, 455, 1000, 455, NULL, NULL, NULL);//930,990
            Graph_Painter("Li8", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 890, 415, 990, 415, NULL, NULL, NULL);//920,1000
            Graph_Painter("Li9", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 900, 395, 980, 395, NULL, NULL, NULL);//910,1010
            Graph_Painter("LI1", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 910, 375, 970, 375, NULL, NULL, NULL);//900,1020
            Graph_Painter("LI2", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 3, 927, 354, 953, 354, NULL, NULL, NULL);//900,1020

            uisend--;
        }

        Char_Painter("friesp", Chassis_Mode[*chassis_mode], UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 20, 1500, 790, Type_Flag_Frie_Speed);
        delay_ms(10);

        if (chassis_flag->Vision_Flag == 1)
            Char_Painter("aim", Ture, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 20, 1500, 750, Type_Flag_Auto_Aiming);
        else if (chassis_flag->Vision_Flag == 0)
            Char_Painter("aim", Flase, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Orange, 3, 20, 1500, 750, Type_Flag_Auto_Aiming);
        delay_ms(10);

        if (chassis_flag->Fric_Flag == 1)
            Char_Painter("shoot", Ture, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 20, 1500, 550, Type_Flag_Shoot_Mode);
        else if (chassis_flag->Fric_Flag == 0)
            Char_Painter("shoot", Flase, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Orange, 3, 20, 1500, 550, Type_Flag_Shoot_Mode);
        delay_ms(10);

        if (Gimbal.goal == 1)
            Graph_Painter("cir", UI_Graph_Change, UI_Graph_Circle, standard_ID1, standard_ID2, 3, Graphic_Color_Green, 10, 960, 540, NULL, NULL, 50, NULL, NULL);
        else if (Gimbal.goal == 0)
            Graph_Painter("cir", UI_Graph_Change, UI_Graph_Circle, standard_ID1, standard_ID2, 3, Graphic_Color_White, 10, 960, 540, NULL, NULL, 50, NULL, NULL);

        if (chassis_flag->Predict_Flag == 1)
            Char_Painter("predict", Ture, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 20, 1500, 710, Type_Flag_Predict);
        else if (chassis_flag->Predict_Flag == 0)
            Char_Painter("predict", Flase, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Orange, 3, 20, 1500, 710, Type_Flag_Predict);
        delay_ms(10);

        if (chassis_flag->Energy_Flag == 1)
            Char_Painter("energy", Ture, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 20, 1500, 670, Type_Flag_Energy);
        else if (chassis_flag->Energy_Flag == 0)
            Char_Painter("energy", Flase, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Orange, 3, 20, 1500, 670, Type_Flag_Energy);
        delay_ms(10);

        if (Chassis.Velocity.Speed_Gear == 0)
            Char_Painter("level", Speed_gears[Chassis.Velocity.Speed_Gear], UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 25, 340, 780, Type_Flag_Level);
        else if (Chassis.Velocity.Speed_Gear == 1)
            Char_Painter("level", Speed_gears[Chassis.Velocity.Speed_Gear], UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Cyan, 3, 25, 340, 780, Type_Flag_Level);
        else if (Chassis.Velocity.Speed_Gear == 2)
            Char_Painter("level", Speed_gears[Chassis.Velocity.Speed_Gear], UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Orange, 3, 25, 340, 780, Type_Flag_Level);
        else if (Chassis.Velocity.Speed_Gear == 3)
            Char_Painter("level", Speed_gears[Chassis.Velocity.Speed_Gear], UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Pink, 3, 25, 340, 780, Type_Flag_Level);

        if (chassis_flag->Speed_Up_Flag == 1)
            Char_Painter("power", null, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Pink, 3, 25, 340, 720, Type_Flag_Speed_up);
        else if (chassis_flag->Speed_Up_Flag == 0) Char_Painter("power", null, UI_Graph_Change, standard_ID1, standard_ID2, 2, Graphic_Color_Green, 3, 25, 340, 720, Type_Flag_Speed_up);

        Num_Painter("outpost_red", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_Purplish_red, 3, 10, 840, 20, NULL, game_robot_HP_t.red_outpost_HP, NULL);
        delay_ms(10);
        Num_Painter("outpost_blue", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_Cyan, 3, 100, 840, 20, NULL, game_robot_HP_t.blue_outpost_HP, NULL);
        delay_ms(10);
        Num_Painter("sentry_red", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_Purplish_red, 3, 10, 810, 20, NULL, game_robot_HP_t.red_7_robot_HP, NULL);
        delay_ms(10);
        Num_Painter("sentry_blue", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_Cyan, 3, 100, 810, 20, NULL, game_robot_HP_t.blue_7_robot_HP, NULL);
        delay_ms(10);
        Num_Painter("base_red", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 3, Graphic_Color_Purplish_red, 3, 10, 780, 20, NULL, game_robot_HP_t.red_base_HP, NULL);
        delay_ms(10);
        Num_Painter("base_blue", UI_Graph_Change, UI_Graph_Int, standard_ID1, standard_ID2, 4, Graphic_Color_Cyan, 3, 100, 780, 20, NULL, game_robot_HP_t.blue_base_HP, NULL);
        delay_ms(10);

        //超级电容
        // if(USART8_RX_BUF[1]<12)
        // {
        // 	Num_Painter ("vo",UI_Graph_Change,UI_Graph_Int,standard_ID1,standard_ID2,3,Graphic_Color_Orange,3,1300,100,20,NULL,USART8_RX_BUF[1],NULL);
        //     Graph_Painter(" ",UI_Graph_Change,UI_Graph_Line,standard_ID1,standard_ID2,3,Graphic_Color_Orange,10,500,100,500+USART8_RX_BUF[1]*44,100,NULL,NULL,NULL);
        // }else if(USART8_RX_BUF[1]>15)
        // {
        // 	Num_Painter ("vo",UI_Graph_Change,UI_Graph_Int,standard_ID1,standard_ID2,3,Graphic_Color_Green,3,1300,100,20,NULL,USART8_RX_BUF[1],NULL);				
        //     Graph_Painter(" ",UI_Graph_Change,UI_Graph_Line,standard_ID1,standard_ID2,3,Graphic_Color_Green,10,500,100,500+USART8_RX_BUF[1]*44,100,NULL,NULL,NULL);
        // }else
        // {
        // 	Num_Painter ("vo",UI_Graph_Change,UI_Graph_Int,standard_ID1,standard_ID2,3,Graphic_Color_Yellow,3,1300,100,20,NULL,USART8_RX_BUF[1],NULL);				
        //     Graph_Painter(" ",UI_Graph_Change,UI_Graph_Line,standard_ID1,standard_ID2,3,Graphic_Color_Yellow,10,500,100,500+USART8_RX_BUF[1]*44,100,NULL,NULL,NULL);				
        // }
        // 	Num_Painter ("li",UI_Graph_Change,UI_Graph_Int,standard_ID1,standard_ID2,3,Graphic_Color_Yellow,3,1350,100,20,NULL,USART8_RX_BUF[2],NULL);
        // delay_ms(10);

        x1 = 120 - arm_sin_f32(-*Chassis.chassis_yaw_relative_angle + 0.52359877f) * 100.0f;
        y1 = 625 + arm_cos_f32(-*Chassis.chassis_yaw_relative_angle + 0.52359877f) * 100.0f;
        x2 = 120 - arm_cos_f32(-*Chassis.chassis_yaw_relative_angle + 1.04719754f) * 100.0f;
        y2 = 625 - arm_sin_f32(-*Chassis.chassis_yaw_relative_angle + 1.04719754f) * 100.0f;
        x3 = 120 + arm_sin_f32(-*Chassis.chassis_yaw_relative_angle + 0.52359877f) * 100.0f;
        y3 = 625 - arm_cos_f32(-*Chassis.chassis_yaw_relative_angle + 0.52359877f) * 100.0f;
        x4 = 120 + arm_cos_f32(-*Chassis.chassis_yaw_relative_angle + 1.04719754f) * 100.0f;
        y4 = 625 + arm_sin_f32(-*Chassis.chassis_yaw_relative_angle + 1.04719754f) * 100.0f;

        Graph_Painter("Li1", UI_Graph_Change, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 13, x1, y1, x2, y2, NULL, NULL, NULL);
        Graph_Painter("Li2", UI_Graph_Change, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 13, x2, y2, x3, y3, NULL, NULL, NULL);
        Graph_Painter("Li3", UI_Graph_Change, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Yellow, 13, x3, y3, x4, y4, NULL, NULL, NULL);
        Graph_Painter("Li4", UI_Graph_Change, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Main, 13, x4, y4, x1, y1, NULL, NULL, NULL);
        Graph_Painter("Li5", UI_Graph_Change, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_Main, 17, 120, 625, 120, 750, NULL, NULL, NULL);

        Graph_Painter("L", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 2, 860, 940, 1060, 940, NULL, NULL, NULL);
        Graph_Painter("L1", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 2, 960, 640, 960, 440, NULL, NULL, NULL);
        Graph_Painter("L2", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 2, 960, 413, 1000, 413, NULL, NULL, NULL);
        Graph_Painter("L3", UI_Graph_ADD, UI_Graph_Line, standard_ID1, standard_ID2, 3, Graphic_Color_White, 2, 960, 405, 1000, 405, NULL, NULL, NULL);
        Graph_Painter("15", UI_Graph_ADD, UI_Graph_Rectangle, standard_ID1, standard_ID2, 3, Graphic_Color_White, 2, 910, 580, 1010, 500, NULL, NULL, NULL);
        vTaskDelay(10);
        // #if INCLUDE_uxTaskGetStackHighWaterMark
        //         UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
        // #endif
    }
}
