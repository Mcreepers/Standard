#include "protocol_ui.h"
#include "string.h"
#include "stdio.h"

ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
ext_client_custom_graphic_delete_t  graphic_delete;
ext_client_custom_character_t  ext_client_custom_characte;
draw_data_struct_t		draw_data_struct;
char_data_struct_t    char_data_struct;
graph_data_struct_t   graph_data_struct;
Num_data_struct_t   	num_data_struct;
map_data_struct_t     map_data_struct;

uint8_t tx7_buf[255u];

char* Char_Splicing(char * dest,char * src)
{	
		char *ret = dest;         
	
	while(* dest!='\0')      
	{                        
		dest++;             
	}
	* dest = *src;           
	while(* src != '\0')     
	{
		* dest++ = *src++;    
	
	}
		*src++ = '\0';       
	return ret;	

}

void Graph_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye, uint16_t Sender_ID, uint16_t Receiver_ID, 
									 uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y,
									 uint32_t Radius, uint32_t start_angle, uint32_t end_angle)
{
	uint8_t src,UI_Seq;
	
		switch(Graphic_tpye)
	{
		case UI_Graph_Line:
		{
			graph_data_struct.graphic_data.start_x = start_x;				 
			graph_data_struct.graphic_data.start_y = start_y;				
			graph_data_struct.graphic_data.end_x = end_x; 						 
			graph_data_struct.graphic_data.end_y = end_y; 
			UI_Seq = 0x08;
			break;
		}			
		case UI_Graph_Rectangle:
		{
			graph_data_struct.graphic_data.start_x = start_x;				 
			graph_data_struct.graphic_data.start_y = start_y;				
			graph_data_struct.graphic_data.end_x = end_x; 		//对顶角坐标					 
			graph_data_struct.graphic_data.end_y = end_y; 		//对顶角坐标	
			UI_Seq = 0x09;
			break;
		}
		case UI_Graph_Circle:
		{
			graph_data_struct.graphic_data.start_x = start_x;				 
			graph_data_struct.graphic_data.start_y = start_y;				
			graph_data_struct.graphic_data.radius = Radius;  //半径
			UI_Seq = 0x0A;
			break;
		}
		case UI_Graph_Ellipse:
		{
			graph_data_struct.graphic_data.start_x = start_x;				 
			graph_data_struct.graphic_data.start_y = start_y;				
			graph_data_struct.graphic_data.end_x = end_x; 	  //x半轴长度					 
			graph_data_struct.graphic_data.end_y = end_y; 		//y半轴长度
			UI_Seq = 0x0B;			
			break;
		}
		case UI_Graph_Arc:
		{
			graph_data_struct.graphic_data.start_x = start_x;				 
			graph_data_struct.graphic_data.start_y = start_y;
			graph_data_struct.graphic_data.start_angle = start_angle;
			graph_data_struct.graphic_data.end_angle = end_angle;			
			graph_data_struct.graphic_data.end_x = end_x; 						 
			graph_data_struct.graphic_data.end_y = end_y;
			UI_Seq = 0x0C;				
			break;
		}
		default: break;
	}
	
	graph_data_struct.UIMsg_head.SOF = 0xA5;
	graph_data_struct.UIMsg_head.DataLength = 21;
	graph_data_struct.UIMsg_head.Seq = UI_Seq;
	
	Append_CRC8_Check_Sum((uint8_t*)&graph_data_struct.UIMsg_head,sizeof(graph_data_struct.UIMsg_head));
	
	graph_data_struct.CmdID = 0x0301;
	
	graph_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
	graph_data_struct.UIdraw_header_id.sender_ID   = Sender_ID;		 
	graph_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;    

	for(src=0;src<3&&name[src]!='\0';src++)
      graph_data_struct.graphic_data.graphic_name[2-src]=name[src];
			
	graph_data_struct.graphic_data.operate_tpye = Operate_tpye;
	graph_data_struct.graphic_data.graphic_tpye = Graphic_tpye;      
	
	graph_data_struct.graphic_data.layer = Layer;             
	graph_data_struct.graphic_data.color = Color;
	graph_data_struct.graphic_data.width = Width;	           
	
	Append_CRC16_Check_Sum((uint8_t*)&graph_data_struct,sizeof(graph_data_struct));    
	memcpy(&tx7_buf[0], (uint8_t *)&graph_data_struct, sizeof(graph_data_struct));
  Usart_SendBuff((uint8_t *)&tx7_buf,sizeof(graph_data_struct));
	
	memcpy(tx7_buf,"\0", sizeof(tx7_buf));
	memset(&graph_data_struct,0,sizeof(graph_data_struct));

}


void Char_Painter(char name[3], char msg[], uint32_t Operate_tpye, uint16_t Sender_ID, uint16_t Receiver_ID,
									uint32_t Layer,uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y, uint8_t Type_Flag)
{
	char  tx7_instruction_buf[30u];
	
	uint8_t i,src,UI_Seq;
	
		switch(Type_Flag)
	{
		case Type_Flag_Cap:
		{
			strcpy(tx7_instruction_buf,"Cap:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=5+sizeof(msg);i<30;i++)            //i=3+sizeof(msg)
				if(i==5+sizeof(msg))
					tx7_instruction_buf[i]='%';
				else
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x02;
			break;
		}
		case Type_Flag_Level:
		{
			strcpy(tx7_instruction_buf,"Level:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=6+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x03;
			break;
		}
		case Type_Flag_Chassis:
		{
			strcpy(tx7_instruction_buf,"Chassis:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=8+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x04;
			break;
		}
		case Type_Flag_Spin:
		{
			strcpy(tx7_instruction_buf,"Spin:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=4+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x05;
			break;
		}
		case Type_Flag_Auto_Aiming:
		{
			strcpy(tx7_instruction_buf,"Aiming:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=7+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x06;
			break;
		}
		case Type_Flag_Shoot_Mode:
		{
			strcpy(tx7_instruction_buf,"Shoot:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=5+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x07;
			break;
		}
		case Type_Flag_Chassis_Speed:
		{
			strcpy(tx7_instruction_buf,"Speed:");	
			Char_Splicing(tx7_instruction_buf, msg);
			for(i=5+sizeof(msg);i<30;i++)
					tx7_instruction_buf[i]=' ';
			memcpy((void *)&char_data_struct.data,(void *)&tx7_instruction_buf, sizeof(tx7_instruction_buf));
			UI_Seq = 0x08;
			break;
		}
		default: break;
	}
	
	char_data_struct.UIMsg_head.SOF = 0xA5;
	char_data_struct.UIMsg_head.DataLength = 6+45;
	char_data_struct.UIMsg_head.Seq = UI_Seq;
	
	Append_CRC8_Check_Sum((uint8_t*)&char_data_struct.UIMsg_head,sizeof(char_data_struct.UIMsg_head));
	
	char_data_struct.CmdID = 0x0301;
	
	char_data_struct.CharUI_header_id.data_cmd_id = 0x0110;
	char_data_struct.CharUI_header_id.sender_ID   = Sender_ID;		 
	char_data_struct.CharUI_header_id.receiver_ID = Receiver_ID;    
	
  for(src=0;src<3&&name[src]!='\0';src++)
      char_data_struct.char_data.graphic_name[2-src]=name[src];
	
	char_data_struct.char_data.operate_tpye = Operate_tpye;
	char_data_struct.char_data.graphic_tpye = 7;
	
	char_data_struct.char_data.layer = Layer;
	char_data_struct.char_data.color = Color;
	char_data_struct.char_data.width = Width;
	
	char_data_struct.char_data.start_angle = Size;
	char_data_struct.char_data.end_angle = strlen((char *)char_data_struct.data);
	
	char_data_struct.char_data.start_x = start_x;
	char_data_struct.char_data.start_y = start_y;
	
	Append_CRC16_Check_Sum((uint8_t*)&char_data_struct,sizeof(char_data_struct));    
	memcpy(&tx7_buf[0], (uint8_t *)&char_data_struct, sizeof(char_data_struct));
  Usart_SendBuff((uint8_t *)&tx7_buf,sizeof(char_data_struct));
	
	memcpy(tx7_buf,"\0", sizeof(tx7_buf));
	memset(&char_data_struct,0,sizeof(char_data_struct));
}

void Num_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye, 
							   uint16_t Sender_ID, uint16_t Receiver_ID, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y,
								 uint32_t start_angle, uint32_t end_angle,int Int, float Float)
{
	uint8_t src;
	
		switch(Graphic_tpye)
	{
		case UI_Graph_Int:
		{
			num_data_struct.UIMsg_head.SOF = 0xA5;
			num_data_struct.UIMsg_head.DataLength = 21;
			num_data_struct.UIMsg_head.Seq = 0x0D;
			
			Append_CRC8_Check_Sum((uint8_t*)&num_data_struct.UIMsg_head,sizeof(num_data_struct.UIMsg_head));
			
			num_data_struct.CmdID = 0x0301;
			
			num_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
			num_data_struct.UIdraw_header_id.sender_ID   = Sender_ID;		 
			num_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;    

			for(src=0;src<3&&name[src]!='\0';src++)
					num_data_struct.graphic_name[2-src]=name[src];
					
			num_data_struct.operate_tpye = Operate_tpye;
			num_data_struct.graphic_tpye = Graphic_tpye;      
			
			num_data_struct.layer = Layer;             
			num_data_struct.color = Color;
			num_data_struct.width = Width;	  

			num_data_struct.start_angle = start_angle;
			num_data_struct.end_angle   = end_angle;
			num_data_struct.start_x 		= start_x;				 
			num_data_struct.start_y 		= start_y;
			num_data_struct.graph_num	 	= Int;			
			
			Append_CRC16_Check_Sum((uint8_t*)&num_data_struct,sizeof(num_data_struct));    
			memcpy(&tx7_buf[0], (uint8_t *)&num_data_struct, sizeof(num_data_struct));
			Usart_SendBuff((uint8_t *)&tx7_buf,sizeof(num_data_struct));
			
			memcpy(tx7_buf,"\0", sizeof(tx7_buf));
			
			break;
		}			
		case UI_Graph_Float:
		{
			num_data_struct.UIMsg_head.SOF = 0xA5;
			num_data_struct.UIMsg_head.DataLength = 21;
			num_data_struct.UIMsg_head.Seq = 0x0D;
			
			Append_CRC8_Check_Sum((uint8_t*)&num_data_struct.UIMsg_head,sizeof(num_data_struct.UIMsg_head));
			
			num_data_struct.CmdID = 0x0301;
			
			num_data_struct.UIdraw_header_id.data_cmd_id = 0x0101;
			num_data_struct.UIdraw_header_id.sender_ID   = Sender_ID;		 
			num_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;    

			for(src=0;src<3&&name[src]!='\0';src++)
					num_data_struct.graphic_name[2-src]=name[src];
					
			num_data_struct.operate_tpye = Operate_tpye;
			num_data_struct.graphic_tpye = Graphic_tpye;      
			
			num_data_struct.layer = Layer;             
			num_data_struct.color = Color;
			num_data_struct.width = Width;	  

			num_data_struct.start_angle = start_angle;
			num_data_struct.end_angle   = end_angle;
			num_data_struct.start_x 		= start_x;				 
			num_data_struct.start_y 		= start_y;
			num_data_struct.graph_num	 	= Float*1000;			
			
			Append_CRC16_Check_Sum((uint8_t*)&num_data_struct,sizeof(num_data_struct));    
			memcpy(&tx7_buf[0], (uint8_t *)&num_data_struct, sizeof(num_data_struct));
			Usart_SendBuff((uint8_t *)&tx7_buf,sizeof(num_data_struct));
			
			memcpy(tx7_buf,"\0", sizeof(tx7_buf));
			
			break;
		}
		default: break;
	}
	
	memset(&num_data_struct,0,sizeof(num_data_struct));

}

void Line_Of_Sight(uint16_t Sender_ID, uint16_t Receiver_ID, uint32_t Layer[7], uint32_t Color[7], uint32_t Width[7], 
									 uint32_t Start_x[7], uint32_t Start_y[7], uint32_t End_x[7], uint32_t End_y[7])
{
	uint8_t i,src;
	char name[]="001002003004005006007";
	
	draw_data_struct.UIMsg_head.SOF = 0xA5;
	draw_data_struct.UIMsg_head.DataLength = 111;
	draw_data_struct.UIMsg_head.Seq = 0x01;
	
	Append_CRC8_Check_Sum((uint8_t*)&draw_data_struct.UIMsg_head,sizeof(draw_data_struct.UIMsg_head));
	
	draw_data_struct.CmdID = 0x0301;
	
	draw_data_struct.UIdraw_header_id.data_cmd_id = 0x0104;
	draw_data_struct.UIdraw_header_id.sender_ID   = Sender_ID;		 
	draw_data_struct.UIdraw_header_id.receiver_ID = Receiver_ID;

	for(i=0;i<7;i++)
	{  
		for(src=0;src<3&&name[src]!='\0';src++)
					draw_data_struct.graphic_data[i].graphic_name[2-src]=name[src+(i*3)];
		
		draw_data_struct.graphic_data[i].operate_tpye = 1;      //图形操作：0空操作；1添加；2修改；3删除
		draw_data_struct.graphic_data[i].graphic_tpye = 0;      //图形类型：0直线；1矩形；2整圆；3椭圆；4圆弧；5浮点数；6整型数；7字符；
		
		draw_data_struct.graphic_data[i].layer = Layer[i];             //图层数0-9
		draw_data_struct.graphic_data[i].color = Color[i];
		draw_data_struct.graphic_data[i].width = Width[i];	//线条宽度
		
		draw_data_struct.graphic_data[i].start_x = Start_x[i];				 //起始坐标x,分辨率1920×1080且全屏时，960，540为屏幕中心
		draw_data_struct.graphic_data[i].start_y = Start_y[i];				 //起始坐标y,分辨率1920×1080且全屏时，960，540为屏幕中心

		draw_data_struct.graphic_data[i].end_x = End_x[i]; 						 //终点坐标x
		draw_data_struct.graphic_data[i].end_y = End_y[i];              //终点坐标y
	}

	Append_CRC16_Check_Sum((uint8_t*)&draw_data_struct,sizeof(draw_data_struct));    
	memcpy(&tx7_buf[0], (uint8_t *)&draw_data_struct, sizeof(draw_data_struct));
  Usart_SendBuff((uint8_t *)&tx7_buf,sizeof(draw_data_struct));
	
	memcpy(tx7_buf,"\0", sizeof(tx7_buf));
	memset(&draw_data_struct,0,sizeof(draw_data_struct));
}

void UI_Delete(uint8_t Del_Operate,uint8_t Del_Layer,uint16_t Sender_ID, uint16_t Receiver_ID)
{
	uint8_t del_buf[255u];
	
	graphic_delete.Del_head.SOF = 0xA5;
	graphic_delete.Del_head.DataLength = 8;
	graphic_delete.Del_head.Seq = 0x10;
	
	Append_CRC8_Check_Sum((uint8_t*)&graphic_delete.Del_head,sizeof(graphic_delete.Del_head));
	
	graphic_delete.CmdID = 0x0301;
	
	graphic_delete.UIdraw_header_id.data_cmd_id = 0x0100;
	graphic_delete.UIdraw_header_id.sender_ID   = Sender_ID;		 
	graphic_delete.UIdraw_header_id.receiver_ID = Receiver_ID; 
	
	graphic_delete.Delete_Operate = Del_Operate;
	graphic_delete.Layer					= Del_Layer;
	
	Append_CRC16_Check_Sum((uint8_t*)&graphic_delete,sizeof(graphic_delete));    
	memcpy(&del_buf[0], (uint8_t *)&graphic_delete, sizeof(graphic_delete));
  Usart_SendBuff((uint8_t *)&del_buf,sizeof(graphic_delete));
	
	memcpy(del_buf,"\0", sizeof(del_buf));
}

void UI_Map(uint16_t Target_Robot_ID,float Target_Position_x,float Target_Position_y,float Reserverd)
{
	uint8_t map_buf[255u];
	
	map_data_struct.Map_head.SOF = 0xA5;
	map_data_struct.Map_head.DataLength = 14;
	map_data_struct.Map_head.Seq = 0x11;
	
	Append_CRC8_Check_Sum((uint8_t*)&map_data_struct.Map_head,sizeof(map_data_struct.Map_head));
	
	map_data_struct.CmdID = 0x0305;
	
	map_data_struct.Target_Robot_ID   = Target_Robot_ID;
	map_data_struct.Target_Position_x = Target_Position_x;
	map_data_struct.Target_Position_y = Target_Position_y;
	map_data_struct.Reserverd         = Reserverd;	
	
	Append_CRC16_Check_Sum((uint8_t*)&map_data_struct,sizeof(map_data_struct));    
	memcpy(&map_buf[0], (uint8_t *)&map_data_struct, sizeof(map_data_struct));
  Usart_SendBuff((uint8_t *)&map_buf,sizeof(map_data_struct));
	
	memcpy(map_buf,"\0", sizeof(map_buf));
	memset(&map_data_struct,0, sizeof(map_data_struct));
}

