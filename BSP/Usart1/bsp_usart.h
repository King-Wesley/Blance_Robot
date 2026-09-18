#ifndef __BSP_USART_H
#define __BSP_USART_H


#include "AllHeader.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t RxTemp;

extern volatile float Glasses_Pitch_Deg;
extern volatile float Glasses_Yaw_Deg;
extern volatile uint8_t Glasses_Imu_Valid;

void Glasses_Imu_Uart2_Start(void);
void Glasses_Imu_Process(void);
void Uart1_Test_Process(void);
void Uart2_Test_Process(void);

void USART1_UART_Init(void);

void USART1_DataByte(uint8_t data_byte);
void USART1_DataString(uint8_t * data_str, uint16_t datasize);
void USART1_Send_Char(char *s);


#endif


