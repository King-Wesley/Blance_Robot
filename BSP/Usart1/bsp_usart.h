#ifndef __BSP_USART_H
#define __BSP_USART_H


#include "AllHeader.h"


extern UART_HandleTypeDef huart1;
extern uint8_t RxTemp;

void USART1_UART_Init(void);

void USART1_DataByte(uint8_t data_byte);
void USART1_DataString(uint8_t * data_str, uint16_t datasize);
void USART1_Send_Char(char *s);


#endif


