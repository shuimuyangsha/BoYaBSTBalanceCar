#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>
//´®¿Ú½ÓÊÕDMA»º´æ
#define UART_RX_LEN		128
extern uint8_t Uart_Rx[UART_RX_LEN];

void USART1_Config(void);
int  fputc(int ch, FILE *f);
void USART1_Send_Byte(unsigned char byte)  ;
void USART3_Config(void);
void NVIC_Configurationusart3(void);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void PrintChar(char *s);
void UART3_Send_Char(char *s);


#endif /* __USART1_H */
