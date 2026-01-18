#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>

#define SERIAL_UART_NUM      UART_NUM_1      // 串口号，可改为 UART_NUM_0 / UART_NUM_2
#define SERIAL_TXD_PIN       9   // TX 引脚
#define SERIAL_RXD_PIN       11   // RX 引脚
#define SERIAL_BAUDRATE      (115200)        // 波特率

#define SERIAL_BUF_SIZE      (256)

extern char Serial_RxPacket[SERIAL_BUF_SIZE];
extern uint8_t Serial_RxFlag;
extern uint16_t Serial_RxLen;

void Serial_Init(void);
void Serial_SendByte(uint8_t byte);
void Serial_SendArray(uint8_t *array, uint16_t length);
void Serial_SendString(const char *str);
void Serial_SendNumber(uint32_t num, uint8_t length);
void Serial_Printf(const char *format, ...);
void Serial_Task(void *arg);  // 串口接收任务

#endif
