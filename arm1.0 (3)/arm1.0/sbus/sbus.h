#ifndef __SBUS_H__
#define __SBUS_H__

#include "main.h"
#include "usart.h"
#include "can.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"

#define SBUS_FRAME_SIZE 25  // Sbus协议帧大小
#define SBUS_CHANNEL_NUM 10 // 只需要10个通道

typedef struct
{
  uint16_t channel0; // 通道0
  uint16_t channel1; // 通道1
  uint16_t channel2; // 通道2
  uint16_t channel3; // 通道3
  uint16_t sw0;      // 开关0
  uint16_t sw1;      // 开关1
  uint16_t sw2;      // 开关2
  uint16_t sw3;      // 开关3
  uint16_t roll0;    // 滚轮0
  uint16_t roll1;    // 滚轮1

  uint16_t is_enable;      // 是否使能
  uint16_t DMmotor_enable; // 电机是否使能
  uint16_t DMmotor_mode;   // 电机模式ID(模式值: 1-MIT, 2-位置速度, 3-速度)
} sbus_data;

extern uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE]; // 接收缓冲区
extern sbus_data sbus_rx_data;                  // 解码后的数据

void sbus_decode(uint8_t *sbus_rx_buffer, sbus_data *sbus_rx_data); // 解码

#endif // __SBUS_H__
