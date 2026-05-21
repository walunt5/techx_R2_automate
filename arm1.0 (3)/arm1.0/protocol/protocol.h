#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

// 适配双臂系统，总计 5 个关节 (3 个物理关节 + 2 个虚拟末端关节)
// ID 1: 吸盘臂主关节 (数组下标 0)
// ID 2: 吸盘臂副关节 (数组下标 1)
// ID 3: 夹爪臂主关节 (数组下标 2)
// ID 4: 虚拟吸盘关节 (数组下标 3)
// ID 5: 虚拟夹爪关节 (数组下标 4)
#define MAX_JOINTS  5

// 定义串口接收环形缓冲区大小 (应对 115200 波特率防溢出)
#define RX_RING_BUF_SIZE 256

/*
 * 末端执行器通信宏定义：
 * ROS 2 下发指令码: 1.0f 触发动作， 0.0f 释放/恢复
 */

// 吸盘状态码 (对应 ID 4)
#define SUCTION_STATUS_OPENED   0.0f  // 空闲/已释放
#define SUCTION_STATUS_SUCKED   1.0f  // 吸附成功 (真空度达标)
#define SUCTION_STATUS_ERROR   -1.0f  // 吸附异常 (气路泄漏等)

// 夹爪状态码 (对应 ID 5)
#define GRIPPER_STATUS_OPENED   0.0f  // 完全张开
#define GRIPPER_STATUS_MOVING   1.0f  // 正在运动中
#define GRIPPER_STATUS_GRASPED  2.0f  // 抓取成功 (堵转)
#define GRIPPER_STATUS_ERROR   -1.0f  // 抓取失败 (抓空/卡死)

extern float g_target_angles[MAX_JOINTS];
extern float g_actual_angles[MAX_JOINTS];
extern uint8_t g_target_updated;
extern volatile uint8_t rx_ring_buf[RX_RING_BUF_SIZE];

void Protocol_Init(void);
void Protocol_Process(void);
void Protocol_UART_RxCallback(void);
void Protocol_On_UART_Error(void);

// 末端执行器闭环控制任务 (需在 main.c 主循环中非阻塞调用)
void EndEffector_Control_Task(void);

#endif // __PROTOCOL_H
