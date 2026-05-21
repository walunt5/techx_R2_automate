#include "protocol.h"
#include "string.h"
// 请包含你的 HAL 库头文件，例如：
#include "stm32f4xx_hal.h" 
#include "usart.h" // 确保包含 huart2 的声明

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


float   g_target_angles[MAX_JOINTS];
float   g_actual_angles[MAX_JOINTS];
uint8_t g_target_updated;

// --- 接收环形队列 ---
volatile uint8_t  rx_ring_buf[RX_RING_BUF_SIZE];
volatile uint16_t rx_head = 0; 
volatile uint16_t rx_tail = 0; 
static uint8_t  uart_rx_byte;

// --- 状态机解析相关 ---
static uint8_t  rx_frame_buf[128]; 
static uint16_t rx_frame_idx = 0;
static uint8_t  rx_state = 0;
static uint8_t  expected_len = 0;
static uint32_t last_rx_tick = 0;  // 用于接收超时防死锁

// --- 发送相关 ---
static uint8_t  uart_tx_buf[64];
static uint32_t last_send_tick = 0;

void Protocol_Init(void)
{
    memset(g_target_angles, 0, sizeof(g_target_angles));
    memset(g_actual_angles, 0, sizeof(g_actual_angles));
    g_target_updated = 0;
    rx_head = 0; rx_tail = 0; rx_state = 0; rx_frame_idx = 0;

    HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_ring_buf, RX_RING_BUF_SIZE);
}

static uint8_t Read_From_RingBuffer(uint8_t *out_byte)
{
    rx_head = (RX_RING_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx)) % RX_RING_BUF_SIZE;

    if(rx_tail == rx_head){
        return 0;
    }

    *out_byte = rx_ring_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_RING_BUF_SIZE;
    return 1;
}

void Protocol_Process(void)
{
    uint8_t byte;

    // 【安全机制 1】：状态机防卡死超时检测 (50ms 未接收完一帧则强制复位)
    if (rx_state != 0 && (HAL_GetTick() - last_rx_tick > 50)) {
        rx_state = 0;
        rx_frame_idx = 0;
    }

    // =========================================================
    // 1. 下发指令解析 (ROS 2 -> STM32，不定长协议)
    // =========================================================
    while (Read_From_RingBuffer(&byte))
    {
        last_rx_tick = HAL_GetTick(); // 只要有数据进账，就喂狗

        switch (rx_state)
        {
            case 0: // 寻找帧头1
                if (byte == 0xAA) { 
                    rx_frame_buf[0] = byte; 
                    rx_frame_idx = 1; 
                    rx_state = 1; 
                }
                break;

            case 1: // 寻找帧头2
                if (byte == 0x55) { 
                    rx_frame_buf[1] = byte; 
                    rx_frame_idx = 2; 
                    rx_state = 2; 
                }
                // 防止连续出现 AA AA 55 导致丢帧
                else if (byte == 0xAA) {
                    rx_state = 1; 
                }
                else { 
                    rx_state = 0; 
                }
                break;

            case 2: // 获取有效的关节数量 Count
                rx_frame_buf[rx_frame_idx++] = byte;
                // 防止收到异常的极大数据或 0
                if (byte > MAX_JOINTS || byte == 0) { 
                    rx_state = 0; 
                    break; 
                }
                // 计算帧长: 帧头(2) + Count(1) + Payload(Count*5) + 校验和(1)
                expected_len = 3 + byte * 5 + 1;
                rx_state = 3;
                break;

            case 3: // 接收动态负载数据并校验
                rx_frame_buf[rx_frame_idx++] = byte;
                if (rx_frame_idx >= expected_len)
                {
                    uint8_t calc_checksum = 0;
                    // 校验范围：从 Count(index 2) 开始异或，一直到倒数第二个字节
                    for (uint16_t j = 2; j < expected_len - 1; j++) {
                        calc_checksum ^= rx_frame_buf[j];
                    }
                        
                    // 校验成功
                    if (calc_checksum == rx_frame_buf[expected_len - 1])
                    {
                        uint8_t count = rx_frame_buf[2]; // 当前包包含了几个关节的数据
                        
                        // 动态解包：只更新包含在此包中的 ID 数据
                        for (uint8_t j = 0; j < count; j++)
                        {
                            uint16_t offset = 3 + j * 5;     // 计算当前关节块的偏移
                            uint8_t motor_id = rx_frame_buf[offset]; // 提取 ID
                            float angle;
                            
                            // memcpy 处理浮点数转换，避免硬件内存对齐错误
                            memcpy(&angle, &rx_frame_buf[offset + 1], sizeof(float));
                            
                            // 赋值至全局数组 (ID 1~5 对应下标 0~4)
                            if (motor_id >= 1 && motor_id <= MAX_JOINTS) {
                                g_target_angles[motor_id - 1] = angle;
                            }
                        }
                        g_target_updated = 1;
                    }
                    rx_state = 0; // 一帧处理完毕，重置状态机
                }
                break;
        }
    }

    // =========================================================
    // 2. 状态上报盲发 (STM32 -> ROS 2，定长协议，100Hz)
    // =========================================================
    uint32_t now = HAL_GetTick();
    if (now - last_send_tick >= 10)
    {
        if(huart2.gState == HAL_UART_STATE_READY){
            last_send_tick = now;
        uint16_t idx = 0;
        
        uart_tx_buf[idx++] = 0xAA;
        uart_tx_buf[idx++] = 0x55;
        uart_tx_buf[idx++] = MAX_JOINTS; // 固定为 0x05
        
        // 盲发：不带 ID，直接按顺序连续压入 5 个 float (前 3 个电机，后 2 个末端状态)
        for (int i = 0; i < MAX_JOINTS; i++) {
            memcpy(&uart_tx_buf[idx], &g_actual_angles[i], sizeof(float));
            idx += sizeof(float);
        }
        
        uint8_t checksum = 0;
        // 校验范围：从 MAX_JOINTS(index 2) 开始一直异或到数据末尾
        for (uint16_t i = 2; i < idx; i++) {
            checksum ^= uart_tx_buf[i];
        }
        uart_tx_buf[idx++] = checksum; // idx 最终变成 24
        
        HAL_UART_Transmit_DMA(&huart2, uart_tx_buf, idx);
        }  
    }
}

// =========================================================
// 3. 末端执行器虚拟关节判定任务
// =========================================================
void EndEffector_Control_Task(void)
{
    // ---- A. 吸盘控制逻辑 (映射为 ID 4，对应数组下标 3) ----
    // TODO: 替换为实际读取真空压力传感器的函数
    float vacuum_pressure = 0.0f; // Read_Vacuum_Pressure(); 

    if (g_target_angles[3] > 0.5f) // 上位机要求吸附
    {
        // Turn_On_Suction_Pump(); // 开启继电器
        if (vacuum_pressure > 0.8f) { // 压力达标阈值
            g_actual_angles[3] = SUCTION_STATUS_SUCKED; // 1.0
        } else {
            g_actual_angles[3] = SUCTION_STATUS_OPENED; // 还没吸稳，先报 0
        }
    }
    else // 上位机要求释放
    {
        // Turn_Off_Suction_Pump(); // 关闭继电器/开破真空阀
        g_actual_angles[3] = SUCTION_STATUS_OPENED; // 0.0
    }

    // ---- B. 夹爪控制逻辑 (映射为 ID 5，对应数组下标 4) ----
    static uint32_t stall_timer = 0;
    static uint32_t empty_timer = 0; // 【安全机制 3】：防抖定时器，防止误判
    
    // TODO: 替换为实际读取夹爪硬件的函数
    float current = 0.0f;  // Read_Gripper_Current();
    float position = 0.0f; // Read_Gripper_Position(); 

    if (g_target_angles[4] > 0.5f) // 上位机要求闭合
    { 
        // 判定 1：抓空防呆 (位置到极点，且持续一定时间)
        if (position <= 0.005f) { 
            empty_timer++;
            if (empty_timer > 20) { // 假设主循环 1ms，20ms 防抖
                // Set_Gripper_Voltage(0); // 切断动力
                g_actual_angles[4] = GRIPPER_STATUS_ERROR; // -1.0
            }
        }
        // 判定 2：真堵转 (电流超限，且位置合理不在极点)
        else if (current > 1.5f && position > 0.01f) { 
            empty_timer = 0; // 只要电流变大，抓空定时器立刻清零
            stall_timer++;
            if (stall_timer > 50) { // 50ms 堵转防抖
                // Set_Gripper_Voltage(HOLD_VOLTAGE); // 降低到维持抓取力的电压
                g_actual_angles[4] = GRIPPER_STATUS_GRASPED; // 2.0
            }
        } 
        // 判定 3：正常闭合运动中
        else {
            empty_timer = 0;
            stall_timer = 0;
            // Set_Gripper_Voltage(MAX_VOLTAGE);
            g_actual_angles[4] = GRIPPER_STATUS_MOVING; // 1.0
        }
    } 
    else // 上位机要求张开
    { 
        empty_timer = 0; 
        stall_timer = 0;

        if (position >= 0.045f) { // 完全张开的阈值
            // Set_Gripper_Voltage(0); 
            g_actual_angles[4] = GRIPPER_STATUS_OPENED; // 0.0
        } 
        else if (current > 1.5f) { 
            // 如果张开时电流异常大，说明机械卡死了
            // Set_Gripper_Voltage(0); 
            g_actual_angles[4] = GRIPPER_STATUS_ERROR; // -1.0
        } 
        else {
            // Set_Gripper_Voltage(-MAX_VOLTAGE); // 反向电压张开
            g_actual_angles[4] = GRIPPER_STATUS_MOVING; // 1.0
        }
    }
}

// protocol.c
void Protocol_On_UART_Error(void)
{
    // 复位环形队列指针
    rx_head = 0;
    rx_tail = 0;
    
    // 顺手复位解析状态机，防止错乱的数据帧拼接
    rx_state = 0;
    rx_frame_idx = 0;
}
