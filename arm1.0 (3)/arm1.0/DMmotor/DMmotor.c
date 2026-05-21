#include "DMmotor.h"
#include "string.h"
#include "stdlib.h"
#include "main.h"
#include "can.h"
#include "usart.h"
#include "sbus.h"
#include "protocol.h"

motor_t motor[4]; // 定义4个电机信息结构体数组
uint8_t is_arm_init = 0;
float pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float vel[4] = {0.5f, 0.5f, 0.5f, 3.0f};
float kp[4] = {5.0f, 5.0f, 5.0f, 5.0f};
float kd[4] = {1.0f, 1.0f, 1.0f, 1.0f};
float tor[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/**
************************************************************************
* @brief:      	DMmotor_init_all: DM电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化四个DM型号的电机，设置默认参数和控制模式。
************************************************************************
**/
void DMmotor_init_all(void)
{
    // 初始化电机结构
    memset(&motor[0], 0, sizeof(motor[0]));
    memset(&motor[1], 0, sizeof(motor[1]));
    memset(&motor[2], 0, sizeof(motor[1]));
    memset(&motor[3], 0, sizeof(motor[1]));

    // 设置0的电机信息
    motor[0].id = 0x01;
    motor[0].ctrl.mode = 1; // 0: MIT模式   1: 位置速度模式   2: 速度模式
    motor[0].cmd.mode = 1;
    // 设置1的电机信息
    motor[1].id = 0x02;
    motor[1].ctrl.mode = 1;
    motor[1].cmd.mode = 1;
    // 设置2的电机信息
    motor[2].id = 0x03;
    motor[2].ctrl.mode = 1;
    motor[2].cmd.mode = 1;
    // 设置3的电机信息
    motor[3].id = 0x04;
    motor[3].ctrl.mode = 1;
    motor[3].cmd.mode = 1;

    HAL_Delay(10);
    ctrl_enable_bigarm(&hcan1, &motor[0], &motor[1]);
    HAL_Delay(10);
    ctrl_enable_smallarm(&hcan1, &motor[2], &motor[3]);
    HAL_Delay(10);
}
/**
************************************************************************
* @brief:      	arm_init: 机械臂初始化函数
* @param:      	void
* @retval:     	void
* @details:    	让机械臂达到设定位置。
************************************************************************
**/
void arm_init(void)
{
    save_pos_zero(&hcan1, &motor[0]);
    save_pos_zero(&hcan1, &motor[1]);
    save_pos_zero(&hcan1, &motor[2]);
    save_pos_zero(&hcan1, &motor[3]);
    HAL_Delay(5);
    // pos[0] = 6.28f * 0.275f;
    // pos[1] = 6.28f * 0.66f;

    // while (1)
    // {
    //     DMmotor_set(&motor[0], pos[0], vel[0], kp[0], kd[0], tor[0]);
    //     HAL_Delay(2);
    //     DMmotor_ctrl_send(&hcan1, &motor[0]);
    //     HAL_Delay(2);
    //     if (motor[0].para.pos > pos[0] * 0.99f && motor[0].para.pos < pos[0] * 1.01f)
    //     {
    //         save_pos_zero(&hcan1, &motor[0]);
    //         pos[0] = 0;
    //     }
    //     DMmotor_set(&motor[1], pos[1], vel[1], kp[1], kd[1], tor[1]);
    //     HAL_Delay(2);
    //     DMmotor_ctrl_send(&hcan1, &motor[1]);
    //     HAL_Delay(2);
    //     if (motor[1].para.pos > pos[1] * 0.99f && motor[1].para.pos < pos[1] * 1.01f)
    //     {
    //         save_pos_zero(&hcan1, &motor[1]);
    //         pos[1] = 0;
    //     }
    //     if (pos[0] == 0 && pos[1] == 0)
    //     {
    //         is_arm_init = 1;
    //         break;
    //     }
    // }
}
/**
************************************************************************
* @brief:      	all_motor_crtl: 电机全控制
* @param:      	void
* @retval:     	void
* @details:    	同时控制所有电机。
************************************************************************
**/
void all_motor_crtl(void)
{
    // float pos0 = ((sbus_rx_data.channel1 - 1024) / 784.0f) * 0.01f;
    // float pos1 = ((sbus_rx_data.channel0 - 1024) / 784.0f) * (-0.01f);
    // pos[0] += pos0;
    // pos[1] += pos1;
    if (sbus_rx_data.sw3 > 1024)
    {
        pos[3] = 118.0f;
        if(motor[3].para.tor > 0.2f && motor[3].para.pos < 7.0f * 1.01f && motor[3].para.pos > 7.0f * 0.99f)
        {
            pos[2] = 0.5f;
        }
        else
        {
            pos[2] = 0.0f;
        }
    }
    else
    {
        pos[3] = 0.0f;
        pos[4] = 0.0f;
    }

    pos[0] = g_target_angles[0];
    pos[1] = g_target_angles[1];

    DMmotor_set(&motor[0], pos[0], vel[0], kp[0], kd[0], tor[0]);
    DMmotor_set(&motor[1], pos[1], vel[1], kp[1], kd[1], tor[1]);
    DMmotor_set(&motor[2], pos[2], vel[2], kp[2], kd[2], tor[2]);
    DMmotor_set(&motor[3], pos[3], vel[3], kp[3], kd[3], tor[3]);
    HAL_Delay(5);
    DMmotor_ctrl_send(&hcan1, &motor[0]);
    HAL_Delay(2);
    DMmotor_ctrl_send(&hcan1, &motor[1]);
    HAL_Delay(2);
    DMmotor_ctrl_send(&hcan1, &motor[2]);
    HAL_Delay(2);
    DMmotor_ctrl_send(&hcan1, &motor[3]);
    HAL_Delay(2);
}
/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	hcan:    指向CAN_HandleTypeDef结构的指针
* @retval:     	mootr:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable_bigarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2)
{
    // 启用0的电机控制
    // DMmotor_enable(hcan, motor1);
    enable_motor_mode(hcan, motor1);
    HAL_Delay(20);
    // 启用0的电机控制
    // DMmotor_enable(hcan, motor2);
    enable_motor_mode(hcan, motor2);
    HAL_Delay(20);
}
void ctrl_enable_smallarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2)
{
    // 启用1的电机控制
    // DMmotor_enable(hcan, motor1);
    enable_motor_mode(hcan, motor1);
    HAL_Delay(20);
    // 启用1的电机控制
    // DMmotor_enable(hcan, motor2);
    enable_motor_mode(hcan, motor2);
    HAL_Delay(20);
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	hcan:   指向CAN_HandleTypeDef结构的指针，包含CAN控制器的相关信息
* @retval:     	motor:   指向motor_t结构体的指针，包含电机控制的相关信息
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable_bigarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2)
{
    // 启用0的电机控制
    // DMmotor_disable(hcan, motor1);
    disable_motor_mode(hcan, motor1);
    HAL_Delay(20);
    // 启用0的电机控制
    // DMmotor_disable(hcan, motor2);
    disable_motor_mode(hcan, motor2);
    HAL_Delay(20);
}
void ctrl_disable_smallarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2)
{
    // 启用1的电机控制
    // DMmotor_disable(hcan, motor1);
    disable_motor_mode(hcan, motor1);
    HAL_Delay(20);
    // 启用1的电机控制
    // DMmotor_disable(hcan, motor2);
    disable_motor_mode(hcan, motor2);
    HAL_Delay(20);
}
/**
************************************************************************
* @brief:      	DMmotor_enable: 启用DMmotor电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式启用相应的模式，通过CAN总线发送启用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void DMmotor_enable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
    case 0:
        enable_motor_mode(hcan, motor);
        break;
    case 1:
        enable_motor_mode(hcan, motor);
        break;
    case 2:
        enable_motor_mode(hcan, motor);
        break;
    }
}
/**
************************************************************************
* @brief:      	DMmotor_disable: 禁用DMmotor电机控制模式函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式禁用相应的模式，通过CAN总线发送禁用命令
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void DMmotor_disable(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
    case 0:
        disable_motor_mode(hcan, motor);
        break;
    case 1:
        disable_motor_mode(hcan, motor);
        break;
    case 2:
        disable_motor_mode(hcan, motor);
        break;
    }
    DMmotor_clear_para(motor);
}
/**
************************************************************************
* @brief:      	DMmotor_ctrl_send: 发送DMmotor电机控制命令函数
* @param[in]:   hcan:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据电机控制模式发送相应的命令到DMmotor电机
*               支持的控制模式包括位置模式、位置速度控制模式和速度控制模式
************************************************************************
**/
void DMmotor_ctrl_send(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    motor->ctrl.kd_set = motor->cmd.kd_set;
    motor->ctrl.kp_set = motor->cmd.kp_set;
    motor->ctrl.pos_set = motor->cmd.pos_set;
    motor->ctrl.vel_set = motor->cmd.vel_set;
    motor->ctrl.tor_set = motor->cmd.tor_set;

    switch (motor->ctrl.mode)
    {
    case 0:
        mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
        break;
    case 1:
        pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
        break;
    case 2:
        speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
        break;
    }
}
/**
************************************************************************
* @brief:      	DMmotor_set: 设置DMmotor电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	根据命令参数设置DMmotor电机的控制参数，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void DMmotor_set(motor_t *motor, float pos_set, float vel_set, float kp_set, float kd_set, float tor_set)
{
    motor->cmd.pos_set = pos_set;
    motor->cmd.vel_set = vel_set;
    motor->cmd.kp_set = kp_set;
    motor->cmd.kd_set = kd_set;
    motor->cmd.tor_set = tor_set;
}
/**
************************************************************************
* @brief:      	DMmotor_clear: 清除DMmotor电机控制参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机相关信息和控制参数
* @retval:     	void
* @details:    	将DMmotor电机的命令参数和控制参数清零，包括位置、速度、
*               比例增益(KP)、微分增益(KD)和扭矩
************************************************************************
**/
void DMmotor_clear_para(motor_t *motor)
{
    motor->cmd.kd_set = 0;
    motor->cmd.kp_set = 0;
    motor->cmd.pos_set = 0;
    motor->cmd.vel_set = 0;
    motor->cmd.tor_set = 0;

    motor->ctrl.kd_set = 0;
    motor->ctrl.kp_set = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
}
/**
************************************************************************
* @brief:      	DMmotor_clear_err: 清除DMmotor电机错误函数
* @param[in]:   hcan: 	 指向CAN控制结构体的指针
* @param[in]:  	motor:   指向电机结构体的指针
* @retval:     	void
* @details:    	根据电机的控制模式，调用对应模式的清除错误函数
************************************************************************
**/
void DMmotor_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    switch (motor->ctrl.mode)
    {
    case 0:
        clear_err(hcan, motor->id, MIT_MODE);
        break;
    case 1:
        clear_err(hcan, motor->id, POS_MODE);
        break;
    case 2:
        clear_err(hcan, motor->id, SPEED_MODE);
        break;
    }
}
/**
************************************************************************
* @brief:      	read_motor_ctrl_fbdata: 发送读取电机反馈数据的命令
* @param[in]:   id:    电机can id
* @retval:     	void
* @details:    	读取电机控制反馈的数据
************************************************************************
**/
void read_motor_ctrl_fbdata(CAN_HandleTypeDef *hcan, uint16_t id)
{
    uint32_t tx_mailbox;
    uint8_t can_id_l = id & 0xFF;        // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位

    uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}
/**
************************************************************************
* @brief:      	DMmotor_fbdata: 获取DMmotor电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DMmotor电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
void DMmotor_fbdata(motor_t *motor, uint8_t *rx_data)
{
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);

    g_actual_angles[motor->id - 1] = motor->para.pos;
}
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
uint8_t edata[8];
void enable_motor_mode(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    // uint16_t id = motor->id + motor->cmd.mode;
    uint16_t id = motor->id;

    edata[0] = 0xFF;
    edata[1] = 0xFF;
    edata[2] = 0xFF;
    edata[3] = 0xFF;
    edata[4] = 0xFF;
    edata[5] = 0xFF;
    edata[6] = 0xFF;
    edata[7] = 0xFC;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    // HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, edata, &tx_mailbox) == HAL_OK)
    {
        motor->start_flag = 1;
    }
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    // uint16_t id = motor->id + motor->cmd.mode;
    uint16_t id = motor->id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    // HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) == HAL_OK)
        motor->start_flag = 0;
}
/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor:    指向motor_t结构的指针
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef *hcan, motor_t *motor)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    // uint16_t id = motor->id + motor->cmd.mode;
    uint16_t id = motor->id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}
/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void clear_err(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    // uint16_t id = motor->id + motor->cmd.mode;
    uint16_t id = motor->id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint32_t tx_mailbox;
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
uint8_t pdata[8];
void pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel)
{
    uint32_t tx_mailbox;
    uint16_t id = motor_id + POS_MODE;
    uint8_t data[8];

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    pdata[0] = *pbuf;
    pdata[1] = *(pbuf + 1);
    pdata[2] = *(pbuf + 2);
    pdata[3] = *(pbuf + 3);

    pdata[4] = *vbuf;
    pdata[5] = *(vbuf + 1);
    pdata[6] = *(vbuf + 2);
    pdata[7] = *(vbuf + 3);

    // memcpy(pdata, &pos, 4);
    // memcpy(pdata + 4, &vel, 4);

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    HAL_CAN_AddTxMessage(hcan, &tx_header, pdata, &tx_mailbox);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float vel)
{
    uint32_t tx_mailbox;
    uint16_t id = motor_id + SPEED_MODE;
    uint8_t data[4];

    uint8_t *vbuf;
    vbuf = (uint8_t *)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);
    // memcpy(data, &vel, 4);

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 4;
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}
