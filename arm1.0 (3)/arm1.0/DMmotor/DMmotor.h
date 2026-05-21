#ifndef _DMMOTOR_H_
#define _DMMOTOR_H_

#include "main.h"
#include "can.h"

#define MIT_MODE 0x000   // 0x000: MIT mode
#define POS_MODE 0x100   // 0x100: Position mode
#define SPEED_MODE 0x200 // 0x200: Speed mode

#define P_MIN -12.5f  // 位置最小值
#define P_MAX 12.5f   // 位置最大值
#define V_MIN -45.0f  // 速度最小值
#define V_MAX 45.0f   // 速度最大值
#define KP_MIN 0.0f   // KP最小值
#define KP_MAX 500.0f // KP最大值
#define KD_MIN 0.0f   // KD最小值
#define KD_MAX 5.0f   // KD最大值
#define T_MIN -18.0f  // 扭矩最小值
#define T_MAX 18.0f   // 扭矩最大值

typedef enum
{
    Motor1 = 0,
    Motor2 = 1,
    Motor3 = 2,
    Motor4 = 3,
    num
} motor_num;

typedef enum
{
    mit_mode = 1,
    pos_mode = 2,
    spd_mode = 3,
} motor_mode;

// 电机回传信息结构体
typedef struct
{
    int id;      // 电机ID
    int state;   // 电机状态
    int p_int;   // 位置值
    int v_int;   // 速度值
    int t_int;   // 扭矩值
    int kp_int;  // Kp值
    int kd_int;  // Kd值
    float pos;   // 实际位置值
    float vel;   // 实际速度值
    float tor;   // 实际扭矩值
    float Kp;    // 实际Kp值
    float Kd;    // 实际Kd值
    float Tmos;  // 驱动上 MOS 的平均温度，单位℃
    float Tcoil; // 驱动上线圈的平均温度，单位℃
} motor_fbpara_t;

// 电机参数设置结构体
typedef struct
{
    uint16_t mode; // 电机模式
    float pos_set; // 位置值
    float vel_set; // 速度值
    float tor_set; // 扭矩值
    float kp_set;  // Kp值
    float kd_set;  // Kd值
} motor_ctrl_t;

typedef struct
{
    uint16_t id;         // 电机ID
    uint8_t start_flag;  // 启动标志
    motor_fbpara_t para; // 电机反馈参数
    motor_ctrl_t ctrl;   // 电机控制参数
    motor_ctrl_t cmd;    // 电机命令参数
} motor_t;

extern motor_t motor[4];
extern uint8_t is_arm_init;                   
extern float pos[4];
extern float vel[4];
extern float kp[4];
extern float kd[4];
extern float tor[4];

void DMmotor_init_all(void);
void arm_init(void);
void all_motor_crtl(void);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void DMmotor_ctrl_send(CAN_HandleTypeDef *hcan, motor_t *motor);
void ctrl_enable_bigarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2);
void ctrl_enable_smallarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2);
void ctrl_disable_bigarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2);
void ctrl_disable_smallarm(CAN_HandleTypeDef *hcan, motor_t *motor1, motor_t *motor2);
void DMmotor_enable(CAN_HandleTypeDef *hcan, motor_t *motor);
void DMmotor_disable(CAN_HandleTypeDef *hcan, motor_t *motor);
void DMmotor_set(motor_t *motor, float pos_set, float vel_set, float kp_set, float kd_set, float tor_set);
void DMmotor_clear_para(motor_t *motor);
void DMmotor_clear_err(CAN_HandleTypeDef *hcan, motor_t *motor);
void read_motor_ctrl_fbdata(CAN_HandleTypeDef *hcan, uint16_t id);
void DMmotor_fbdata(motor_t *motor, uint8_t *rx_data);
void enable_motor_mode(CAN_HandleTypeDef *hcan, motor_t *motor);
void disable_motor_mode(CAN_HandleTypeDef *hcan, motor_t *motor);
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void pos_speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel);
void speed_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float _vel);
void save_pos_zero(CAN_HandleTypeDef *hcan, motor_t *motor);
void clear_err(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);

#endif // _DMMOTOR_H_
