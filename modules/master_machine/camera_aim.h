#ifndef CAMERA_AIM_H
#define CAMERA_AIM_H

#include <stdint.h>
#include "bsp_usart.h"

typedef struct
{
    uint8_t mode; // 算法工作模式，只作状态信息，不用来选择电机
    uint8_t foe_color;
    float yaw_deg;   // 小 yaw 相对 IMU 0 点的目标转角，单位：度
    float pitch_deg; // pitch 相对 IMU 0 点的目标转角，单位：度
    float bullet_speed;
    float big_yaw_deg; // 大 yaw 相对 IMU 0 点的目标转角，单位：度
    int32_t current_robot_quantity;
    int32_t blood_warn_state;
    int32_t game_process;
    float current_state_left_time;
    uint8_t is_success_ourpreempt;
    uint8_t is_success_enemypreempt;
    float quaternion[4]; // w、x、y、z；当前只接收，不参与云台控制
    uint16_t bullet_count;
    float time;
} CameraAimFrame_s;

void CameraAimInit(UART_HandleTypeDef *uart);
void CameraAimInitUsb(void);
void CameraAimUsbReceive(const uint8_t *data, uint16_t length);
uint8_t CameraAimGetLatest(CameraAimFrame_s *frame, uint32_t *sequence,
                           uint32_t *receive_tick);

#endif
