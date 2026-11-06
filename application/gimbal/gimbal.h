#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

typedef struct
{
    uint8_t online;
    uint8_t enabled;
    uint8_t zero_valid;
    uint16_t raw_ecd;
    uint32_t feedback_count;
    uint32_t task_count;
    uint32_t debug_stage;
    uint32_t robot_loop_count;
    uint32_t robot_after_delay_count;
    float total_angle;
    float current_angle;
    float target_angle;
    float ramped_target_angle;
    float velocity;
    float current_feedforward;
    float angle_kp;
    float speed_kp;
    float speed_ki;
    float ramp_speed;
    float static_feedforward;
} GimbalMotorStatus_s;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float yaw_total;
} GimbalImuStatus_s;

void GimbalInit(void);
void GimbalTask(void);
void GimbalHandleCommand(const char *command, float value1, float value2);
void GimbalDebugSelect(const char *command);
void GimbalDebugHandleCommand(const char *command, float value1);
uint8_t GimbalDebugCanMoveDirectly(void); /* 供旧调试命令检查运动互锁 */
void GimbalDebugGetAngles(float *target_angle, float *actual_angle);
void GimbalDebugSetStage(uint32_t stage);
void GimbalDebugSetRobotCounters(uint32_t loop_count, uint32_t after_delay_count);
const GimbalMotorStatus_s *GimbalGetSmallYawStatus(void);
const GimbalImuStatus_s *GimbalGetImuStatus(void);

#endif
