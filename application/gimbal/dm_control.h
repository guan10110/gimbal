#ifndef DM_CONTROL_H
#define DM_CONTROL_H

#include <stdint.h>

typedef struct
{
    uint8_t id;
    uint8_t online;
    uint8_t enabled;
    uint8_t ready;
    uint8_t error_code;
    float position;
    float output_position;
    float output_position_relative;
    float zero_position;
    float motor_zero_position;
    float target_position;
    float target_output_position;
    float velocity;
    float torque;
    float kp;
    float kd;
    float velocity_ref;
    float mos_temperature;
    float rotor_temperature;
    uint32_t last_feedback_ms;
    uint32_t last_mit_feedback_ms;
    uint32_t last_register_feedback_ms;
    uint32_t last_output_position_ms;
    uint32_t mit_feedback_count;
} DMControlStatus_s;

void DMControlInit(void);
void DMControlTask(void);
void DMControlHandleCommand(const char *command, uint8_t id, float value1, float value2);
uint8_t DMControlSetCameraOrigin(uint8_t id);
void DMControlSetCameraTarget(uint8_t id, float angle_deg);
void DMControlHoldCameraPosition(uint8_t id);
const DMControlStatus_s *DMControlGetStatus(uint8_t id);

#endif
