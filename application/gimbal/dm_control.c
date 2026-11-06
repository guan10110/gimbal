#include "dm_control.h"

#include "robot_def.h"

#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "string.h"

#define DM4310_ID 1U
#define DM4310_MST_ID 0x11U
#define DM4340_ID 6U
#define DM4340_MST_ID 0x16U
#define DM8006_ID 3U
#define DM8006_MST_ID 0x13U
#define DM_CONFIG_CAN_ID 0x7FFU

#define DM_REG_CTRL_MODE 0x0AU
#define DM_REG_P_MAX 0x15U
#define DM_REG_V_MAX 0x16U
#define DM_REG_T_MAX 0x17U
#define DM_REG_KP_MAX 0x18U
#define DM_REG_KD_MAX 0x19U
#define DM_REG_POSITION 0x50U
#define DM_REG_OUTPUT_POSITION 0x51U

#define DM_MODE_MIT 1U
#define DM_CONFIG_READY_MASK 0x3FU
#define DM_MIT_KP_MAX 500.0f
#define DM_MIT_KD_MAX 5.0f
#define DM_RAD_TO_DEG 57.2957795f
#define DM_DEG_TO_RAD 0.0174532925f

typedef struct
{
    DMControlStatus_s status;
    CANInstance *can_instance;
    float p_max;
    float v_max;
    float t_max;
    float kp_max;
    float kd_max;
    uint8_t config_flags;
    uint8_t zero_valid;
    uint8_t motor_zero_valid;
    uint8_t camera_zero_valid;
    uint8_t camera_target_active;
    float camera_zero_position;
    uint8_t enable_phase;
    uint8_t enable_request;
    uint32_t last_feedback_ms;
    uint32_t last_stop_ms;
    uint32_t enable_start_ms;
} DMControlMotor_s;

static DMControlMotor_s dm_motors[3];
static const uint8_t query_registers[] = {
    DM_REG_CTRL_MODE, DM_REG_P_MAX, DM_REG_V_MAX, DM_REG_T_MAX,
    DM_REG_KP_MAX, DM_REG_KD_MAX, DM_REG_POSITION, DM_REG_OUTPUT_POSITION,
};
static uint8_t query_index;
static uint32_t last_query_ms;

static DMControlMotor_s *DMControlFindMotor(uint8_t id)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (dm_motors[i].status.id == id)
            return &dm_motors[i];
    }
    return NULL;
}

static float DMClamp(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static float DMGetPitchLimitRad(void)
{
    return DM4310_PITCH_LIMIT_DEG * DM_DEG_TO_RAD;
}

static void DMSet4340Target(DMControlMotor_s *motor, float angle_deg)
{
    /* 调试时保留任意角度控制；折叠和展开命令在下方使用各自标定的位置。 */
    float target = motor->status.zero_position + angle_deg * DM_DEG_TO_RAD;

    motor->status.target_output_position = target;
    motor->status.target_position = target;
}

static void DMSetOutputTarget(DMControlMotor_s *motor, float angle_deg)
{
    float target = motor->status.zero_position + angle_deg * DM_DEG_TO_RAD;

    motor->status.target_output_position = target;
    motor->status.target_position = target;
}

static void DMUpdatePitchTarget(DMControlMotor_s *motor)
{
    float limit_center;
    float target_output;

    if (motor->status.id != DM4310_ID || motor->motor_zero_valid == 0)
        return;

    limit_center = motor->camera_target_active != 0U && motor->camera_zero_valid != 0U ?
                   motor->camera_zero_position : motor->status.zero_position;
    target_output = DMClamp(motor->status.target_output_position,
                            limit_center - DMGetPitchLimitRad(),
                            limit_center + DMGetPitchLimitRad());
    motor->status.target_output_position = target_output;
    /* MIT 控制和反馈使用相同的弧度坐标，保持一一对应，只限制输出目标范围。 */
    motor->status.target_position = target_output;
}

static uint16_t DMFloatToUint(float value, float min, float max, uint8_t bits)
{
    uint32_t span;
    value = DMClamp(value, min, max);
    span = ((uint32_t)1U << bits) - 1U;
    return (uint16_t)((value - min) * (float)span / (max - min));
}

static float DMUintToFloat(uint16_t value, float min, float max, uint8_t bits)
{
    uint32_t span = ((uint32_t)1U << bits) - 1U;
    return (float)value * (max - min) / (float)span + min;
}

static float DMBytesToFloat(const uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static uint8_t DMTransmit(DMControlMotor_s *motor, uint32_t can_id, const uint8_t *data)
{
    uint32_t saved_id;
    uint8_t result;

    saved_id = motor->can_instance->txconf.Identifier;
    motor->can_instance->txconf.Identifier = can_id;
    memcpy(motor->can_instance->tx_buff, data, 8);
    result = CANTransmit(motor->can_instance, 1);
    motor->can_instance->txconf.Identifier = saved_id;
    return result;
}

static void DMSendStop(DMControlMotor_s *motor)
{
    uint8_t data[8];
    memset(data, 0xFF, 7);
    data[7] = 0xFD;
    DMTransmit(motor, motor->status.id, data);
    motor->status.enabled = 0;
    motor->enable_phase = 0;
}

static void DMSendEnable(DMControlMotor_s *motor)
{
    uint8_t data[8];
    memset(data, 0xFF, 7);
    data[7] = 0xFC;
    DMTransmit(motor, motor->status.id, data);
}

static void DMSendRegisterRead(DMControlMotor_s *motor, uint8_t reg)
{
    uint8_t data[8] = {0};
    data[0] = motor->status.id;
    data[2] = 0x33;
    data[3] = reg;
    DMTransmit(motor, DM_CONFIG_CAN_ID, data);
}

static void DMSendRegisterU32(DMControlMotor_s *motor, uint8_t reg, uint32_t value)
{
    uint8_t data[8] = {0};
    data[0] = motor->status.id;
    data[2] = 0x55;
    data[3] = reg;
    data[4] = (uint8_t)(value & 0xFFU);
    data[5] = (uint8_t)((value >> 8) & 0xFFU);
    data[6] = (uint8_t)((value >> 16) & 0xFFU);
    data[7] = (uint8_t)((value >> 24) & 0xFFU);
    DMTransmit(motor, DM_CONFIG_CAN_ID, data);
}

static void DMSendMIT(DMControlMotor_s *motor)
{
    uint8_t data[8];
    uint16_t position;
    uint16_t velocity;
    uint16_t kp;
    uint16_t kd;
    uint16_t torque;

    if (motor->status.ready == 0 || motor->zero_valid == 0)
        return;

    DMUpdatePitchTarget(motor);
    if (motor->status.id == DM4340_ID || motor->status.id == DM8006_ID)
        motor->status.target_position = motor->status.target_output_position;

    position = DMFloatToUint(motor->status.target_position, -motor->p_max, motor->p_max, 16);
    velocity = DMFloatToUint(motor->status.velocity_ref, -motor->v_max, motor->v_max, 12);
    kp = DMFloatToUint(motor->status.kp, 0.0f, motor->kp_max, 12);
    kd = DMFloatToUint(motor->status.kd, 0.0f, motor->kd_max, 12);
    torque = DMFloatToUint(0.0f, -motor->t_max, motor->t_max, 12);

    data[0] = (uint8_t)(position >> 8);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4);
    data[3] = (uint8_t)(((velocity & 0x0FU) << 4) | (kp >> 8));
    data[4] = (uint8_t)kp;
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0x0FU) << 4) | (torque >> 8));
    data[7] = (uint8_t)torque;
    DMTransmit(motor, motor->status.id, data);
}

static void DMStoreRegister(DMControlMotor_s *motor, uint8_t reg, const uint8_t *data)
{
    float value = DMBytesToFloat(&data[4]);

    switch (reg)
    {
    case DM_REG_CTRL_MODE:
        break;
    case DM_REG_P_MAX:
        motor->p_max = value;
        motor->config_flags |= 0x01U;
        break;
    case DM_REG_V_MAX:
        motor->v_max = value;
        motor->config_flags |= 0x02U;
        break;
    case DM_REG_T_MAX:
        motor->t_max = value;
        motor->config_flags |= 0x04U;
        break;
    case DM_REG_KP_MAX:
        /* 0x18 是电机内部参数，不是 MIT 控制使用的 Kp 范围。 */
        motor->kp_max = DM_MIT_KP_MAX;
        motor->config_flags |= 0x08U;
        break;
    case DM_REG_KD_MAX:
        /* 0x19 是电机内部参数，不是 MIT 控制使用的 Kd 范围。 */
        motor->kd_max = DM_MIT_KD_MAX;
        motor->config_flags |= 0x10U;
        break;
    case DM_REG_POSITION:
        motor->status.position = value;
        if (motor->status.id == DM4310_ID && motor->motor_zero_valid == 0)
        {
            motor->status.motor_zero_position = value;
            motor->motor_zero_valid = 1U;
            DMUpdatePitchTarget(motor);
        }
        motor->config_flags |= 0x20U;
        break;
    case DM_REG_OUTPUT_POSITION:
        motor->status.output_position = value;
        motor->status.output_position_relative = value - motor->status.zero_position;
        break;
    default:
        return;
    }

    if ((motor->config_flags & DM_CONFIG_READY_MASK) == DM_CONFIG_READY_MASK &&
        motor->p_max > 0.0f && motor->v_max > 0.0f && motor->t_max > 0.0f &&
        motor->kp_max > 0.0f && motor->kd_max > 0.0f)
    {
        motor->status.ready = 1;
    }
}

static void DMControlDecode(CANInstance *can_instance)
{
    DMControlMotor_s *motor = (DMControlMotor_s *)can_instance->id;
    uint8_t *data = can_instance->rx_buff;
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_torque;

    if (motor == NULL || can_instance->rx_len != 8)
        return;

    /* 寄存器回复和电机反馈使用相同的 MST_ID，因此要检查完整帧头。
     * POS[7:0] 也可能恰好是 0x33；使能时 CTRL_MODE 写入的 0x55 应答
     * 也不能误当成位置、速度和力矩反馈。 */
    if (data[0] == motor->status.id && data[1] == 0U &&
        (data[2] == 0x33U || data[2] == 0x55U || data[2] == 0xAAU))
    {
        motor->last_feedback_ms = HAL_GetTick();
        motor->status.last_feedback_ms = motor->last_feedback_ms;
        motor->status.last_register_feedback_ms = motor->last_feedback_ms;
        motor->status.online = 1;
        if (data[2] == 0x33U)
        {
            DMStoreRegister(motor, data[3], data);
            if (data[3] == DM_REG_OUTPUT_POSITION)
                motor->status.last_output_position_ms = motor->last_feedback_ms;
        }
        return;
    }

    if ((data[0] & 0x0FU) != motor->status.id)
        return;

    motor->last_feedback_ms = HAL_GetTick();
    motor->status.last_feedback_ms = motor->last_feedback_ms;
    motor->status.last_mit_feedback_ms = motor->last_feedback_ms;
    motor->status.mit_feedback_count++;
    motor->status.online = 1;

    motor->status.error_code = data[0] >> 4;
    motor->status.mos_temperature = (float)data[6];
    motor->status.rotor_temperature = (float)data[7];

    if (motor->status.ready == 0)
        return;

    raw_position = (uint16_t)((data[1] << 8) | data[2]);
    raw_velocity = (uint16_t)((data[3] << 4) | (data[4] >> 4));
    raw_torque = (uint16_t)(((data[4] & 0x0FU) << 8) | data[5]);
    motor->status.position = DMUintToFloat(raw_position, -motor->p_max, motor->p_max, 16);
    motor->status.velocity = DMUintToFloat(raw_velocity, -motor->v_max, motor->v_max, 12);
    motor->status.torque = DMUintToFloat(raw_torque, -motor->t_max, motor->t_max, 12);

    /* MIT 反馈位置与当前的位置指令使用相同坐标。所有达妙电机的 VOFA
     * 实际角度都使用该反馈，避免 pitch 受较慢的 0x51 寄存器轮询限制。 */
    motor->status.output_position = motor->status.position;
    motor->status.last_output_position_ms = motor->last_feedback_ms;
    motor->status.output_position_relative =
        motor->status.output_position - motor->status.zero_position;
}

static void DMControlInitMotor(DMControlMotor_s *motor, uint8_t id, uint8_t mst_id)
{
    CAN_Init_Config_s config = {0};

    memset(motor, 0, sizeof(*motor));
    motor->status.id = id;
    motor->status.kp = 0.0f;
    motor->status.kd = 0.0f;
    if (id == DM4310_ID)
    {
        motor->status.kp = DM4310_PITCH_KP;
        motor->status.kd = DM4310_PITCH_KD;
    }
    else if (id == DM4340_ID)
    {
        motor->status.kp = DM4340_ROLL_KP;
        motor->status.kd = DM4340_ROLL_KD;
    }
    else if (id == DM8006_ID)
    {
        motor->status.kp = DM8006_BIG_YAW_KP;
        motor->status.kd = DM8006_BIG_YAW_KD;
    }
#if GIMBAL_ZERO_CONFIGURED
    if (id == DM4310_ID)
        motor->status.zero_position = DM4310_PITCH_ZERO_POSITION_RAD;
    else if (id == DM4340_ID)
        motor->status.zero_position = DM4340_ROLL_ZERO_POSITION_RAD;
    else
        motor->status.zero_position = DM8006_BIG_YAW_ZERO_POSITION_RAD;
    motor->status.target_position = motor->status.zero_position;
    motor->status.target_output_position = motor->status.zero_position;
    motor->zero_valid = 1U;
#endif
    config.can_handle = &hfdcan2;
    config.tx_id = id;
    config.rx_id = mst_id;
    config.can_module_callback = DMControlDecode;
    config.id = motor;
    motor->can_instance = CANRegister(&config);
}

void DMControlInit(void)
{
    DMControlInitMotor(&dm_motors[0], DM4310_ID, DM4310_MST_ID);
    DMControlInitMotor(&dm_motors[1], DM4340_ID, DM4340_MST_ID);
    DMControlInitMotor(&dm_motors[2], DM8006_ID, DM8006_MST_ID);
}

void DMControlTask(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < 3; i++)
    {
        DMControlMotor_s *motor = &dm_motors[i];

        /* 失能时只轮询寄存器，三轴一轮约 600 ms，不能用 300 ms 判掉线。 */
        if ((now - motor->last_feedback_ms) > 1000U)
            motor->status.online = 0;

        if (motor->enable_request != 0)
        {
            if (motor->enable_phase == 0)
            {
                DMSendRegisterU32(motor, DM_REG_CTRL_MODE, DM_MODE_MIT);
                motor->enable_phase = 1;
                motor->enable_start_ms = now;
            }
            else if (motor->enable_phase == 1 && (now - motor->enable_start_ms) >= 60U)
            {
                DMSendEnable(motor);
                motor->enable_phase = 2;
                motor->status.enabled = 1;
            }
            else if (motor->enable_phase == 2)
            {
                DMSendMIT(motor);
            }
        }
    }

    if ((now - last_query_ms) >= 25U)
    {
        uint8_t motor_index = (query_index / (sizeof(query_registers) / sizeof(query_registers[0]))) % 3U;
        uint8_t register_index = query_index % (sizeof(query_registers) / sizeof(query_registers[0]));
        DMSendRegisterRead(&dm_motors[motor_index], query_registers[register_index]);
        query_index = (uint8_t)((query_index + 1U) % (3U * (sizeof(query_registers) / sizeof(query_registers[0]))));
        last_query_ms = now;
    }
}

void DMControlHandleCommand(const char *command, uint8_t id, float value1, float value2)
{
    DMControlMotor_s *motor = DMControlFindMotor(id);

    if (motor == NULL)
        return;

    if (strcmp(command, "e") == 0)
    {
        if (value1 > 0.5f && motor->zero_valid != 0)
        {
            if (motor->enable_request == 0U && motor->status.enabled == 0U)
            {
                motor->enable_request = 1;
                motor->enable_phase = 0;
            }
        }
        else
        {
            motor->enable_request = 0;
            motor->enable_phase = 0;
            if (motor->status.enabled != 0)
                DMSendStop(motor);
        }
    }
    else if (strcmp(command, "s") == 0)
    {
        motor->enable_request = 0;
        DMSendStop(motor);
    }
    else if (strcmp(command, "z") == 0 && motor->zero_valid != 0)
    {
        motor->status.zero_position = motor->status.position;
        motor->status.target_position = motor->status.position;
        motor->status.target_output_position = motor->status.output_position;
        if (motor->status.id == DM4310_ID || motor->status.id == DM4340_ID || motor->status.id == DM8006_ID)
        {
            motor->status.zero_position = motor->status.output_position;
            motor->status.motor_zero_position = motor->status.position;
            motor->status.target_output_position = motor->status.output_position;
            motor->status.target_position = motor->status.output_position;
            motor->motor_zero_valid = 1U;
        }
    }
    else if (strcmp(command, "p") == 0 && motor->zero_valid != 0)
    {
        motor->camera_target_active = 0U;
        if (motor->status.id == DM4310_ID)
        {
            motor->status.target_output_position = motor->status.zero_position + value1 * DM_DEG_TO_RAD;
            DMUpdatePitchTarget(motor);
        }
        else if (motor->status.id == DM4340_ID)
        {
            DMSet4340Target(motor, value1);
        }
        else if (motor->status.id == DM8006_ID)
        {
            DMSetOutputTarget(motor, value1);
        }
        else
        {
            motor->status.target_position = motor->status.zero_position + value1 * DM_DEG_TO_RAD;
        }
        if (motor->status.id != DM4310_ID && motor->p_max > 0.0f)
            motor->status.target_position = DMClamp(motor->status.target_position, -motor->p_max, motor->p_max);
    }
    else if ((strcmp(command, "fold") == 0 || strcmp(command, "deploy") == 0) &&
             motor->status.id == DM4340_ID && motor->zero_valid != 0U)
    {
        motor->camera_target_active = 0U;
        DMSet4340Target(motor, strcmp(command, "fold") == 0 ?
                                  DM4340_FOLDED_POSITION_DEG :
                                  DM4340_DEPLOYED_POSITION_DEG);
    }
    else if (strcmp(command, "v") == 0 && motor->status.ready != 0)
    {
        motor->status.velocity_ref = DMClamp(value1, -motor->v_max, motor->v_max);
    }
    else if (strcmp(command, "akp") == 0)
    {
        motor->status.kp = motor->kp_max > 0.0f ? DMClamp(value1, 0.0f, motor->kp_max) : value1;
    }
    else if (strcmp(command, "skp") == 0)
    {
        motor->status.kd = motor->kd_max > 0.0f ? DMClamp(value1, 0.0f, motor->kd_max) : value1;
    }
}

uint8_t DMControlSetCameraOrigin(uint8_t id)
{
    DMControlMotor_s *motor = DMControlFindMotor(id);

    if (motor == NULL || motor->status.online == 0U || motor->status.ready == 0U ||
        motor->status.last_output_position_ms == 0U)
        return 0U;

    /* 保存当前位置，供相对角度换算或临时保持使用。
     * 正常视觉控制会在上层建立 IMU 0 点后再记录一次；这里不修改 IMU 零点或机械零点。 */
    motor->camera_zero_position = motor->status.output_position;
    motor->camera_zero_valid = 1U;
    motor->camera_target_active = 1U;
    motor->motor_zero_valid = 1U;
    motor->status.target_output_position = motor->camera_zero_position;
    motor->status.target_position = motor->camera_zero_position;
    if (motor->status.id == DM4310_ID)
        DMUpdatePitchTarget(motor);
    return 1U;
}

void DMControlSetCameraTarget(uint8_t id, float angle_deg)
{
    DMControlMotor_s *motor = DMControlFindMotor(id);

    if (motor == NULL || motor->camera_zero_valid == 0U)
        return;

    motor->camera_target_active = 1U;
    /* angle_deg 是相对 IMU 零点的角度，转换到达妙电机的位置坐标。 */
    motor->status.target_output_position =
        motor->camera_zero_position + angle_deg * DM_DEG_TO_RAD;
    motor->status.target_position = motor->status.target_output_position;
    if (motor->status.id == DM4310_ID)
        DMUpdatePitchTarget(motor);
}

void DMControlHoldCameraPosition(uint8_t id)
{
    DMControlMotor_s *motor = DMControlFindMotor(id);

    if (motor == NULL || motor->camera_zero_valid == 0U)
        return;

    motor->camera_target_active = 1U;
    motor->status.target_output_position = motor->status.output_position;
    motor->status.target_position = motor->status.output_position;
    if (motor->status.id == DM4310_ID)
        DMUpdatePitchTarget(motor);
}

const DMControlStatus_s *DMControlGetStatus(uint8_t id)
{
    DMControlMotor_s *motor = DMControlFindMotor(id);
    return motor == NULL ? NULL : &motor->status;
}
