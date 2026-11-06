#include "gimbal.h"

#include "dji_motor.h"
#include "dm_control.h"
#include "camera_aim.h"
#include "ins_task.h"
#include "motor_def.h"
#include "robot_def.h"
#include "string.h"
#include "math.h"

#define SMALL_YAW_ID 1U
#define DJI_ECD_TO_DEG (360.0f / 8192.0f)
#define SMALL_YAW_TASK_PERIOD_S 0.005f
#define SMALL_YAW_RAMP_SPEED_DEFAULT 800.0f // 小 yaw 目标角度的变化速度，单位：度/秒
#define SMALL_YAW_POSITION_DEADBAND_DEG 0.05f
#define SMALL_YAW_SPEED_DEADBAND_DPS 0.5f
#define SMALL_YAW_STATIC_FEEDFORWARD_DEFAULT 0.0f
#define SMALL_YAW_FEEDFORWARD_FULL_DEG 0.9f
#define SMALL_YAW_FEEDFORWARD_SPEED_LIMIT_DPS 20.0f
#define DM_DEBUG_PITCH_ID 1U
#define DM_DEBUG_YAW1_ID 3U
#define DM_DEBUG_YAW2_ID 3U
#define DM_DEBUG_DEFAULT_AMPLITUDE_DEG 30.0f // 正弦运动默认幅值，实际幅值还受每轴限位约束
#define DM_DEBUG_DEFAULT_FREQUENCY_HZ 1.0f  // 正弦运动默认频率；运行中可用 f 命令修改
#define DM_DEBUG_TWO_PI 6.283185307f
#define DM_RAD_TO_DEG 57.2957795f
#define DM_DEBUG_RETURN_SPEED_DPS 30.0f // s 或 z 后回零时的最大速度，单位：度/秒
#define DM_DEBUG_SINE_ENTRY_SPEED_DPS 30.0f
#define DM_DEBUG_RETURN_DEADBAND_DEG 1.0f
#define DM_DEBUG_FOLD_SPEED_DPS 150.0f // roll 折叠/展开时目标角度的变化速度，单位：度/秒
#define DM_DEBUG_STARTUP_DEPLOY_SPEED_DPS 50.0f
#define DM_DEBUG_STARTUP_DEPLOY_TOLERANCE_DEG 1.0f
#define DM_DEBUG_STARTUP_SETTLE_MS 200U
#define DM_DEBUG_STARTUP_FEEDBACK_TIMEOUT_MS 1000U
#define DM_DEBUG_ROLL_ID 6U

typedef enum
{
    DM_DEBUG_NONE = 0,
    DM_DEBUG_PITCH,
    DM_DEBUG_YAW1,
    DM_DEBUG_YAW2,
    DM_DEBUG_ROLL,
} DM_DebugMotor_e;

typedef enum
{
    // z：先让 pitch、yaw1、yaw2 回零，再移动 roll；u：roll 回展开点后释放电机。
    DM_FOLD_STARTUP_CHECK = 0,
    DM_FOLD_STARTUP_UNFOLDING,
    DM_FOLD_STARTUP_REINIT_IMU,
    DM_FOLD_STARTUP_ABORTED,
    DM_FOLD_DEPLOYED,
    DM_FOLD_RETURNING,
    DM_FOLD_FOLDING,
    DM_FOLD_FOLDED,
    DM_FOLD_UNFOLDING,
} DM_FoldState_e;

typedef struct
{
    // 每个正弦运动轴各自保存状态；切换当前轴只切换调参对象，不清除其他轴状态。
    uint8_t enabled;
    uint8_t returning;
    float amplitude_deg;
    float frequency_hz;
    float phase;
    float return_target_deg;
    float start_target_deg;
    uint8_t starting;
    uint8_t direct_target;
} DM_DebugState_s;

static DJIMotorInstance *small_yaw_motor;
static attitude_t *gimbal_imu;
static GimbalMotorStatus_s small_yaw_status;
static GimbalImuStatus_s imu_status;
static float small_yaw_zero_total_angle;
static uint8_t small_yaw_zero_base_set;
static float small_yaw_ref_angle;
static uint8_t small_yaw_ref_initialized;
static float small_yaw_last_error;
static float small_yaw_current_feedforward;
static float small_yaw_ramp_speed_dps = SMALL_YAW_RAMP_SPEED_DEFAULT;
static float small_yaw_static_feedforward = SMALL_YAW_STATIC_FEEDFORWARD_DEFAULT;
static DM_DebugMotor_e dm_debug_motor;
static DM_DebugState_s dm_debug_states[3] = {
    {0U, 0U, DM_DEBUG_DEFAULT_AMPLITUDE_DEG, DM_DEBUG_DEFAULT_FREQUENCY_HZ, 0.0f, 0.0f, 0.0f, 0U, 0U},
    {0U, 0U, DM_DEBUG_DEFAULT_AMPLITUDE_DEG, DM_DEBUG_DEFAULT_FREQUENCY_HZ, 0.0f, 0.0f, 0.0f, 0U, 0U},
    {0U, 0U, DM_DEBUG_DEFAULT_AMPLITUDE_DEG, DM_DEBUG_DEFAULT_FREQUENCY_HZ, 0.0f, 0.0f, 0.0f, 0U, 0U},
};
static uint32_t dm_debug_last_ms;
static DM_FoldState_e dm_fold_state = DM_FOLD_STARTUP_CHECK;
static float dm_roll_motion_target_deg;
static uint32_t camera_aim_last_sequence;
static uint8_t camera_aim_control_active;
static uint8_t camera_aim_timeout_handled;
static uint8_t camera_aim_inhibited; // s 或本地调试后屏蔽视觉，发送 aim 才恢复
static uint32_t camera_aim_last_accepted_tick;
static uint8_t camera_reference_initialized;
static float camera_small_yaw_origin_deg;
static uint32_t startup_roll_settle_ms;
static uint32_t startup_roll_enable_ms;

#if SMALL_YAW_ZERO_ECD <= 8191U
static float GimbalEcdDeltaToDeg(uint16_t ecd, uint16_t zero_ecd)
{
    int32_t delta = (int32_t)ecd - (int32_t)zero_ecd;

    if (delta > 4096)
        delta -= 8192;
    else if (delta < -4096)
        delta += 8192;

    return (float)delta * DJI_ECD_TO_DEG;
}
#endif

static float GimbalClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static float GimbalAbsFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

static float GimbalStepToward(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step)
        return current + max_step;
    if (delta < -max_step)
        return current - max_step;
    return target;
}

static uint8_t GimbalDebugDmId(void)
{
    if (dm_debug_motor == DM_DEBUG_PITCH)
        return DM_DEBUG_PITCH_ID;
    if (dm_debug_motor == DM_DEBUG_ROLL)
        return DM_DEBUG_ROLL_ID;
    return DM_DEBUG_YAW1_ID;
}

static float GimbalDebugLimitDegForMotor(DM_DebugMotor_e motor)
{
    // 正弦调试的软限位以各电机软件零点为基准：pitch +/-70 度，两个 yaw +/-90 度。
    return motor == DM_DEBUG_PITCH ? 70.0f : 90.0f;
}

static float GimbalDebugActualAngle(DM_DebugMotor_e motor);

static DM_DebugState_s *GimbalDebugState(DM_DebugMotor_e motor)
{
    if (motor < DM_DEBUG_PITCH || motor > DM_DEBUG_YAW2)
        return NULL;
    return &dm_debug_states[(uint8_t)motor - 1U];
}

static void GimbalDebugBeginReturn(DM_DebugMotor_e motor)
{
    float current_angle;
    uint8_t id;
    const DMControlStatus_s *dm_status;
    DM_DebugState_s *state = GimbalDebugState(motor);

    if (motor == DM_DEBUG_NONE || state == NULL)
        return;

    current_angle = GimbalDebugActualAngle(motor);
    if (motor == DM_DEBUG_YAW2)
    {
        if (small_yaw_status.enabled == 0U)
            GimbalHandleCommand("e", 1.0f, 0.0f);
    }
    else
    {
        id = motor == DM_DEBUG_PITCH ? DM_DEBUG_PITCH_ID : DM_DEBUG_YAW1_ID;
        dm_status = DMControlGetStatus(id);
        if (dm_status != NULL && dm_status->enabled == 0U)
            DMControlHandleCommand("e", id, 1.0f, 0.0f);
    }
    state->enabled = 0U;
    state->returning = 1U;
    state->starting = 0U;
    state->direct_target = 0U;
    state->return_target_deg = current_angle;
    state->phase = 0.0f;
}

static void GimbalDebugFinishReturn(DM_DebugMotor_e motor)
{
    DM_DebugState_s *state = GimbalDebugState(motor);

    if (state == NULL)
        return;

    state->returning = 0U;
    if (motor == DM_DEBUG_YAW2)
    {
        small_yaw_status.target_angle = 0.0f;
        if (dm_fold_state != DM_FOLD_RETURNING)
            GimbalHandleCommand("e", 0.0f, 0.0f);
        return;
    }

    if (motor == DM_DEBUG_PITCH || motor == DM_DEBUG_YAW1)
    {
        uint8_t id = motor == DM_DEBUG_PITCH ? DM_DEBUG_PITCH_ID : DM_DEBUG_YAW1_ID;
        DMControlHandleCommand("p", id, 0.0f, 0.0f);
        if (dm_fold_state != DM_FOLD_RETURNING)
            DMControlHandleCommand("e", id, 0.0f, 0.0f);
    }
}

static float GimbalDebugActualAngle(DM_DebugMotor_e motor)
{
    const DMControlStatus_s *status;

    if (motor == DM_DEBUG_YAW2)
        return small_yaw_status.current_angle;
    // 达妙反馈位置用弧度表示，调试命令和 VOFA 曲线统一用度。
    status = DMControlGetStatus(motor == DM_DEBUG_PITCH ? DM_DEBUG_PITCH_ID :
                                (motor == DM_DEBUG_ROLL ? DM_DEBUG_ROLL_ID : DM_DEBUG_YAW1_ID));
    return status == NULL ? 0.0f : status->output_position_relative * DM_RAD_TO_DEG;
}

static uint8_t GimbalHasFreshDMPosition(const DMControlStatus_s *status, uint32_t now)
{
    return status != NULL && status->online != 0U && status->ready != 0U &&
           status->last_output_position_ms != 0U &&
           (now - status->last_output_position_ms) < DM_DEBUG_STARTUP_FEEDBACK_TIMEOUT_MS;
}

static void GimbalUpdateStartupDeploy(float dt, uint32_t now)
{
    const DMControlStatus_s *roll = DMControlGetStatus(DM_DEBUG_ROLL_ID);
    float actual_roll_deg;

    if (dm_fold_state == DM_FOLD_STARTUP_CHECK)
    {
        const DMControlStatus_s *pitch = DMControlGetStatus(DM_DEBUG_PITCH_ID);
        const DMControlStatus_s *big_yaw = DMControlGetStatus(DM_DEBUG_YAW1_ID);

        if (GimbalHasFreshDMPosition(roll, now) == 0U)
            return;

        actual_roll_deg = roll->output_position_relative * DM_RAD_TO_DEG;
        if (GimbalAbsFloat(actual_roll_deg - DM4340_DEPLOYED_POSITION_DEG) <=
            DM_DEBUG_STARTUP_DEPLOY_TOLERANCE_DEG)
        {
            // 本来就在展开位置也要重新建立本次姿态参考。
            INS_RequestReinit();
            dm_fold_state = DM_FOLD_STARTUP_REINIT_IMU;
            return;
        }

        // 展开 roll 前先让其余三个轴锁住当前位置，避免折叠机构带动它们。
        if (GimbalHasFreshDMPosition(pitch, now) == 0U ||
            GimbalHasFreshDMPosition(big_yaw, now) == 0U ||
            small_yaw_status.online == 0U)
            return;
#if GIMBAL_ZERO_CONFIGURED
        if (small_yaw_zero_base_set == 0U)
            return;
#endif
        if (DMControlSetCameraOrigin(DM_DEBUG_PITCH_ID) == 0U ||
            DMControlSetCameraOrigin(DM_DEBUG_YAW1_ID) == 0U)
            return;

        if (small_yaw_status.zero_valid != 0U)
        {
            GimbalHandleCommand("p", small_yaw_status.current_angle, 0.0f);
            GimbalHandleCommand("e", 1.0f, 0.0f);
        }
        DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);

        dm_roll_motion_target_deg = actual_roll_deg;
        DMControlHandleCommand("p", DM_DEBUG_ROLL_ID, dm_roll_motion_target_deg, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);
        startup_roll_enable_ms = now;
        dm_fold_state = DM_FOLD_STARTUP_UNFOLDING;
        startup_roll_settle_ms = 0U;
        return;
    }

    if (dm_fold_state == DM_FOLD_STARTUP_UNFOLDING)
    {
        // 达妙先切 MIT 模式，60 ms 后才使能；给首个反馈留出时间。
        if ((now - startup_roll_enable_ms) < 150U)
            return;
        if (roll == NULL || roll->online == 0U || roll->ready == 0U ||
            roll->last_mit_feedback_ms == 0U || (now - roll->last_mit_feedback_ms) >= 300U)
        {
            // 反馈中断后停止本次开机展开，人工检查后用 u 重试。
            DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
            GimbalHandleCommand("e", 0.0f, 0.0f);
            dm_fold_state = DM_FOLD_STARTUP_ABORTED;
            return;
        }

        actual_roll_deg = roll->output_position_relative * DM_RAD_TO_DEG;
        dm_roll_motion_target_deg = GimbalStepToward(dm_roll_motion_target_deg,
                                                     DM4340_DEPLOYED_POSITION_DEG,
                                                     DM_DEBUG_STARTUP_DEPLOY_SPEED_DPS * dt);
        DMControlHandleCommand("p", DM_DEBUG_ROLL_ID, dm_roll_motion_target_deg, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);

        if (GimbalAbsFloat(actual_roll_deg - DM4340_DEPLOYED_POSITION_DEG) <=
            DM_DEBUG_STARTUP_DEPLOY_TOLERANCE_DEG)
        {
            if (startup_roll_settle_ms == 0U)
                startup_roll_settle_ms = now;
            else if ((now - startup_roll_settle_ms) >= DM_DEBUG_STARTUP_SETTLE_MS)
            {
                DMControlHandleCommand("p", DM_DEBUG_ROLL_ID,
                                       DM4340_DEPLOYED_POSITION_DEG, 0.0f);
                // 保持各轴位置完成姿态采样，避免失能下垂后记录了错误原点。
                INS_RequestReinit();
                dm_fold_state = DM_FOLD_STARTUP_REINIT_IMU;
            }
        }
        else
            startup_roll_settle_ms = 0U;
        return;
    }

    if (dm_fold_state == DM_FOLD_STARTUP_REINIT_IMU && INS_IsReinitDone() != 0U)
    {
        camera_reference_initialized = 0U;
        camera_aim_control_active = 0U;
        dm_fold_state = DM_FOLD_DEPLOYED;
    }
}

static void GimbalUpdateDebugSine(void)
{
    uint32_t now;
    float dt;
    float target;
    float entry_angle;
    DM_DebugState_s *state;
    DM_DebugMotor_e motor;

    now = HAL_GetTick();
    dt = (float)(now - dm_debug_last_ms) * 0.001f;
    dm_debug_last_ms = now;
    if (dt > 0.1f)
        dt = 0.1f;

    if (dm_fold_state == DM_FOLD_STARTUP_CHECK ||
        dm_fold_state == DM_FOLD_STARTUP_UNFOLDING ||
        dm_fold_state == DM_FOLD_STARTUP_REINIT_IMU)
    {
        GimbalUpdateStartupDeploy(dt, now);
        return;
    }

    // 正弦只用于展开状态下的手动调试；相机控制和折叠状态不运行正弦轨迹。
    if (camera_aim_control_active == 0U &&
        (dm_fold_state == DM_FOLD_DEPLOYED || dm_fold_state == DM_FOLD_RETURNING))
    {
        // 折叠前只允许已经启动的轴按缓坡回零，不再继续正弦运动。
        for (motor = DM_DEBUG_PITCH; motor <= DM_DEBUG_YAW2; motor++)
        {
            state = GimbalDebugState(motor);
            if (state == NULL || (state->enabled == 0U && state->returning == 0U &&
                                  state->direct_target == 0U))
                continue;
            if (dm_fold_state == DM_FOLD_RETURNING && state->returning == 0U)
                continue;
            if (state->direct_target != 0U)
                continue;

            if (state->returning != 0U)
            {
                state->return_target_deg = GimbalStepToward(state->return_target_deg, 0.0f,
                                                            DM_DEBUG_RETURN_SPEED_DPS * dt);
                target = state->return_target_deg;
                if (GimbalAbsFloat(GimbalDebugActualAngle(motor)) <= DM_DEBUG_RETURN_DEADBAND_DEG &&
                    GimbalAbsFloat(target) <= DM_DEBUG_RETURN_DEADBAND_DEG)
                {
                    GimbalDebugFinishReturn(motor);
                    continue;
                }
            }
            else if (state->starting != 0U)
            {
                /* 正弦运动仍以软件零点为中心。若起始角度超出幅值范围，
                 * 先缓慢移动到最近端点，再从对应相位接入正弦轨迹。 */
                entry_angle = GimbalClampFloat(state->start_target_deg,
                                               -state->amplitude_deg, state->amplitude_deg);
                state->start_target_deg = GimbalStepToward(state->start_target_deg, entry_angle,
                                                           DM_DEBUG_SINE_ENTRY_SPEED_DPS * dt);
                target = state->start_target_deg;
                if (target == entry_angle)
                {
                    state->phase = state->amplitude_deg > 0.0f ?
                        asinf(GimbalClampFloat(entry_angle / state->amplitude_deg, -1.0f, 1.0f)) : 0.0f;
                    state->starting = 0U;
                }
            }
            else
            {
                target = state->amplitude_deg * sinf(state->phase);
                state->phase += DM_DEBUG_TWO_PI * state->frequency_hz * dt;
                while (state->phase >= DM_DEBUG_TWO_PI)
                    state->phase -= DM_DEBUG_TWO_PI;
            }
            target = GimbalClampFloat(target, -GimbalDebugLimitDegForMotor(motor),
                                      GimbalDebugLimitDegForMotor(motor));
            if (motor == DM_DEBUG_YAW2)
                small_yaw_status.target_angle = target;
            else
                DMControlHandleCommand("p", motor == DM_DEBUG_PITCH ? DM_DEBUG_PITCH_ID : DM_DEBUG_YAW1_ID,
                                       target, 0.0f);
        }
    }

    if (dm_fold_state == DM_FOLD_RETURNING)
    {
        uint8_t all_at_zero = 1U;
        for (motor = DM_DEBUG_PITCH; motor <= DM_DEBUG_YAW2; motor++)
        {
            state = GimbalDebugState(motor);
            if (state->returning != 0U || GimbalAbsFloat(GimbalDebugActualAngle(motor)) > DM_DEBUG_RETURN_DEADBAND_DEG)
            {
                all_at_zero = 0U;
                break;
            }
        }
        if (all_at_zero != 0U)
        {
            dm_roll_motion_target_deg = GimbalDebugActualAngle(DM_DEBUG_ROLL);
            DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
            DMControlHandleCommand("p", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);
            DMControlHandleCommand("p", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
            GimbalHandleCommand("e", 1.0f, 0.0f);
            small_yaw_status.target_angle = 0.0f;
            DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);
            dm_fold_state = DM_FOLD_FOLDING;
        }
    }
    else if (dm_fold_state == DM_FOLD_FOLDING)
    {
        dm_roll_motion_target_deg = GimbalStepToward(dm_roll_motion_target_deg,
                                                    DM4340_FOLDED_POSITION_DEG,
                                                    DM_DEBUG_FOLD_SPEED_DPS * dt);
        DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_ROLL_ID, dm_roll_motion_target_deg, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
        GimbalHandleCommand("e", 1.0f, 0.0f);
        small_yaw_status.target_angle = 0.0f;
        if (GimbalAbsFloat(GimbalDebugActualAngle(DM_DEBUG_ROLL) - DM4340_FOLDED_POSITION_DEG) < 1.0f)
            dm_fold_state = DM_FOLD_FOLDED;
    }
    else if (dm_fold_state == DM_FOLD_FOLDED)
    {
        DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_ROLL_ID, DM4340_FOLDED_POSITION_DEG, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
        GimbalHandleCommand("e", 1.0f, 0.0f);
        small_yaw_status.target_angle = 0.0f;
    }
    else if (dm_fold_state == DM_FOLD_UNFOLDING)
    {
        dm_roll_motion_target_deg = GimbalStepToward(dm_roll_motion_target_deg,
                                                    DM4340_DEPLOYED_POSITION_DEG,
                                                    DM_DEBUG_FOLD_SPEED_DPS * dt);
        DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_ROLL_ID, dm_roll_motion_target_deg, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
        DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);
        DMControlHandleCommand("p", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
        GimbalHandleCommand("e", 1.0f, 0.0f);
        small_yaw_status.target_angle = 0.0f;
        if (GimbalAbsFloat(GimbalDebugActualAngle(DM_DEBUG_ROLL) - DM4340_DEPLOYED_POSITION_DEG) < 1.0f)
        {
            DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
            GimbalHandleCommand("e", 0.0f, 0.0f);
            dm_fold_state = DM_FOLD_DEPLOYED;
        }
    }
}

static void GimbalUpdateSmallYawReference(void)
{
    float error;
    float abs_error;
    float abs_velocity;
    float error_scale;
    float speed_scale;

    if (small_yaw_ref_initialized == 0U)
    {
        small_yaw_ref_angle = small_yaw_status.current_angle;
        small_yaw_last_error = 0.0f;
        small_yaw_ref_initialized = 1U;
    }

    // 先限制目标变化速度；误差换向时清掉速度环积分，避免旧积分推着电机继续走。
    small_yaw_ref_angle = GimbalStepToward(small_yaw_ref_angle,
                                           small_yaw_status.target_angle,
                                           small_yaw_ramp_speed_dps * SMALL_YAW_TASK_PERIOD_S);

    error = small_yaw_ref_angle - small_yaw_status.current_angle;
    abs_error = GimbalAbsFloat(error);
    abs_velocity = GimbalAbsFloat(small_yaw_status.velocity);

    if (error * small_yaw_last_error < 0.0f)
    {
        small_yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
        small_yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
    }
    small_yaw_last_error = error;

    error_scale = GimbalClampFloat(abs_error / SMALL_YAW_FEEDFORWARD_FULL_DEG, 0.0f, 1.0f);
    speed_scale = GimbalClampFloat(1.0f - abs_velocity / SMALL_YAW_FEEDFORWARD_SPEED_LIMIT_DPS,
                                   0.0f, 1.0f);
    small_yaw_current_feedforward = small_yaw_static_feedforward * error_scale * speed_scale;
    if (error < 0.0f)
        small_yaw_current_feedforward = -small_yaw_current_feedforward;
}

static void GimbalUpdateStatus(void)
{
    // IMU姿态作为视觉角度的共同参考；各电机编码器仍负责对应关节的位置闭环。
    if (gimbal_imu != NULL)
    {
        imu_status.yaw = gimbal_imu->Yaw;
        imu_status.pitch = gimbal_imu->Pitch;
        imu_status.roll = gimbal_imu->Roll;
        imu_status.yaw_total = gimbal_imu->YawTotalAngle;
    }

    if (small_yaw_motor == NULL)
        return;

    small_yaw_status.online = DaemonIsOnline(small_yaw_motor->daemon);
    small_yaw_status.raw_ecd = small_yaw_motor->measure.ecd;
    small_yaw_status.feedback_count = small_yaw_motor->feedback_count;
    small_yaw_status.total_angle = small_yaw_motor->measure.total_angle;
    small_yaw_status.velocity = small_yaw_motor->measure.speed_aps;
    small_yaw_status.ramped_target_angle = small_yaw_ref_angle;
    small_yaw_status.current_feedforward = small_yaw_current_feedforward;
    small_yaw_status.angle_kp = small_yaw_motor->motor_controller.angle_PID.Kp;
    small_yaw_status.speed_kp = small_yaw_motor->motor_controller.speed_PID.Kp;
    small_yaw_status.speed_ki = small_yaw_motor->motor_controller.speed_PID.Ki;
    small_yaw_status.ramp_speed = small_yaw_ramp_speed_dps;
    small_yaw_status.static_feedforward = small_yaw_static_feedforward;

#if SMALL_YAW_ZERO_ECD <= 8191U
    small_yaw_status.zero_valid = small_yaw_status.online;
    small_yaw_status.current_angle = small_yaw_status.total_angle - small_yaw_zero_total_angle;
#else
    small_yaw_status.zero_valid = 0U;
    small_yaw_status.current_angle = 0.0f;
#endif
}

static void GimbalTryCaptureCameraReference(void)
{
    const DMControlStatus_s *dm_pitch = DMControlGetStatus(DM_DEBUG_PITCH_ID);
    const DMControlStatus_s *dm_big_yaw = DMControlGetStatus(DM_DEBUG_YAW1_ID);
    uint32_t now = HAL_GetTick();

    if (dm_fold_state != DM_FOLD_DEPLOYED || camera_reference_initialized != 0U ||
        small_yaw_status.online == 0U || small_yaw_status.zero_valid == 0U ||
        dm_pitch == NULL || dm_big_yaw == NULL ||
        dm_pitch->online == 0U || dm_big_yaw->online == 0U ||
        dm_pitch->ready == 0U || dm_big_yaw->ready == 0U ||
        dm_pitch->last_output_position_ms == 0U ||
        dm_big_yaw->last_output_position_ms == 0U ||
        (now - dm_pitch->last_output_position_ms) >= 1000U ||
        (now - dm_big_yaw->last_output_position_ms) >= 1000U)
        return;

    /* 展开后 IMU 已建立本次 0 点姿态。这里读取各电机位置只是做坐标换算，
     * 让视觉的 IMU 零点角度能变成电机位置指令；它不是云台的参考零点。 */
    if (DMControlSetCameraOrigin(DM_DEBUG_PITCH_ID) == 0U ||
        DMControlSetCameraOrigin(DM_DEBUG_YAW1_ID) == 0U)
        return;

    camera_small_yaw_origin_deg = small_yaw_status.current_angle;
    camera_reference_initialized = 1U;
    // 建立参考之前收到的目标不执行，等视觉在新参考建立后继续发帧。
    CameraAimFrame_s pending;
    uint32_t pending_tick;
    CameraAimGetLatest(&pending, &camera_aim_last_sequence, &pending_tick);
}

static void GimbalUpdateCameraTargets(void)
{
    CameraAimFrame_s frame;
    const DMControlStatus_s *dm_pitch;
    const DMControlStatus_s *dm_big_yaw;
    uint32_t sequence;
    uint32_t receive_tick;
    uint32_t now = HAL_GetTick();

    if (CameraAimGetLatest(&frame, &sequence, &receive_tick) == 0U)
        return;
    if (camera_reference_initialized == 0U || camera_aim_inhibited != 0U)
        return;

    if (dm_fold_state != DM_FOLD_DEPLOYED)
    {
        camera_aim_last_sequence = sequence;
        return;
    }

    // 只有通过角度检查的帧才续期；持续收到越界帧也必须停住。
    if (camera_aim_control_active != 0U &&
        (now - camera_aim_last_accepted_tick) > 100U)
    {
        if (camera_aim_control_active != 0U && camera_aim_timeout_handled == 0U)
        {
            dm_pitch = DMControlGetStatus(DM_DEBUG_PITCH_ID);
            dm_big_yaw = DMControlGetStatus(DM_DEBUG_YAW1_ID);
            if (small_yaw_status.online != 0U && dm_pitch != NULL && dm_big_yaw != NULL)
            {
                GimbalHandleCommand("p", small_yaw_status.current_angle, 0.0f);
                DMControlHoldCameraPosition(DM_DEBUG_PITCH_ID);
                DMControlHoldCameraPosition(DM_DEBUG_YAW1_ID);
            }
            camera_aim_timeout_handled = 1U;
        }
        // 本周期若已有新有效帧，可以在下方恢复更新目标。
    }

    if (sequence == camera_aim_last_sequence || (now - receive_tick) > 100U)
        return;

    camera_aim_last_sequence = sequence;
    dm_pitch = DMControlGetStatus(DM_DEBUG_PITCH_ID);
    dm_big_yaw = DMControlGetStatus(DM_DEBUG_YAW1_ID);
    if (small_yaw_status.online == 0U ||
        GimbalHasFreshDMPosition(dm_pitch, now) == 0U ||
        GimbalHasFreshDMPosition(dm_big_yaw, now) == 0U)
    {
        // 位置不可信时不继续发运动目标，恢复连接后用 aim 重新接管。
        if (camera_aim_control_active != 0U)
        {
            GimbalHandleCommand("e", 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
            camera_aim_control_active = 0U;
            camera_aim_inhibited = 1U;
        }
        return;
    }
    // 角度相对本次建立的 IMU 0 点，分别对应三个关节，单位是度。
    // 越过以启动姿态为中心的安全范围时直接丢弃。
    // yaw / bigYaw 限幅为 +/-90 度，pitch 限幅为 +/-70 度。
    if (
        GimbalAbsFloat(frame.yaw_deg) > 90.0f ||
        GimbalAbsFloat(frame.pitch_deg) > 70.0f ||
        GimbalAbsFloat(frame.big_yaw_deg) > 90.0f)
        return;

    camera_aim_timeout_handled = 0U;
    camera_aim_last_accepted_tick = receive_tick;

    // 视觉接管时清除正弦和回零状态，防止两套轨迹相互覆盖。
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        dm_debug_states[i].enabled = 0U;
        dm_debug_states[i].returning = 0U;
        dm_debug_states[i].starting = 0U;
    }

    /* 三个字段都是相对固定 IMU 0 点的独立目标角；新帧覆盖旧目标，不做累加。 */
    GimbalHandleCommand("p", camera_small_yaw_origin_deg + frame.yaw_deg, 0.0f);
    DMControlSetCameraTarget(DM_DEBUG_PITCH_ID, frame.pitch_deg);
    DMControlSetCameraTarget(DM_DEBUG_YAW1_ID, frame.big_yaw_deg);

    GimbalHandleCommand("e", 1.0f, 0.0f);
    DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 1.0f, 0.0f);
    DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 1.0f, 0.0f);
    camera_aim_control_active = 1U;
}

void GimbalInit(void)
{
    Motor_Init_Config_s small_yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = SMALL_YAW_ID,
        },
        .controller_param_init_config = {
            .current_feedforward_ptr = &small_yaw_current_feedforward,
            .angle_PID = {
                .Kp = 88.8f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = SMALL_YAW_POSITION_DEADBAND_DEG,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 700.0f,
            },
            .speed_PID = {
                .Kp = 12.0f,
                .Ki = 0.2f,
                .Kd = 0.0f,
                .DeadBand = SMALL_YAW_SPEED_DEADBAND_DPS,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 800.0f,
                .MaxOut = 10000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
        .current_control_mode = 1U,
    };

    memset(&small_yaw_status, 0, sizeof(small_yaw_status));
    memset(&imu_status, 0, sizeof(imu_status));
    gimbal_imu = INS_Init();
    small_yaw_motor = DJIMotorInit(&small_yaw_config);
    DJIMotorStop(small_yaw_motor);

    DMControlInit();
}

void GimbalTask(void)
{
    // 更新反馈后处理相机目标；没有相机控制时运行本地正弦/折叠调试，最后执行电机控制任务。
    small_yaw_status.task_count++;
    small_yaw_status.debug_stage = 101U;
    GimbalUpdateStatus();
    small_yaw_status.debug_stage = 102U;

    GimbalUpdateDebugSine();

#if SMALL_YAW_ZERO_ECD <= 8191U
    if (small_yaw_status.online != 0U && small_yaw_zero_base_set == 0U)
    {
        // 使用手动测得的编码器零点，折算成本次上电的多圈角度坐标。
        small_yaw_zero_total_angle = small_yaw_status.total_angle -
                                     GimbalEcdDeltaToDeg(small_yaw_status.raw_ecd,
                                                         SMALL_YAW_ZERO_ECD);
        small_yaw_zero_base_set = 1U;
        small_yaw_status.current_angle =
            GimbalEcdDeltaToDeg(small_yaw_status.raw_ecd, SMALL_YAW_ZERO_ECD);
        small_yaw_status.zero_valid = 1U;
    }
#endif

    GimbalTryCaptureCameraReference();
    GimbalUpdateCameraTargets();

    if (small_yaw_status.enabled != 0U && small_yaw_status.zero_valid != 0U)
    {
        GimbalUpdateSmallYawReference();
        DJIMotorEnable(small_yaw_motor);
        DJIMotorSetRef(small_yaw_motor, small_yaw_zero_total_angle + small_yaw_ref_angle);
    }
    else
    {
        small_yaw_ref_initialized = 0U;
        small_yaw_last_error = 0.0f;
        small_yaw_current_feedforward = 0.0f;
        small_yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
        small_yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
        DJIMotorStop(small_yaw_motor);
    }
    small_yaw_status.debug_stage = 103U;

#if DM_CONTROL_ENABLED
    DMControlTask();
#endif
    small_yaw_status.debug_stage = 104U;
}
void GimbalHandleCommand(const char *command, float value1, float value2)
{
    if (small_yaw_motor == NULL)
        return;

    if (strcmp(command, "e") == 0)
        small_yaw_status.enabled = value1 > 0.5f;
    else if (strcmp(command, "s") == 0)
        small_yaw_status.enabled = 0U;
    else if (strcmp(command, "p") == 0 && small_yaw_status.zero_valid != 0U)
        small_yaw_status.target_angle = value1;
    else if (strcmp(command, "akp") == 0)
        small_yaw_motor->motor_controller.angle_PID.Kp = value1;
    else if (strcmp(command, "skp") == 0)
        small_yaw_motor->motor_controller.speed_PID.Kp = value1;
    else if (strcmp(command, "i") == 0)
        small_yaw_motor->motor_controller.speed_PID.Ki = value1;
    else if (strcmp(command, "r") == 0)
        small_yaw_ramp_speed_dps = GimbalClampFloat(value1, 50.0f, 2000.0f);
    else if (strcmp(command, "f") == 0)
    {
        small_yaw_static_feedforward = GimbalClampFloat(value1, 0.0f, 4000.0f);
    }
}

void GimbalDebugSelect(const char *command)
{
    DM_DebugMotor_e next = DM_DEBUG_NONE;

    // 选择后续命令作用的电机，同时切换 VOFA 显示；已启动的其他电机不会因此停止。
    if (strcmp(command, "pitch") == 0)
        next = DM_DEBUG_PITCH;
    else if (strcmp(command, "yaw1") == 0)
        next = DM_DEBUG_YAW1;
    else if (strcmp(command, "yaw") == 0 || strcmp(command, "yaw2") == 0)
        next = DM_DEBUG_YAW2;
    else if (strcmp(command, "roll") == 0)
        next = DM_DEBUG_ROLL;
    else
        return;

    dm_debug_motor = next;
}

void GimbalDebugHandleCommand(const char *command, float value1)
{
    uint8_t id;
    DM_DebugState_s *state = GimbalDebugState(dm_debug_motor);

    // e 启停正弦运动，a 直接设置目标角度，amp/f 调正弦幅值和频率，akp/skp 调增益。
    if (strcmp(command, "s") == 0)
    {
        uint8_t was_camera_active = camera_aim_control_active;
        camera_aim_inhibited = 1U;
        camera_aim_control_active = 0U;
        if (dm_fold_state == DM_FOLD_STARTUP_CHECK ||
            dm_fold_state == DM_FOLD_STARTUP_UNFOLDING ||
            dm_fold_state == DM_FOLD_STARTUP_REINIT_IMU)
        {
            DMControlHandleCommand("e", DM_DEBUG_ROLL_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_PITCH_ID, 0.0f, 0.0f);
            DMControlHandleCommand("e", DM_DEBUG_YAW1_ID, 0.0f, 0.0f);
            GimbalHandleCommand("e", 0.0f, 0.0f);
            camera_aim_control_active = 0U;
            dm_fold_state = DM_FOLD_STARTUP_ABORTED;
            return;
        }

        for (DM_DebugMotor_e motor = DM_DEBUG_PITCH; motor <= DM_DEBUG_YAW2; motor++)
        {
            DM_DebugState_s *running = GimbalDebugState(motor);
            if (was_camera_active == 0U && running->enabled == 0U &&
                running->returning == 0U && running->direct_target == 0U)
                continue;
            GimbalDebugBeginReturn(motor);
        }
        return;
    }

    if (strcmp(command, "aim") == 0 && dm_fold_state == DM_FOLD_DEPLOYED)
    {
        // 恢复视觉只接收随后到来的新帧，不把缓存里的旧目标重新执行。
        CameraAimFrame_s unused;
        uint32_t tick;
        CameraAimGetLatest(&unused, &camera_aim_last_sequence, &tick);
        camera_aim_inhibited = 0U;
        return;
    }

    if (strcmp(command, "z") == 0 && dm_fold_state == DM_FOLD_DEPLOYED)
    {
        camera_aim_control_active = 0U;
        for (DM_DebugMotor_e motor = DM_DEBUG_PITCH; motor <= DM_DEBUG_YAW2; motor++)
            GimbalDebugBeginReturn(motor);
        dm_fold_state = DM_FOLD_RETURNING;
        return;
    }
    if (strcmp(command, "u") == 0 && dm_fold_state == DM_FOLD_FOLDED)
    {
        dm_roll_motion_target_deg = GimbalDebugActualAngle(DM_DEBUG_ROLL);
        dm_fold_state = DM_FOLD_UNFOLDING;
        return;
    }
    if (strcmp(command, "u") == 0 && dm_fold_state == DM_FOLD_STARTUP_ABORTED)
    {
        dm_fold_state = DM_FOLD_STARTUP_CHECK;
        return;
    }

    // 折叠、折叠完成或展开过程中，屏蔽手动电机运动/正弦命令。
    if (dm_fold_state != DM_FOLD_DEPLOYED)
        return;

    if (dm_debug_motor == DM_DEBUG_ROLL)
    {
        if (strcmp(command, "akp") == 0 || strcmp(command, "skp") == 0)
            DMControlHandleCommand(command, DM_DEBUG_ROLL_ID, value1, 0.0f);
        return;
    }

    if (state == NULL)
        return;

    if (strcmp(command, "e") == 0)
    {
        camera_aim_inhibited = 1U;
        camera_aim_control_active = 0U;
        if (value1 > 0.5f)
        {
            if (state->enabled == 0U)
            {
                state->direct_target = 0U;
                /* 从当前实测角度起步，不改变软件零点或正弦运动中心。 */
                state->start_target_deg = GimbalClampFloat(GimbalDebugActualAngle(dm_debug_motor),
                                                          -GimbalDebugLimitDegForMotor(dm_debug_motor),
                                                          GimbalDebugLimitDegForMotor(dm_debug_motor));
                state->starting = 1U;
                state->phase = 0.0f;
                state->returning = 0U;
                state->enabled = 1U;
                if (dm_debug_motor == DM_DEBUG_YAW2)
                {
                    GimbalHandleCommand("p", state->start_target_deg, 0.0f);
                    GimbalHandleCommand("e", 1.0f, 0.0f);
                }
                else
                {
                    id = GimbalDebugDmId();
                    DMControlHandleCommand("p", id, state->start_target_deg, 0.0f);
                    DMControlHandleCommand("e", id, 1.0f, 0.0f);
                }
            }
        }
        else if (state->enabled != 0U || state->returning != 0U)
        {
            GimbalDebugBeginReturn(dm_debug_motor);
        }
    }
    else if (strcmp(command, "a") == 0 ||
             (strcmp(command, "p") == 0 && dm_debug_motor == DM_DEBUG_YAW2))
    {
        // a 命令直接定位到指定角度，不启动正弦轨迹；目标仍以各轴软件零点为基准。
        if (dm_debug_motor == DM_DEBUG_YAW2 && small_yaw_status.zero_valid == 0U)
            return;

        camera_aim_inhibited = 1U;
        camera_aim_control_active = 0U;
        state->enabled = 0U;
        state->returning = 0U;
        state->starting = 0U;
        state->direct_target = 1U;
        value1 = GimbalClampFloat(value1, -GimbalDebugLimitDegForMotor(dm_debug_motor),
                                  GimbalDebugLimitDegForMotor(dm_debug_motor));
        if (dm_debug_motor == DM_DEBUG_YAW2)
        {
            small_yaw_status.target_angle = value1;
            GimbalHandleCommand("e", 1.0f, 0.0f);
        }
        else
        {
            id = GimbalDebugDmId();
            DMControlHandleCommand("p", id, value1, 0.0f);
            DMControlHandleCommand("e", id, 1.0f, 0.0f);
        }
    }
    else if (strcmp(command, "amp") == 0)
        state->amplitude_deg = GimbalClampFloat(value1, 0.0f, GimbalDebugLimitDegForMotor(dm_debug_motor));
    else if (strcmp(command, "f") == 0)
        state->frequency_hz = GimbalClampFloat(value1, 0.01f, 20.0f);
    else if (strcmp(command, "akp") == 0 || strcmp(command, "skp") == 0)
    {
        if (dm_debug_motor == DM_DEBUG_YAW2)
            GimbalHandleCommand(command, value1, 0.0f);
        else
        {
            id = GimbalDebugDmId();
            DMControlHandleCommand(command, id, value1, 0.0f);
        }
    }
}

uint8_t GimbalDebugCanMoveDirectly(void)
{
    return dm_fold_state == DM_FOLD_DEPLOYED && camera_aim_control_active == 0U;
}

void GimbalDebugGetAngles(float *target_angle, float *actual_angle)
{
    const DMControlStatus_s *status;

    *target_angle = 0.0f;
    *actual_angle = 0.0f;
    if (dm_debug_motor == DM_DEBUG_YAW2)
    {
        *target_angle = small_yaw_status.target_angle;
        // 未配置零点时，用实际值通道显示原始编码器计数，供人工标定。
        *actual_angle = small_yaw_status.zero_valid != 0U ?
                        small_yaw_status.current_angle : (float)small_yaw_status.raw_ecd;
    }
    else if (dm_debug_motor == DM_DEBUG_PITCH || dm_debug_motor == DM_DEBUG_YAW1 ||
             dm_debug_motor == DM_DEBUG_ROLL)
    {
        status = DMControlGetStatus(GimbalDebugDmId());
        if (status != NULL)
        {
            *target_angle = (status->target_output_position - status->zero_position) * DM_RAD_TO_DEG;
            *actual_angle = status->output_position_relative * DM_RAD_TO_DEG;
        }
    }
}

const GimbalMotorStatus_s *GimbalGetSmallYawStatus(void)
{
    return &small_yaw_status;
}

const GimbalImuStatus_s *GimbalGetImuStatus(void)
{
    return &imu_status;
}

void GimbalDebugSetRobotCounters(uint32_t loop_count, uint32_t after_delay_count)
{
    small_yaw_status.robot_loop_count = loop_count;
    small_yaw_status.robot_after_delay_count = after_delay_count;
}
void GimbalDebugSetStage(uint32_t stage)
{
    small_yaw_status.debug_stage = stage;
}
