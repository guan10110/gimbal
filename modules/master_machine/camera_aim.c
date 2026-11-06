#include "camera_aim.h"

#include "string.h"
#include "math.h"

#define CAMERA_AIM_FRAME_SIZE 62U // 固定帧长：数据 60 字节，末尾 2 字节 CRC
#define CAMERA_AIM_RX_BUFFER_SIZE 128U
#define CAMERA_AIM_HEAD_LOW 0xA5U
#define CAMERA_AIM_HEAD_HIGH 0x00U

static USARTInstance *camera_aim_usart;
static uint8_t camera_frame[CAMERA_AIM_FRAME_SIZE];
static uint8_t camera_frame_length;
static CameraAimFrame_s latest_frame;
static uint32_t latest_sequence;
static uint32_t latest_receive_tick;

static void CameraAimFeedBytes(const uint8_t *data, uint16_t length);

static uint16_t CameraAimReadU16LE(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t CameraAimReadU32LE(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t CameraAimReadI32LE(const uint8_t *data)
{
    uint32_t raw = CameraAimReadU32LE(data);
    int32_t value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static float CameraAimReadFloatLE(const uint8_t *data)
{
    uint32_t raw = CameraAimReadU32LE(data);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint16_t CameraAimCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static uint8_t CameraAimDecodeFrame(const uint8_t *data, CameraAimFrame_s *frame)
{
    // CRC 按小端读取，校验范围是帧头开始的前 60 字节。
    uint16_t expected_crc = CameraAimReadU16LE(&data[60]);
    uint16_t actual_crc = CameraAimCrc16(data, 60U);

    if (expected_crc != actual_crc || data[0] != CAMERA_AIM_HEAD_LOW ||
        data[1] != CAMERA_AIM_HEAD_HIGH || data[2] > 3U || data[3] > 1U ||
        data[36] > 1U || data[37] > 1U)
        return 0U;

    frame->mode = data[2];
    frame->foe_color = data[3];
    frame->yaw_deg = CameraAimReadFloatLE(&data[4]);
    frame->pitch_deg = CameraAimReadFloatLE(&data[8]);
    frame->bullet_speed = CameraAimReadFloatLE(&data[12]);
    frame->big_yaw_deg = CameraAimReadFloatLE(&data[16]);
    frame->current_robot_quantity = CameraAimReadI32LE(&data[20]);
    frame->blood_warn_state = CameraAimReadI32LE(&data[24]);
    frame->game_process = CameraAimReadI32LE(&data[28]);
    frame->current_state_left_time = CameraAimReadFloatLE(&data[32]);
    frame->is_success_ourpreempt = data[36];
    frame->is_success_enemypreempt = data[37];
    for (uint8_t i = 0U; i < 4U; i++)
        frame->quaternion[i] = CameraAimReadFloatLE(&data[38U + 4U * i]);
    frame->bullet_count = CameraAimReadU16LE(&data[54]);
    frame->time = CameraAimReadFloatLE(&data[56]);

    if (!isfinite(frame->yaw_deg) || !isfinite(frame->pitch_deg) ||
        !isfinite(frame->bullet_speed) || !isfinite(frame->big_yaw_deg) ||
        !isfinite(frame->current_state_left_time) || !isfinite(frame->time))
        return 0U;
    for (uint8_t i = 0U; i < 4U; i++)
        if (!isfinite(frame->quaternion[i]))
            return 0U;

    return 1U;
}

static void CameraAimResyncAfterBadFrame(void)
{
    uint8_t start;

    for (start = 1U; start < CAMERA_AIM_FRAME_SIZE - 1U; start++)
    {
        if (camera_frame[start] == CAMERA_AIM_HEAD_LOW &&
            camera_frame[start + 1U] == CAMERA_AIM_HEAD_HIGH)
        {
            camera_frame_length = (uint8_t)(CAMERA_AIM_FRAME_SIZE - start);
            memmove(camera_frame, &camera_frame[start], camera_frame_length);
            return;
        }
    }

    camera_frame_length = camera_frame[CAMERA_AIM_FRAME_SIZE - 1U] == CAMERA_AIM_HEAD_LOW ? 1U : 0U;
    if (camera_frame_length != 0U)
        camera_frame[0] = CAMERA_AIM_HEAD_LOW;
}

static void CameraAimConsumeByte(uint8_t byte)
{
    CameraAimFrame_s decoded;

    if (camera_frame_length == 0U)
    {
        if (byte == CAMERA_AIM_HEAD_LOW)
        {
            camera_frame[0] = byte;
            camera_frame_length = 1U;
        }
        return;
    }

    if (camera_frame_length == 1U)
    {
        if (byte == CAMERA_AIM_HEAD_HIGH)
        {
            camera_frame[1] = byte;
            camera_frame_length = 2U;
        }
        else if (byte == CAMERA_AIM_HEAD_LOW)
        {
            camera_frame[0] = byte;
        }
        else
            camera_frame_length = 0U;
        return;
    }

    camera_frame[camera_frame_length++] = byte;
    if (camera_frame_length == CAMERA_AIM_FRAME_SIZE)
    {
        if (CameraAimDecodeFrame(camera_frame, &decoded) != 0U)
        {
            latest_frame = decoded;
            latest_receive_tick = HAL_GetTick();
            latest_sequence++;
            camera_frame_length = 0U;
        }
        else
            CameraAimResyncAfterBadFrame();
    }
}

static void CameraAimReceiveCallback(uint16_t receive_size)
{
    CameraAimFeedBytes(camera_aim_usart->recv_buff, receive_size);
}

static void CameraAimFeedBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL)
        return;

    for (uint16_t i = 0U; i < length; i++)
        CameraAimConsumeByte(data[i]);
}

void CameraAimInit(UART_HandleTypeDef *uart)
{
    USART_Init_Config_s config = {0};

    config.recv_buff_size = CAMERA_AIM_RX_BUFFER_SIZE;
    config.usart_handle = uart;
    camera_aim_usart = USARTRegisterWithSizeCallback(&config, CameraAimReceiveCallback);
}

void CameraAimInitUsb(void)
{
    camera_aim_usart = NULL;
    camera_frame_length = 0U;
    latest_sequence = 0U;
    latest_receive_tick = 0U;
}

void CameraAimUsbReceive(const uint8_t *data, uint16_t length)
{
    CameraAimFeedBytes(data, length);
}

uint8_t CameraAimGetLatest(CameraAimFrame_s *frame, uint32_t *sequence,
                           uint32_t *receive_tick)
{
    uint32_t primask;

    if (frame == NULL || sequence == NULL || receive_tick == NULL)
        return 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    *frame = latest_frame;
    *sequence = latest_sequence;
    *receive_tick = latest_receive_tick;
    __set_PRIMASK(primask);

    return *sequence != 0U;
}
