#include "vofa.h"

#include "dm_control.h"
#include "gimbal.h"
#include "robot_def.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_RX_LINE_SIZE 48U
#define DM_RAD_TO_DEG 57.2957795f
static uint8_t vofa_rx_byte;
static char vofa_rx_line[VOFA_RX_LINE_SIZE];
static char vofa_pending_line[VOFA_RX_LINE_SIZE];
static uint8_t vofa_tx_buffer[12U];
static volatile uint8_t vofa_rx_length;
static volatile uint8_t vofa_command_ready;

static uint8_t VOFA_Split(char *line, char **parts, uint8_t max_parts)
{
    uint8_t count = 0U;
    char *cursor = line;

    while (*cursor != '\0' && count < max_parts)
    {
        parts[count++] = cursor;
        while (*cursor != '\0' && *cursor != ',')
            cursor++;
        if (*cursor == ',')
            *cursor++ = '\0';
    }
    return count;
}

void VOFA_Init(void)
{
    HAL_UART_Receive_IT(&huart7, &vofa_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart7)
        return;

    if (vofa_rx_byte == ';' || vofa_rx_byte == '\n')
    {
        if (vofa_rx_length > 0U && vofa_command_ready == 0U)
        {
            vofa_rx_line[vofa_rx_length] = '\0';
            memcpy(vofa_pending_line, vofa_rx_line, vofa_rx_length + 1U);
            vofa_command_ready = 1U;
        }
        vofa_rx_length = 0U;
    }
    else if (vofa_rx_byte != '\r')
    {
        if (vofa_rx_length < (VOFA_RX_LINE_SIZE - 1U))
            vofa_rx_line[vofa_rx_length++] = (char)vofa_rx_byte;
        else
            vofa_rx_length = 0U;
    }

    HAL_UART_Receive_IT(&huart7, &vofa_rx_byte, 1U);
}

void VOFA_ProcessCommand(void)
{
    char command[VOFA_RX_LINE_SIZE];
    char *parts[4];
    uint8_t count;
    uint8_t id;
    float value1 = 0.0f;
    float value2 = 0.0f;

    if (vofa_command_ready == 0U)
        return;

    memcpy(command, vofa_pending_line, sizeof(command));
    vofa_command_ready = 0U;
    count = VOFA_Split(command, parts, 4U);
    if (count == 0U)
        return;

    if (strcmp(parts[0], "pitch") == 0 || strcmp(parts[0], "yaw") == 0 ||
        strcmp(parts[0], "yaw1") == 0 || strcmp(parts[0], "yaw2") == 0 ||
        strcmp(parts[0], "roll") == 0)
    {
        GimbalDebugSelect(parts[0]);
        return;
    }

    if (count == 1U)
    {
        if (strcmp(parts[0], "e") == 0 || strcmp(parts[0], "s") == 0 ||
            strcmp(parts[0], "z") == 0 || strcmp(parts[0], "u") == 0 ||
            strcmp(parts[0], "aim") == 0)
            GimbalDebugHandleCommand(parts[0], strcmp(parts[0], "e") == 0 ? 1.0f : 0.0f);
        return;
    }

#if defined(DM_GIMBAL_TEST)
    // p,<角度>; 是选中 yaw2 后的小 yaw 定位命令，不带电机 ID。
    if (strcmp(parts[0], "p") == 0 && count == 2U)
    {
        GimbalDebugHandleCommand("p", strtof(parts[1], NULL));
        return;
    }
#endif

#if defined(DM_GIMBAL_TEST)
    /* 当前轴调参不带 ID，第二项就是数值，如 akp,96; skp,0.6;。 */
    if (strcmp(parts[0], "a") == 0 || strcmp(parts[0], "amp") == 0 ||
        strcmp(parts[0], "f") == 0 ||
        strcmp(parts[0], "akp") == 0 || strcmp(parts[0], "skp") == 0 ||
        strcmp(parts[0], "e") == 0 || strcmp(parts[0], "s") == 0)
    {
        GimbalDebugHandleCommand(parts[0], strtof(parts[1], NULL));
        return;
    }
#endif

    id = (uint8_t)strtoul(parts[1], NULL, 0);
    if (count > 2U)
        value1 = strtof(parts[2], NULL);
    if (count > 3U)
        value2 = strtof(parts[3], NULL);

#if defined(DM_GIMBAL_TEST)
    /* 带 ID 的旧命令只发给达妙，防止 ID 3 与 DJI 小 yaw 混淆。
     * 同样遵守折叠和视觉控制互锁，不能绕过状态机直接移动电机。 */
    if (GimbalDebugCanMoveDirectly() == 0U)
        return;
    DMControlHandleCommand(parts[0], id, value1, value2);
#else
    if (id == 3U)
        GimbalHandleCommand(parts[0], value1, value2);
    else
        DMControlHandleCommand(parts[0], id, value1, value2);
#endif
}

void VOFA_TestSend(void)
{
    const uint8_t frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    float target_angle_deg;
    float actual_angle_deg;

    GimbalDebugGetAngles(&target_angle_deg, &actual_angle_deg);

    memcpy(vofa_tx_buffer, &target_angle_deg, sizeof(float));
    memcpy(vofa_tx_buffer + sizeof(float), &actual_angle_deg, sizeof(float));
    memcpy(vofa_tx_buffer + 2U * sizeof(float), frame_tail, sizeof(frame_tail));

    HAL_UART_Transmit(&huart7, vofa_tx_buffer, 12U, 100U);
}
