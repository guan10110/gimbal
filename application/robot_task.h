/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "robot_def.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "vofa.h"
#include "gimbal.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId vofaTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartVOFATASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始�? *
 */
void OSTaskInit()
{
#if defined(CAN_ID_SCAN_TEST)
    osThreadDef(vofatask, StartVOFATASK, osPriorityNormal, 0, 512);
    vofaTaskHandle = osThreadCreate(osThread(vofatask), NULL);
    return;
#endif

    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保�?khz的频率执�?    // // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

#if !defined(CAN_ID_SCAN_TEST) && defined(DM_GIMBAL_TEST)
    osThreadDef(vofatask, StartVOFATASK, osPriorityBelowNormal, 0, 512);
    vofaTaskHandle = osThreadCreate(osThread(vofatask), NULL);
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
#endif

    HTMotorControlInit(); // 没有注册HT电机则不会执�?}
}

#if defined(DM_GIMBAL_TEST)
__attribute__((noreturn)) void StartVOFATASK(void const *argument)
{
    LOGINFO("[freeRTOS] VOFA Task Start");
    for (;;)
    {
        VOFA_ProcessCommand();
        VOFA_TestSend();
        osDelay(10);
    }
}
#endif

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
#if !defined(DM_GIMBAL_TEST)
        VisionSend(); // 解算完成后发送视觉数�?但是当前的实现不太优�?后续若添加硬件触发需要重新考虑结构的组�?
#endif
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    static uint32_t robot_loop_count;
    static uint32_t robot_after_delay_count;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    for (;;)
    {
        robot_loop_count++;
        GimbalDebugSetRobotCounters(robot_loop_count, robot_after_delay_count);
        GimbalDebugSetStage(201U);
        robot_start = DWT_GetTimeline_ms();
        GimbalDebugSetStage(202U);
        RobotTask();
        GimbalDebugSetStage(203U);
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        GimbalDebugSetStage(204U);
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        GimbalDebugSetStage(205U);
        osDelay(5);
        robot_after_delay_count++;
        GimbalDebugSetRobotCounters(robot_loop_count, robot_after_delay_count);
        GimbalDebugSetStage(206U);
    }
}
__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    for (;;)
    {
        UITask();
        osDelay(1);
    }
}
