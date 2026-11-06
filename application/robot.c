#include "bsp_init.h"
#include "bsp_can.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "vofa.h"
#include "camera_aim.h"
#include "usart.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(DM_GIMBAL_TEST)
    VOFA_Init();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#if defined(DM_GIMBAL_TEST)
    GimbalInit();
    CameraAimInitUsb();
#else
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(CAN_ID_SCAN_TEST)
    return;
#else
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#if defined(DM_GIMBAL_TEST)
    GimbalTask();
#else
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif
#endif
}
