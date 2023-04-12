#ifndef __WOLF_INFANTRY_H
#define __WOLF_INFANTRY_H

/* Macro Definitions ---------------------------------------------------------*/
/* 以此纪念野狼步兵机器人 */

#define ROBOT_ID 3 // - 当前机器人 ID

#define INFANTRY_2019 1 // - 2019步兵
#define INFANTRY_2020 2 // - 2020步兵
#define INFANTRY_2021 3 // - 2021步兵
#define INFANTRY_2021_ZJ 4 // - 2021焯钧步兵
#define INFANTRY_2022_Test 5 // - 2022舵轮步兵

#define Device_BoardType 2

#define Device_BoardType_Wolf 0 // 彬哥板
#define Device_BoardType_old  1 // RM 旧板
#define Device_BoardType_A    2 // RM A板
#define Device_BoardType_C    3 // RM C板

//以下宏用于测试时使用
#define USE_DeviceMonitor 1 //启用离线、错误报警
#define USE_Buzzer          //启用蜂鸣器

#define USE_GimbalCtrl      // 使能云台控制
#define USE_GimbalIMU       // 启用云台陀螺仪

#endif