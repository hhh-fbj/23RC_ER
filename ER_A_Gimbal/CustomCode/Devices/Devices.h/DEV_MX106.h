#ifndef __DEV_MX106_H
#define __DEV_MX106_H
/*** 指令包 ***/
//ID范围：0 ~ 252，253个数字；广播ID254
//数据包长度：长度=参数数量+ 3
/* 指令
0x01 Ping 指令检查数据包是否已到达具有与包 ID 相同的 ID 的
设备
0x02 Read 从设备读取数据指令。
0x03 Write 在设备上写入数据指令。
0x04 Reg Write 将指令包注册到备用状态的指令，稍后通过 Action 指
令执行数据包。
0x05 Action 使用 Red Write 执行预先注册的指令包。
0x06 Factory Reset 指令将控制表重置为初始工厂默认设置。
0x08 Reboot 指令重启设备。
0x10 Clear 指令重置特定信息。
0x55 Status(Return) 指令包的返回指令。
0x82 Sync Read 对于多个设备，指令在同一地址上同时读取相同长度
的数据。
0x83 Sync Write 对于多个设备，指令在同一地址上同时写入相同长度
的数据。
0x92 Bulk Read 于多个设备，指令一次在不同长度的地址上读取数据。
0x93 Bulk Write 于多个设备，指令一次在不同长度的地址上写入数据。
*/


/*** 状态包 ***/
//状态包的指令被指定为 0x55 (状态)
/*错误
0x01 结果错误 l 发送指令包失败。
0x02 指令错误 未定义的指令已被使用。没有使用 Reg Write 操作。
0x03 CRC 错误 发送数据包的 CRC 不匹配。
0x04 数据范围错误 写入相应地址的数据在最小/最大值范围之外。
0x05 数据长度错误 尝试写入比对应地址的数据长度短的数据。(例如:当您尝试仅使用已定义为 4 字节的项目的 2 字
节)
0x06 数据限值错误 在相应地址中写入的数据超出极限值。
0x07 访问错误 试图在只读或未定义的地址中写入值。尝试读取仅写入或未定义的地址中的值。试图在扭矩启用（ROM 锁定）状态下在 ROM 域中写
入值
*/

#include <stdint.h>
#include "ALGO_Kalman.h"
#include "APP_Devices.h"
#include "stm32f4xx_hal.h"

#define MXDATASIZE 32

/*** 读写常用内存地址 ***/
/*

*/
class MX106_classdef
{
private:
    uint8_t ID;
    unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
    uint8_t State,Error;
    uint8_t send_data[16];
public:
		MX106_classdef(uint8_t id): ID(id){};
		uint8_t MX106_Data[MXDATASIZE];
    uint8_t Torque_Flag;
		uint32_t Posotion;
		uint8_t Tyg;

    void Ping(void);
    void Torque(uint8_t NON);
    void Position_Control(uint32_t position);
		void ExtendPosition_Control(uint32_t position);
		void ReadPosition(void);
    void Read(void);
    void Write(void);
    void Receive(uint8_t *data);
};

#endif