#ifndef MONITOR_H
#define MONITOR_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "string.h"

#include "dvc_dwt.h"
#include "FreeRTOS.h"

#ifdef INC_FREERTOS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif


#define DAEMON_MX_CNT 64

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *Param);

typedef enum
{
    Device_Offline = 0,
    Device_Online,
}Enum_Device_Status;

/* daemon结构体定义 */
typedef struct daemon_ins
{
    uint8_t Count, Reload_Count;               //模块离线检测周期   10ms * Count
    offline_callback Callback_Function;

    struct_DwtTime Dwt;
    Enum_Device_Status Device_Status;

    void *Param;                                //可配置的传入参数    
} DaemonInstance;

typedef struct
{
    uint8_t Reload_Count;               //模块离线检测周期   10ms * Reload_Count
    offline_callback Callback_Function;

    void *Param;                                //可配置的传入参数    
} Daemon_Init_Config_s;

Daemon_Init_Config_s Get_DaemonInitConfig_s(uint8_t Reload_Count, void *Param, offline_callback Callback_Function);

/**
 * @brief 注册一个daemon实例
 *
 * @param config 初始化配置
 * @return DaemonInstance* 返回实例指针
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s config);

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance daemon实例指针
 */
void DaemonReload(DaemonInstance *instance);

/**
 * @brief 确认模块是否离线
 *
 * @param instance daemon实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
Enum_Device_Status Get_Device_Status(DaemonInstance *instance);

/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask();

#ifdef __cplusplus
}
#endif

#endif