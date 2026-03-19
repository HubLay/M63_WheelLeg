#include "daemon.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx = 0; // 用于记录当前的daemon instance数量,配合回调使用

Daemon_Init_Config_s Get_DaemonInitConfig_s(uint8_t Reload_Count, void *Param, offline_callback Callback_Function)
{
  Daemon_Init_Config_s Daemon_Init_Config;

  Daemon_Init_Config.Param = Param;
  Daemon_Init_Config.Reload_Count = Reload_Count;
  Daemon_Init_Config.Callback_Function = Callback_Function;

  return Daemon_Init_Config;
}

DaemonInstance *DaemonRegister(Daemon_Init_Config_s config)
{
    DaemonInstance *instance = (DaemonInstance *)user_malloc(sizeof(DaemonInstance));

    if(instance == NULL){
        return NULL;
    }

    memset(instance, 0, sizeof(DaemonInstance)); 

    if(idx > DAEMON_MX_CNT){
        return NULL;                //设备溢出
    }

    instance->Count = 0;                                                //上电默认离线
    instance->Device_Status = Device_Offline;
    instance->Param = config.Param;
    instance->Reload_Count = config.Reload_Count;
    instance->Callback_Function = config.Callback_Function;
    DWT_GetDeltaT(&instance->Dwt.cnt_last);

    daemon_instances[idx] = instance;

    idx++;

    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    instance->Count = instance->Reload_Count;
    instance->Dwt.dt = DWT_GetDeltaT(&instance->Dwt.cnt_last);
}

Enum_Device_Status Get_Device_Status(DaemonInstance *instance)
{
  return (instance->Device_Status);
}

void DaemonTask()
{
    DaemonInstance *instance; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; i++)
    {
        instance = daemon_instances[i];

        if(instance->Count > 0){
            instance->Count --;
            instance->Device_Status = Device_Online;
        }
        else{
            instance->Callback_Function(instance->Param);
            instance->Device_Status = Device_Offline;
        }
    }
}
