#ifndef __TD_H__
#define __TD_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x1;        // 跟踪值（位置）
    float x2;        // 导数估计（速度）

    float target;    // 输入目标

    float wn;        // 自然频率
    float zeta;      // 阻尼比

    float dt;        // 采样周期
} TD_HandleTypeDef;

/**
 * @brief 初始化
 */
void TD_Init(TD_HandleTypeDef *td, float wn, float zeta, float dt);

/**
 * @brief 设置目标
 */
void TD_SetTarget(TD_HandleTypeDef *td, float target);

/**
 * @brief 更新（周期调用）
 */
void TD_Update(TD_HandleTypeDef *td);

/**
 * @brief 获取位置
 */
float TD_GetX1(TD_HandleTypeDef *td);

/**
 * @brief 获取速度
 */
float TD_GetX2(TD_HandleTypeDef *td);

#ifdef __cplusplus
}
#endif

#endif