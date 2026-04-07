#include "Td.h"

/**
 * @brief 初始化
 */
void TD_Init(TD_HandleTypeDef *td, float wn, float zeta, float dt)
{
    td->x1 = 0.0f;
    td->x2 = 0.0f;
    td->target = 0.0f;

    td->wn = wn;
    td->zeta = zeta;
    td->dt = dt;
}

/**
 * @brief 设置目标
 */
void TD_SetTarget(TD_HandleTypeDef *td, float target)
{
    td->target = target;
}

/**
 * @brief 更新（半隐式Euler，稳定性更好）
 */
void TD_Update(TD_HandleTypeDef *td)
{
    float e = td->x1 - td->target;

    // 系统参数
    float wn = td->wn;
    float zeta = td->zeta;
    float dt = td->dt;

    // x2 更新（先更新速度）
    float x2_dot = -2.0f * zeta * wn * td->x2
                   - wn * wn * e;

    td->x2 += x2_dot * dt;

    // x1 更新（再用新速度更新）
    td->x1 += td->x2 * dt;
}

/**
 * @brief 获取位置
 */
float TD_GetX1(TD_HandleTypeDef *td)
{
    return td->x1;
}

/**
 * @brief 获取速度
 */
float TD_GetX2(TD_HandleTypeDef *td)
{
    return td->x2;
}