#ifndef INVERTER_MODULATION_H
#define INVERTER_MODULATION_H

#include "main.h"

/**
 * @brief 三相逆变调制方式
 */
typedef enum
{
    INVERTER_MODULATION_MODE_DPWM1 = 0,
    INVERTER_MODULATION_MODE_SVPWM = 1
} Inverter_ModulationModeTypeDef;

/**
 * 编译后的默认调制方式。
 *
 * @note 保持DPWM1可兼容当前工程；改成SVPWM即可让固件默认使用连续SVPWM。
 */
#ifndef INVERTER_MODULATION_MODE_DEFAULT
#define INVERTER_MODULATION_MODE_DEFAULT \
INVERTER_MODULATION_MODE_SVPWM
#endif

/** 当前选择的调制方式，可在调试器中观察。 */
extern volatile Inverter_ModulationModeTypeDef
    inverter_modulation_mode;

/**
 * @brief          在停机状态选择DPWM1或连续SVPWM
 * @param[in]      modulation_mode 目标调制方式
 * @retval         HAL_OK 已选择目标调制方式
 * @retval         HAL_BUSY 逆变输出仍处于使能状态
 * @retval         HAL_ERROR 未初始化或调制方式参数无效
 *
 * @note           切换顺序：RequestStop()，等待enabled清零，调用本函数，
 *                 再RequestStart()。运行中禁止切换，避免占空比突变。
 */
HAL_StatusTypeDef Inverter_Control_SetModulationMode(
    Inverter_ModulationModeTypeDef modulation_mode);

#endif /* INVERTER_MODULATION_H */
