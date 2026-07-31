#ifndef INVERTER_SVPWM_H
#define INVERTER_SVPWM_H

#include <stdint.h>

#include "inverter_dpwm.h"

/**
 * @brief 连续SVPWM输出和调试状态
 *
 * @note Timer A/B/C的初始化、计数器、输出使能和紧急关断仍由
 *       inverter_dpwm模块中的共用HRTIM后端负责。本模块只在被选择时
 *       计算连续SVPWM并写入同一组CMP1预装载寄存器。
 */
typedef struct
{
    float requested_m_a;      /**< 控制器请求的A相归一化调制量。 */
    float requested_m_b;      /**< 控制器请求的B相归一化调制量。 */
    float requested_m_c;      /**< 控制器请求的C相归一化调制量。 */
    float modulation_scale;   /**< 过调制时对三相指令统一施加的缩放系数。 */
    float zero_sequence;      /**< -(max+min)/2连续零序分量。 */
    float applied_m_a;        /**< 注入零序后的A相调制量。 */
    float applied_m_b;        /**< 注入零序后的B相调制量。 */
    float applied_m_c;        /**< 注入零序后的C相调制量。 */
    float duty_a_next;        /**< 下一周期Timer A上管占空比。 */
    float duty_b_next;        /**< 下一周期Timer B上管占空比。 */
    float duty_c_next;        /**< 下一周期Timer C上管占空比。 */
    uint32_t period_counts;   /**< Timer A/B/C中心对齐计数峰值。 */
    uint32_t compare_a_next;  /**< 下一周期Timer A CMP1值。 */
    uint32_t compare_b_next;  /**< 下一周期Timer B CMP1值。 */
    uint32_t compare_c_next;  /**< 下一周期Timer C CMP1值。 */
    uint8_t sector;           /**< 按三相调制量大小关系得到的扇区1至6。 */
    uint8_t initialized;
} Inverter_SVPWM_StateTypeDef;

/** SVPWM运行状态，可在调试器中观察。 */
extern volatile Inverter_SVPWM_StateTypeDef inverter_svpwm_state;

/**
 * @brief          初始化SVPWM状态并核对共用Timer A/B/C周期
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR DPWM/HRTIM后端未初始化或周期无效
 *
 * @note           必须在Inverter_DPWM_Init()成功后调用。
 */
HAL_StatusTypeDef Inverter_SVPWM_Init(void);

/**
 * @brief          计算连续SVPWM零序并预装载下一周期三相CMP1
 * @param[in]      m_a A相归一化相电压指令，定义为2乘Va_cmd除以Vdc
 * @param[in]      m_b B相归一化相电压指令，定义为2乘Vb_cmd除以Vdc
 * @param[in]      m_c C相归一化相电压指令，定义为2乘Vc_cmd除以Vdc
 *
 * @note           使用等效的min-max零序注入：m0=-(max+min)/2。
 *                 过调制时三相统一缩放，不分别削顶。
 */
void Inverter_SVPWM_Update(float m_a,
                           float m_b,
                           float m_c);

/**
 * @brief 清除SVPWM调试状态，保留初始化标志和HRTIM周期
 */
void Inverter_SVPWM_Reset(void);

#endif /* INVERTER_SVPWM_H */
