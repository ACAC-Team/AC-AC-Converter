#ifndef INVERTER_DPWM_H
#define INVERTER_DPWM_H

#include <stdint.h>
#include "hrtim.h"

/* ==================== HRTIM与三相桥映射 ==================== */

/** Timer A互补输出驱动功率管1和功率管4。 */
#define INVERTER_DPWM_PHASE_A_OUTPUTS \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2)

/** Timer B互补输出驱动功率管3和功率管6。 */
#define INVERTER_DPWM_PHASE_B_OUTPUTS \
    (HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)

/** Timer C互补输出驱动功率管5和功率管2。 */
#define INVERTER_DPWM_PHASE_C_OUTPUTS \
    (HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)

/** 三相桥六路HRTIM功率输出位掩码。 */
#define INVERTER_DPWM_ALL_OUTPUTS \
    (INVERTER_DPWM_PHASE_A_OUTPUTS | \
     INVERTER_DPWM_PHASE_B_OUTPUTS | \
     INVERTER_DPWM_PHASE_C_OUTPUTS)

/** 三相逆变使用的HRTIM计数器。Master由PFC应用先行启动。 */
#define INVERTER_DPWM_COUNTERS \
    (HRTIM_TIMERID_TIMER_A | \
     HRTIM_TIMERID_TIMER_B | \
     HRTIM_TIMERID_TIMER_C)

/* ==================== DPWM1边界参数 ==================== */

/**
 * 本工程固定使用DPWM1：按max+min的符号交替选择上母线或下母线钳位。
 *
 * @note 若高侧驱动采用自举供电，应先确认连续钳位时间满足驱动电源要求，
 *       再使能功率输出。
 */

/**
 * 非钳位相CMP1与0、PER边界之间保留的最小计数。
 *
 * @note 真正被DPWM选中的钳位相允许写入0或PER；其余开关相保留该边界，
 *       避免产生过窄脉冲。
 */
#define INVERTER_DPWM_COMPARE_GUARD_COUNTS 24U

/**
 * @brief 当前DPWM钳位状态
 */
typedef enum
{
    INVERTER_DPWM_CLAMP_NONE = 0,
    INVERTER_DPWM_CLAMP_A_LOW,
    INVERTER_DPWM_CLAMP_A_HIGH,
    INVERTER_DPWM_CLAMP_B_LOW,
    INVERTER_DPWM_CLAMP_B_HIGH,
    INVERTER_DPWM_CLAMP_C_LOW,
    INVERTER_DPWM_CLAMP_C_HIGH
} Inverter_DPWM_ClampTypeDef;

/**
 * @brief DPWM输出运行和调试状态
 */
typedef struct
{
    float requested_m_a;      /**< 控制器请求的A相归一化调制量。 */
    float requested_m_b;      /**< 控制器请求的B相归一化调制量。 */
    float requested_m_c;      /**< 控制器请求的C相归一化调制量。 */
    float modulation_scale;   /**< 过调制时对三相指令统一施加的缩放系数。 */
    float zero_sequence;      /**< 本周期注入的公共零序分量。 */
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
    Inverter_DPWM_ClampTypeDef clamp;
    uint8_t initialized;
    uint8_t counters_running;
    uint8_t outputs_enabled;
} Inverter_DPWM_StateTypeDef;

/** DPWM运行状态，可在调试器中观察。 */
extern volatile Inverter_DPWM_StateTypeDef inverter_dpwm_state;

/**
 * @brief          初始化Timer A/B/C的安全比较值和DPWM运行状态
 * @retval         HAL_OK 初始化成功，六路功率输出保持关闭
 * @retval         HAL_ERROR Timer A/B/C周期无效或不一致
 * @retval         其他HAL状态 HRTIM操作失败
 *
 * @note           静态HRTIM配置完全沿用threenibian.ioc生成结果。
 */
HAL_StatusTypeDef Inverter_DPWM_Init(void);

/**
 * @brief          启动Timer A/B/C计数器，但不使能六路功率输出
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           必须在HRTIM Master采样时基启动之后调用。
 */
HAL_StatusTypeDef Inverter_DPWM_StartCounters(void);

/**
 * @brief          以三相50%同相安全状态使能六路互补输出
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_DPWM_Enable(void);

/**
 * @brief          计算DPWM1零序并预装载下一周期三相CMP1
 * @param[in]      m_a A相归一化相电压指令
 * @param[in]      m_b B相归一化相电压指令
 * @param[in]      m_c C相归一化相电压指令
 *
 * @note           函数会在需要时统一缩放三相指令，不会分别削顶。
 */
void Inverter_DPWM_Update(float m_a,
                          float m_b,
                          float m_c);

/**
 * @brief          通过ODISR立即关闭Timer A/B/C六路功率输出
 */
void Inverter_DPWM_Disable(void);

#endif /* INVERTER_DPWM_H */
