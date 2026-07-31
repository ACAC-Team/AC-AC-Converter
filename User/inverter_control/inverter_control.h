#ifndef INVERTER_CONTROL_H
#define INVERTER_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "inverter_dpwm.h"

/* ==================== 基本控制参数 ==================== */

/** ADC1全传输回调和三相逆变控制更新频率，单位为Hz。 */
#define INVERTER_CONTROL_FREQ_HZ              20000.0f

/** 仿真模型当前默认的三相输出基波频率，单位为Hz。 */
#define INVERTER_OUTPUT_FREQ_DEFAULT_HZ       44.963f

/** 题目要求的两个可选输出频率，单位为Hz。 */
#define INVERTER_OUTPUT_FREQ_LOW_HZ           30.0f
#define INVERTER_OUTPUT_FREQ_HIGH_HZ          60.0f

/** 题目要求的输出线电压有效值，单位为V RMS。 */
#define INVERTER_LINE_VOLTAGE_TARGET_RMS_V    32.0f

/** 输出线电压参考软启动斜率，单位为V RMS/s。 */
#define INVERTER_LINE_VOLTAGE_SLEW_V_PER_S    3.0//16.0f

/** 允许三相逆变闭环投入的最低直流母线电压。 */
#define INVERTER_MIN_DC_BUS_V                 45.0f//45.0f

/** 2π常数。 */
#define INVERTER_TWO_PI                       6.28318530718f

/** 2π/3常数。 */
#define INVERTER_TWO_PI_OVER_THREE            2.09439510239f

/** 线电压RMS换算到相电压峰值的系数sqrt(2/3)。 */
#define INVERTER_LINE_RMS_TO_PHASE_PEAK       0.81649658093f

/**
 * 上电后是否自动提交三相逆变启动请求。
 *
 * @note 默认关闭，调试器将inverter_start_request写1后才会使能六路PWM。
 */
#define INVERTER_CONTROL_AUTO_START           0U


/* ==================== 电压PR外环参数 ==================== */

/** 电压PR外环比例增益，输出单位为A。 */
#define INVERTER_VOLTAGE_PR_KP                0.050f

/** 电压PR外环谐振增益，输出单位为A。 */
#define INVERTER_VOLTAGE_PR_KR                10.0f

/** 电压PR外环谐振带宽，单位为rad/s。 */
#define INVERTER_VOLTAGE_PR_WC_RAD_S          1.0f

/**
 * 电压外环允许的相电流参考绝对值，单位为A。
 *
 * @note 低于ADC2模拟看门狗的3A硬件限值。
 */
#define INVERTER_CURRENT_REFERENCE_LIMIT_A    5.0f

/* ==================== 电流PR内环参数 ==================== */

/** 电流PR内环比例增益，输出单位为V。 */
#define INVERTER_CURRENT_PR_KP                2.0f

/** 电流PR内环谐振增益，输出单位为V。 */
#define INVERTER_CURRENT_PR_KR                80.0f

/** 电流PR内环谐振带宽，单位为rad/s。 */
#define INVERTER_CURRENT_PR_WC_RAD_S          5.0f

/** 电流PR内环电压修正量绝对值上限，单位为V。 */
#define INVERTER_VOLTAGE_CORRECTION_LIMIT_V   80.0f

/**
 * @brief 双线性变换离散准PR控制器
 */
typedef struct
{
    float kp;
    float b0;
    float b2;
    float a1;
    float a2;
    float error_z1;
    float error_z2;
    float resonant_z1;
    float resonant_z2;
    float output_limit;
} Inverter_PR_ControllerTypeDef;

/**
 * @brief 三相逆变双PR闭环运行和调试状态
 */
typedef struct
{
    Inverter_PR_ControllerTypeDef voltage_pr_a;
    Inverter_PR_ControllerTypeDef voltage_pr_c;
    Inverter_PR_ControllerTypeDef current_pr_a;
    Inverter_PR_ControllerTypeDef current_pr_c;

    float phase_rad;
    float output_frequency_hz;
    float line_voltage_reference_rms_v;
    float dc_bus_v;

    float u_ab_v;
    float u_bc_v;
    float i_a_a;
    float i_b_a;
    float i_c_a;

    float v_a_v;
    float v_b_v;
    float v_c_v;

    float v_a_reference_v;
    float v_b_reference_v;
    float v_c_reference_v;

    float i_a_reference_a;
    float i_b_reference_a;
    float i_c_reference_a;

    float v_a_correction_v;
    float v_c_correction_v;
    float v_a_command_v;
    float v_b_command_v;
    float v_c_command_v;

    float m_a;
    float m_b;
    float m_c;

    uint32_t update_count;
    uint8_t initialized;
    uint8_t enabled;
} Inverter_Control_StateTypeDef;

/** 三相逆变控制运行状态，可在调试器中观察。 */
extern volatile Inverter_Control_StateTypeDef
    inverter_control_state;

/** 写1请求启动三相逆变，服务函数处理后自动清零。 */
extern volatile uint8_t inverter_start_request;

/** 写1请求停止三相逆变，服务函数处理后自动清零。 */
extern volatile uint8_t inverter_stop_request;

/** 三相逆变控制最近一次HAL执行状态。 */
extern volatile HAL_StatusTypeDef inverter_control_last_status;

/**
 * @brief          初始化双PR、DPWM和Timer A/B/C计数器
 * @retval         HAL_OK 初始化成功，六路功率输出保持关闭
 * @retval         其他HAL状态 DPWM初始化或计数器启动失败
 *
 * @note           必须在PFC_App_Init()启动HRTIM Master之后调用。
 */
HAL_StatusTypeDef Inverter_Control_Init(void);

/**
 * @brief          在停机状态选择题目要求的30Hz或60Hz输出
 * @param[in]      output_frequency_hz 仅接受30Hz或60Hz
 * @retval         HAL_OK 频率和四个PR谐振系数已更新
 * @retval         HAL_BUSY 逆变输出仍处于使能状态
 * @retval         HAL_ERROR 未初始化或频率参数无效
 *
 * @note           切换顺序：RequestStop()，等待enabled清零，调用本函数，
 *                 再RequestStart()。
 */
HAL_StatusTypeDef Inverter_Control_SetOutputFrequency(
    float output_frequency_hz);

/**
 * @brief          处理启动、停止和故障状态
 * @param[in]      dc_bus_v 当前直流母线电压
 *
 * @note           在main()的while(1)中持续调用；20kHz算法不在这里执行。
 */
void Inverter_Control_Service(float dc_bus_v);

/**
 * @brief          在ADC1全传输回调中执行一次双PR闭环和DPWM更新
 * @param[in]      u_ab_v 采样线电压Uab
 * @param[in]      u_bc_v 采样线电压Ubc
 * @param[in]      i_a_a 采样A相线电流
 * @param[in]      i_c_a 采样C相线电流
 * @param[in]      dc_bus_v 当前直流母线电压
 */
void Inverter_Control_Update(float u_ab_v,
                             float u_bc_v,
                             float i_a_a,
                             float i_c_a,
                             float dc_bus_v);

/**
 * @brief          清除PR历史量、相角和软启动参考
 */
void Inverter_Control_Reset(void);

/**
 * @brief          请求在主循环中启动三相逆变
 */
void Inverter_Control_RequestStart(void);

/**
 * @brief          请求在主循环中停止三相逆变
 */
void Inverter_Control_RequestStop(void);

/**
 * @brief          立即关闭三相逆变六路输出并复位控制器
 */
void Inverter_Control_Disable(void);

#endif /* INVERTER_CONTROL_H */
