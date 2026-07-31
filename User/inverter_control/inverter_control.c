#include "inverter_control.h"

#include <math.h>

#include "inverter_adc_watchdog.h"
#include "inverter_modulation.h"
#include "inverter_svpwm.h"

/** 三相逆变控制运行状态，可在调试器中观察。 */
volatile Inverter_Control_StateTypeDef
    inverter_control_state;

/** 当前选择的调制方式，可在调试器中观察。 */
volatile Inverter_ModulationModeTypeDef
    inverter_modulation_mode;

/** 写1请求启动三相逆变。 */
volatile uint8_t inverter_start_request;

/** 写1请求停止三相逆变。 */
volatile uint8_t inverter_stop_request;

/** 三相逆变控制最近一次HAL执行状态。 */
volatile HAL_StatusTypeDef inverter_control_last_status;

/**
 * @brief 将浮点数限制到指定范围
 */
static float Inverter_Control_Clamp(float value,
                                    float minimum,
                                    float maximum)
{
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

/**
 * @brief 配置双线性变换离散准PR系数
 */
static void Inverter_PR_Configure(
    volatile Inverter_PR_ControllerTypeDef *controller,
    float kp,
    float kr,
    float wc_rad_s,
    float resonant_frequency_hz,
    float output_limit)
{
    float transform_k;
    float w0_rad_s;
    float denominator;

    transform_k = 2.0f * INVERTER_CONTROL_FREQ_HZ;
    w0_rad_s =
        INVERTER_TWO_PI * resonant_frequency_hz;

    denominator =
        transform_k * transform_k +
        2.0f * wc_rad_s * transform_k +
        w0_rad_s * w0_rad_s;

    controller->kp = kp;
    controller->b0 =
        2.0f * kr * wc_rad_s * transform_k /
        denominator;
    controller->b2 = -controller->b0;
    controller->a1 =
        2.0f *
        (w0_rad_s * w0_rad_s -
         transform_k * transform_k) /
        denominator;
    controller->a2 =
        (transform_k * transform_k -
         2.0f * wc_rad_s * transform_k +
         w0_rad_s * w0_rad_s) /
        denominator;
    controller->output_limit = output_limit;
    controller->error_z1 = 0.0f;
    controller->error_z2 = 0.0f;
    controller->resonant_z1 = 0.0f;
    controller->resonant_z2 = 0.0f;
}

/**
 * @brief 清除单个PR历史量，不改变系数
 */
static void Inverter_PR_Reset(
    volatile Inverter_PR_ControllerTypeDef *controller)
{
    controller->error_z1 = 0.0f;
    controller->error_z2 = 0.0f;
    controller->resonant_z1 = 0.0f;
    controller->resonant_z2 = 0.0f;
}

/**
 * @brief 按当前输出频率配置两路电压PR和两路电流PR
 */
static void Inverter_Control_ConfigurePRControllers(
    float output_frequency_hz)
{
    Inverter_PR_Configure(
        &inverter_control_state.voltage_pr_a,
        INVERTER_VOLTAGE_PR_KP,
        INVERTER_VOLTAGE_PR_KR,
        INVERTER_VOLTAGE_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_CURRENT_REFERENCE_LIMIT_A);
    Inverter_PR_Configure(
        &inverter_control_state.voltage_pr_c,
        INVERTER_VOLTAGE_PR_KP,
        INVERTER_VOLTAGE_PR_KR,
        INVERTER_VOLTAGE_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_CURRENT_REFERENCE_LIMIT_A);
    Inverter_PR_Configure(
        &inverter_control_state.current_pr_a,
        INVERTER_CURRENT_PR_KP,
        INVERTER_CURRENT_PR_KR,
        INVERTER_CURRENT_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_VOLTAGE_CORRECTION_LIMIT_V);
    Inverter_PR_Configure(
        &inverter_control_state.current_pr_c,
        INVERTER_CURRENT_PR_KP,
        INVERTER_CURRENT_PR_KR,
        INVERTER_CURRENT_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_VOLTAGE_CORRECTION_LIMIT_V);
}

/**
 * @brief 执行一次离散准PR
 */
static float Inverter_PR_Run(
    volatile Inverter_PR_ControllerTypeDef *controller,
    float error)
{
    float resonant;
    float output;
    float limited_output;

    resonant =
        -controller->a1 * controller->resonant_z1 -
        controller->a2 * controller->resonant_z2 +
        controller->b0 * error +
        controller->b2 * controller->error_z2;

    output = controller->kp * error + resonant;
    limited_output =
        Inverter_Control_Clamp(
            output,
            -controller->output_limit,
            controller->output_limit);

    /*
     * 饱和时把本周期谐振状态回算到受限输出，抑制PR历史量继续增大。
     */
    if (limited_output != output) {
        resonant =
            limited_output - controller->kp * error;
        resonant =
            Inverter_Control_Clamp(
                resonant,
                -controller->output_limit,
                controller->output_limit);
    }

    controller->error_z2 = controller->error_z1;
    controller->error_z1 = error;
    controller->resonant_z2 = controller->resonant_z1;
    controller->resonant_z1 = resonant;

    return limited_output;
}

/**
 * @brief 初始化双PR、两种调制器和Timer A/B/C计数器
 */
HAL_StatusTypeDef Inverter_Control_Init(void)
{
    HAL_StatusTypeDef status;

    inverter_control_state =
        (Inverter_Control_StateTypeDef){0};
    inverter_start_request = 0U;
    inverter_stop_request = 0U;
    inverter_control_last_status = HAL_OK;
    inverter_control_state.output_frequency_hz =
        INVERTER_OUTPUT_FREQ_DEFAULT_HZ;
    inverter_modulation_mode =
        INVERTER_MODULATION_MODE_DEFAULT;

    if ((inverter_modulation_mode !=
         INVERTER_MODULATION_MODE_DPWM1) &&
        (inverter_modulation_mode !=
         INVERTER_MODULATION_MODE_SVPWM)) {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    Inverter_Control_ConfigurePRControllers(
        inverter_control_state.output_frequency_hz);

    status = Inverter_DPWM_Init();
    if (status != HAL_OK) {
        inverter_control_last_status = status;
        return status;
    }

    status = Inverter_SVPWM_Init();
    if (status != HAL_OK) {
        Inverter_DPWM_Disable();
        inverter_control_last_status = status;
        return status;
    }

    status = Inverter_DPWM_StartCounters();
    if (status != HAL_OK) {
        Inverter_DPWM_Disable();
        inverter_control_last_status = status;
        return status;
    }

    inverter_control_state.initialized = 1U;

#if (INVERTER_CONTROL_AUTO_START != 0U)
    inverter_start_request = 1U;
#endif

    return HAL_OK;
}

/**
 * @brief 在停机状态选择30Hz或60Hz，并同步重算四个PR谐振系数
 */
HAL_StatusTypeDef Inverter_Control_SetOutputFrequency(
    float output_frequency_hz)
{
    float selected_frequency_hz;

    if (inverter_control_state.initialized == 0U) {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    if (inverter_control_state.enabled != 0U) {
        inverter_control_last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    if (fabsf(output_frequency_hz -
              INVERTER_OUTPUT_FREQ_LOW_HZ) <= 0.1f) {
        selected_frequency_hz =
            INVERTER_OUTPUT_FREQ_LOW_HZ;
    } else if (fabsf(output_frequency_hz -
                     INVERTER_OUTPUT_FREQ_HIGH_HZ) <= 0.1f) {
        selected_frequency_hz =
            INVERTER_OUTPUT_FREQ_HIGH_HZ;
    } else {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    inverter_control_state.output_frequency_hz =
        selected_frequency_hz;
    Inverter_Control_ConfigurePRControllers(
        selected_frequency_hz);
    Inverter_Control_Reset();

    inverter_control_last_status = HAL_OK;
    return HAL_OK;
}

/**
 * @brief 在停机状态选择DPWM1或连续SVPWM
 */
HAL_StatusTypeDef Inverter_Control_SetModulationMode(
    Inverter_ModulationModeTypeDef modulation_mode)
{
    if (inverter_control_state.initialized == 0U) {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    if ((inverter_control_state.enabled != 0U) ||
        (inverter_dpwm_state.outputs_enabled != 0U)) {
        inverter_control_last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    if ((modulation_mode != INVERTER_MODULATION_MODE_DPWM1) &&
        (modulation_mode != INVERTER_MODULATION_MODE_SVPWM)) {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    inverter_modulation_mode = modulation_mode;
    Inverter_Control_Reset();
    Inverter_SVPWM_Reset();

    inverter_control_last_status = HAL_OK;
    return HAL_OK;
}

/**
 * @brief 清除PR历史量、相角和软启动参考
 */
void Inverter_Control_Reset(void)
{
    Inverter_PR_Reset(
        &inverter_control_state.voltage_pr_a);
    Inverter_PR_Reset(
        &inverter_control_state.voltage_pr_c);
    Inverter_PR_Reset(
        &inverter_control_state.current_pr_a);
    Inverter_PR_Reset(
        &inverter_control_state.current_pr_c);

    inverter_control_state.phase_rad = 0.0f;
    inverter_control_state.line_voltage_reference_rms_v = 0.0f;
    inverter_control_state.i_a_reference_a = 0.0f;
    inverter_control_state.i_b_reference_a = 0.0f;
    inverter_control_state.i_c_reference_a = 0.0f;
    inverter_control_state.v_a_correction_v = 0.0f;
    inverter_control_state.v_c_correction_v = 0.0f;
    inverter_control_state.v_a_command_v = 0.0f;
    inverter_control_state.v_b_command_v = 0.0f;
    inverter_control_state.v_c_command_v = 0.0f;
    inverter_control_state.m_a = 0.0f;
    inverter_control_state.m_b = 0.0f;
    inverter_control_state.m_c = 0.0f;
}

/**
 * @brief 处理启动、停止和故障状态
 */
void Inverter_Control_Service(float dc_bus_v)
{
    HAL_StatusTypeDef status;

    if (inverter_control_state.initialized == 0U) {
        return;
    }

    inverter_control_state.dc_bus_v = dc_bus_v;

    if (Inverter_ADC_Watchdog_IsFaulted() != 0U) {
        Inverter_Control_Disable();
        inverter_start_request = 0U;
        inverter_stop_request = 0U;
        inverter_control_last_status = HAL_ERROR;
        return;
    }

    if (inverter_stop_request != 0U) {
        inverter_stop_request = 0U;
        inverter_start_request = 0U;
        Inverter_Control_Disable();
        inverter_control_last_status = HAL_OK;
        return;
    }

    if (inverter_start_request == 0U) {
        return;
    }

    /*
     * 保留启动请求，直到PFC把直流母线建立到安全阈值。
     * 这样自动启动和调试器手动启动都不会在母线过低时误投入。
     */
    if (dc_bus_v < INVERTER_MIN_DC_BUS_V) {
        inverter_control_last_status = HAL_BUSY;
        return;
    }

    inverter_start_request = 0U;

    if (inverter_control_state.enabled != 0U) {
        inverter_control_last_status = HAL_BUSY;
        return;
    }

    Inverter_Control_Reset();
    status = Inverter_DPWM_Enable();
    inverter_control_last_status = status;

    if (status == HAL_OK) {
        inverter_control_state.enabled = 1U;
    } else {
        Inverter_DPWM_Disable();
    }
}

/**
 * @brief 在ADC1全传输回调中执行一次双PR闭环和所选调制更新
 */
void Inverter_Control_Update(float u_ab_v,
                             float u_bc_v,
                             float i_a_a,
                             float i_c_a,
                             float dc_bus_v)
{
    float phase_peak_v;
    float voltage_error_a;
    float voltage_error_c;
    float current_error_a;
    float current_error_c;
    float current_reference_max;
    float current_reference_scale;
    float phase_step_rad;

    inverter_control_state.u_ab_v = u_ab_v;
    inverter_control_state.u_bc_v = u_bc_v;
    inverter_control_state.i_a_a = i_a_a;
    inverter_control_state.i_c_a = i_c_a;
    inverter_control_state.i_b_a = -i_a_a - i_c_a;
    inverter_control_state.dc_bus_v = dc_bus_v;

    /*
     * 由Uab、Ubc重构三相三线制等效相电压：
     * Va=(2Uab+Ubc)/3；
     * Vb=(-Uab+Ubc)/3；
     * Vc=-(Uab+2Ubc)/3。
     */
    inverter_control_state.v_a_v =
        (2.0f * u_ab_v + u_bc_v) / 3.0f;
    inverter_control_state.v_b_v =
        (-u_ab_v + u_bc_v) / 3.0f;
    inverter_control_state.v_c_v =
        -(u_ab_v + 2.0f * u_bc_v) / 3.0f;

    if ((inverter_control_state.initialized == 0U) ||
        (inverter_control_state.enabled == 0U)) {
        return;
    }

    if ((Inverter_ADC_Watchdog_IsFaulted() != 0U) ||
        (dc_bus_v < INVERTER_MIN_DC_BUS_V)) {
        Inverter_Control_Disable();
        inverter_control_last_status = HAL_ERROR;
        return;
    }

    inverter_control_state.line_voltage_reference_rms_v +=
        INVERTER_LINE_VOLTAGE_SLEW_V_PER_S /
        INVERTER_CONTROL_FREQ_HZ;
    if (inverter_control_state.line_voltage_reference_rms_v >
        INVERTER_LINE_VOLTAGE_TARGET_RMS_V) {
        inverter_control_state.line_voltage_reference_rms_v =
            INVERTER_LINE_VOLTAGE_TARGET_RMS_V;
    }

    phase_peak_v =
        inverter_control_state.line_voltage_reference_rms_v *
        INVERTER_LINE_RMS_TO_PHASE_PEAK;

    inverter_control_state.v_a_reference_v =
        phase_peak_v *
        sinf(inverter_control_state.phase_rad);
    /* ABC正序：B滞后A 120°，C超前A 120°。 */
    inverter_control_state.v_c_reference_v =
        phase_peak_v *
        sinf(inverter_control_state.phase_rad +
             INVERTER_TWO_PI_OVER_THREE);
    inverter_control_state.v_b_reference_v =
        -inverter_control_state.v_a_reference_v -
        inverter_control_state.v_c_reference_v;

    voltage_error_a =
        inverter_control_state.v_a_reference_v -
        inverter_control_state.v_a_v;
    voltage_error_c =
        inverter_control_state.v_c_reference_v -
        inverter_control_state.v_c_v;

    inverter_control_state.i_a_reference_a =
        Inverter_PR_Run(
            &inverter_control_state.voltage_pr_a,
            voltage_error_a);
    inverter_control_state.i_c_reference_a =
        Inverter_PR_Run(
            &inverter_control_state.voltage_pr_c,
            voltage_error_c);
    inverter_control_state.i_b_reference_a =
        -inverter_control_state.i_a_reference_a -
        inverter_control_state.i_c_reference_a;

    /*
     * 两个外环独立限幅后，恢复出的B相瞬态值仍可能超过单相限值。
     * 对三相参考统一缩放，保持iA+iB+iC=0且不改变相间比例。
     */
    current_reference_max =
        fmaxf(
            fabsf(inverter_control_state.i_a_reference_a),
            fmaxf(
                fabsf(inverter_control_state.i_b_reference_a),
                fabsf(inverter_control_state.i_c_reference_a)));

    if (current_reference_max >
        INVERTER_CURRENT_REFERENCE_LIMIT_A) {
        current_reference_scale =
            INVERTER_CURRENT_REFERENCE_LIMIT_A /
            current_reference_max;
        inverter_control_state.i_a_reference_a *=
            current_reference_scale;
        inverter_control_state.i_b_reference_a *=
            current_reference_scale;
        inverter_control_state.i_c_reference_a *=
            current_reference_scale;
    }

    current_error_a =
        inverter_control_state.i_a_reference_a -
        inverter_control_state.i_a_a;
    current_error_c =
        inverter_control_state.i_c_reference_a -
        inverter_control_state.i_c_a;

    inverter_control_state.v_a_correction_v =
        Inverter_PR_Run(
            &inverter_control_state.current_pr_a,
            current_error_a);
    inverter_control_state.v_c_correction_v =
        Inverter_PR_Run(
            &inverter_control_state.current_pr_c,
            current_error_c);

    /*
     * 参考相电压前馈与电流PR修正量相加，再由A/C恢复B相。
     */
    inverter_control_state.v_a_command_v =
        inverter_control_state.v_a_reference_v +
        inverter_control_state.v_a_correction_v;
    inverter_control_state.v_c_command_v =
        inverter_control_state.v_c_reference_v +
        inverter_control_state.v_c_correction_v;
    inverter_control_state.v_b_command_v =
        -inverter_control_state.v_a_command_v -
        inverter_control_state.v_c_command_v;

    inverter_control_state.m_a =
        2.0f * inverter_control_state.v_a_command_v /
        dc_bus_v;
    inverter_control_state.m_b =
        2.0f * inverter_control_state.v_b_command_v /
        dc_bus_v;
    inverter_control_state.m_c =
        2.0f * inverter_control_state.v_c_command_v /
        dc_bus_v;

    if (inverter_modulation_mode ==
        INVERTER_MODULATION_MODE_DPWM1) {
        Inverter_DPWM_Update(
            inverter_control_state.m_a,
            inverter_control_state.m_b,
            inverter_control_state.m_c);
    } else if (inverter_modulation_mode ==
               INVERTER_MODULATION_MODE_SVPWM) {
        Inverter_SVPWM_Update(
            inverter_control_state.m_a,
            inverter_control_state.m_b,
            inverter_control_state.m_c);
    } else {
        Inverter_Control_Disable();
        inverter_control_last_status = HAL_ERROR;
        return;
    }

    phase_step_rad =
        INVERTER_TWO_PI *
        inverter_control_state.output_frequency_hz /
        INVERTER_CONTROL_FREQ_HZ;
    inverter_control_state.phase_rad += phase_step_rad;

    if (inverter_control_state.phase_rad >=
        INVERTER_TWO_PI) {
        inverter_control_state.phase_rad -=
            INVERTER_TWO_PI;
    }

    inverter_control_state.update_count++;
}

/**
 * @brief 请求在主循环中启动三相逆变
 */
void Inverter_Control_RequestStart(void)
{
    inverter_stop_request = 0U;
    inverter_start_request = 1U;
}

/**
 * @brief 请求在主循环中停止三相逆变
 */
void Inverter_Control_RequestStop(void)
{
    inverter_start_request = 0U;
    inverter_stop_request = 1U;
}

/**
 * @brief 立即关闭三相逆变六路输出并复位控制器
 */
void Inverter_Control_Disable(void)
{
    Inverter_DPWM_Disable();
    inverter_control_state.enabled = 0U;
    Inverter_Control_Reset();
    Inverter_SVPWM_Reset();
}
