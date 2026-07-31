#include "inverter_dpwm.h"

/** DPWM运行状态，可在调试器中观察。 */
volatile Inverter_DPWM_StateTypeDef inverter_dpwm_state;

/**
 * @brief 将浮点数限制到指定范围
 */
static float Inverter_DPWM_ClampFloat(float value,
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
 * @brief 根据三相指令的大小关系生成调试用扇区号
 */
static uint8_t Inverter_DPWM_GetSector(float m_a,
                                       float m_b,
                                       float m_c)
{
    if (m_a >= m_b) {
        if (m_b >= m_c) {
            return 1U; /* A >= B >= C */
        }

        if (m_a >= m_c) {
            return 6U; /* A >= C > B */
        }

        return 5U;     /* C > A >= B */
    }

    if (m_a >= m_c) {
        return 2U;     /* B > A >= C */
    }

    if (m_b >= m_c) {
        return 3U;     /* B >= C > A */
    }

    return 4U;         /* C > B > A */
}

/**
 * @brief 将上管占空比换算为中心对齐CMP1值
 *
 * @note 当前HRTIM配置满足：
 *       duty=(PER-CMP1)/PER。
 *       DPWM钳位相允许CMP1=PER（0%）或CMP1=0（100%）；
 *       非钳位相保留比较边界余量。
 */
static uint32_t Inverter_DPWM_DutyToCompare(float duty,
                                             uint8_t is_clamped)
{
    float compare_f;
    uint32_t compare;
    uint32_t period;

    period = inverter_dpwm_state.period_counts;
    duty = Inverter_DPWM_ClampFloat(duty, 0.0f, 1.0f);

    if (is_clamped != 0U) {
        if (duty <= 0.0f) {
            return period;
        }

        if (duty >= 1.0f) {
            return 0U;
        }
    }

    compare_f = (1.0f - duty) * (float)period;
    compare = (uint32_t)(compare_f + 0.5f);

    if (compare < INVERTER_DPWM_COMPARE_GUARD_COUNTS) {
        compare = INVERTER_DPWM_COMPARE_GUARD_COUNTS;
    }

    if (compare >
        (period - INVERTER_DPWM_COMPARE_GUARD_COUNTS)) {
        compare =
            period - INVERTER_DPWM_COMPARE_GUARD_COUNTS;
    }

    return compare;
}

/**
 * @brief 写入Timer A/B/C的CMP1预装载寄存器
 */
static void Inverter_DPWM_WritePreload(uint32_t compare_a,
                                        uint32_t compare_b,
                                        uint32_t compare_c)
{
    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_A,
        HRTIM_COMPAREUNIT_1,
        compare_a);

    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_B,
        HRTIM_COMPAREUNIT_1,
        compare_b);

    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_C,
        HRTIM_COMPAREUNIT_1,
        compare_c);
}

/**
 * @brief 初始化Timer A/B/C的安全比较值和DPWM运行状态
 */
HAL_StatusTypeDef Inverter_DPWM_Init(void)
{
    HAL_StatusTypeDef status;
    uint32_t period_a;
    uint32_t period_b;
    uint32_t period_c;
    uint32_t compare_half;

    hhrtim1.Instance->sCommonRegs.ODISR =
        INVERTER_DPWM_ALL_OUTPUTS;

    period_a =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A]
            .PERxR;
    period_b =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B]
            .PERxR;
    period_c =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C]
            .PERxR;

    if ((period_a <=
         (2U * INVERTER_DPWM_COMPARE_GUARD_COUNTS)) ||
        (period_a != period_b) ||
        (period_a != period_c)) {
        return HAL_ERROR;
    }

    inverter_dpwm_state =
        (Inverter_DPWM_StateTypeDef){0};
    inverter_dpwm_state.period_counts = period_a;
    inverter_dpwm_state.modulation_scale = 1.0f;
    inverter_dpwm_state.duty_a_next = 0.5f;
    inverter_dpwm_state.duty_b_next = 0.5f;
    inverter_dpwm_state.duty_c_next = 0.5f;

    compare_half = period_a / 2U;
    inverter_dpwm_state.compare_a_next = compare_half;
    inverter_dpwm_state.compare_b_next = compare_half;
    inverter_dpwm_state.compare_c_next = compare_half;

    Inverter_DPWM_WritePreload(
        compare_half,
        compare_half,
        compare_half);

    status = HAL_HRTIM_SoftwareUpdate(
        &hhrtim1,
        HRTIM_TIMERUPDATE_A |
        HRTIM_TIMERUPDATE_B |
        HRTIM_TIMERUPDATE_C);
    if (status != HAL_OK) {
        return status;
    }

    inverter_dpwm_state.initialized = 1U;
    return HAL_OK;
}

/**
 * @brief 启动Timer A/B/C计数器
 */
HAL_StatusTypeDef Inverter_DPWM_StartCounters(void)
{
    HAL_StatusTypeDef status;

    if (inverter_dpwm_state.initialized == 0U) {
        return HAL_ERROR;
    }

    status = HAL_HRTIM_WaveformCounterStart(
        &hhrtim1,
        INVERTER_DPWM_COUNTERS);

    if (status == HAL_OK) {
        inverter_dpwm_state.counters_running = 1U;
    }

    return status;
}

/**
 * @brief 以三相50%同相安全状态使能六路互补输出
 */
HAL_StatusTypeDef Inverter_DPWM_Enable(void)
{
    HAL_StatusTypeDef status;
    uint32_t compare_half;

    if ((inverter_dpwm_state.initialized == 0U) ||
        (inverter_dpwm_state.counters_running == 0U) ||
        (inverter_dpwm_state.outputs_enabled != 0U)) {
        return HAL_ERROR;
    }

    compare_half = inverter_dpwm_state.period_counts / 2U;
    Inverter_DPWM_WritePreload(
        compare_half,
        compare_half,
        compare_half);

    inverter_dpwm_state.duty_a_next = 0.5f;
    inverter_dpwm_state.duty_b_next = 0.5f;
    inverter_dpwm_state.duty_c_next = 0.5f;
    inverter_dpwm_state.compare_a_next = compare_half;
    inverter_dpwm_state.compare_b_next = compare_half;
    inverter_dpwm_state.compare_c_next = compare_half;
    inverter_dpwm_state.clamp = INVERTER_DPWM_CLAMP_NONE;

    status = HAL_HRTIM_WaveformOutputStart(
        &hhrtim1,
        INVERTER_DPWM_ALL_OUTPUTS);

    if (status == HAL_OK) {
        inverter_dpwm_state.outputs_enabled = 1U;
    }

    return status;
}

/**
 * @brief 计算DPWM零序并预装载下一周期三相CMP1
 */
void Inverter_DPWM_Update(float m_a,
                          float m_b,
                          float m_c)
{
    float phase[3];
    float maximum;
    float minimum;
    float span;
    float scale;
    float zero_sequence;
    float duty[3];
    uint8_t maximum_index;
    uint8_t minimum_index;
    uint8_t clamp_index;
    uint8_t clamp_high;
    uint8_t is_clamped_a;
    uint8_t is_clamped_b;
    uint8_t is_clamped_c;
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t compare_c;

    if (inverter_dpwm_state.initialized == 0U) {
        return;
    }

    inverter_dpwm_state.requested_m_a = m_a;
    inverter_dpwm_state.requested_m_b = m_b;
    inverter_dpwm_state.requested_m_c = m_c;

    phase[0] = m_a;
    phase[1] = m_b;
    phase[2] = m_c;

    maximum = phase[0];
    minimum = phase[0];
    maximum_index = 0U;
    minimum_index = 0U;

    if (phase[1] > maximum) {
        maximum = phase[1];
        maximum_index = 1U;
    }
    if (phase[2] > maximum) {
        maximum = phase[2];
        maximum_index = 2U;
    }
    if (phase[1] < minimum) {
        minimum = phase[1];
        minimum_index = 1U;
    }
    if (phase[2] < minimum) {
        minimum = phase[2];
        minimum_index = 2U;
    }

    /*
     * 零序注入不改变线电压，但三相指令跨度必须不大于2。
     * 过调制时统一缩放三相量，避免分别限幅破坏相间关系。
     */
    span = maximum - minimum;
    scale = 1.0f;
    if (span > 2.0f) {
        scale = 2.0f / span;
        phase[0] *= scale;
        phase[1] *= scale;
        phase[2] *= scale;
        maximum *= scale;
        minimum *= scale;
    }

    /* DPWM1：按当前三相最大值与最小值之和选择高侧或低侧钳位。 */
    clamp_high =
        ((maximum + minimum) >= 0.0f) ? 1U : 0U;

    if (clamp_high != 0U) {
        zero_sequence = 1.0f - maximum;
        clamp_index = maximum_index;
    } else {
        zero_sequence = -1.0f - minimum;
        clamp_index = minimum_index;
    }

    duty[0] =
        0.5f * (phase[0] + zero_sequence + 1.0f);
    duty[1] =
        0.5f * (phase[1] + zero_sequence + 1.0f);
    duty[2] =
        0.5f * (phase[2] + zero_sequence + 1.0f);

    /* 消除浮点舍入，确保钳位相得到严格的0%或100%。 */
    duty[clamp_index] =
        (clamp_high != 0U) ? 1.0f : 0.0f;

    duty[0] = Inverter_DPWM_ClampFloat(duty[0], 0.0f, 1.0f);
    duty[1] = Inverter_DPWM_ClampFloat(duty[1], 0.0f, 1.0f);
    duty[2] = Inverter_DPWM_ClampFloat(duty[2], 0.0f, 1.0f);

    is_clamped_a = (clamp_index == 0U) ? 1U : 0U;
    is_clamped_b = (clamp_index == 1U) ? 1U : 0U;
    is_clamped_c = (clamp_index == 2U) ? 1U : 0U;

    compare_a =
        Inverter_DPWM_DutyToCompare(duty[0], is_clamped_a);
    compare_b =
        Inverter_DPWM_DutyToCompare(duty[1], is_clamped_b);
    compare_c =
        Inverter_DPWM_DutyToCompare(duty[2], is_clamped_c);

    Inverter_DPWM_WritePreload(
        compare_a,
        compare_b,
        compare_c);

    inverter_dpwm_state.modulation_scale = scale;
    inverter_dpwm_state.zero_sequence = zero_sequence;
    inverter_dpwm_state.applied_m_a =
        phase[0] + zero_sequence;
    inverter_dpwm_state.applied_m_b =
        phase[1] + zero_sequence;
    inverter_dpwm_state.applied_m_c =
        phase[2] + zero_sequence;
    inverter_dpwm_state.duty_a_next = duty[0];
    inverter_dpwm_state.duty_b_next = duty[1];
    inverter_dpwm_state.duty_c_next = duty[2];
    inverter_dpwm_state.compare_a_next = compare_a;
    inverter_dpwm_state.compare_b_next = compare_b;
    inverter_dpwm_state.compare_c_next = compare_c;
    inverter_dpwm_state.sector =
        Inverter_DPWM_GetSector(
            phase[0],
            phase[1],
            phase[2]);

    if (clamp_index == 0U) {
        inverter_dpwm_state.clamp =
            (clamp_high != 0U) ?
            INVERTER_DPWM_CLAMP_A_HIGH :
            INVERTER_DPWM_CLAMP_A_LOW;
    } else if (clamp_index == 1U) {
        inverter_dpwm_state.clamp =
            (clamp_high != 0U) ?
            INVERTER_DPWM_CLAMP_B_HIGH :
            INVERTER_DPWM_CLAMP_B_LOW;
    } else {
        inverter_dpwm_state.clamp =
            (clamp_high != 0U) ?
            INVERTER_DPWM_CLAMP_C_HIGH :
            INVERTER_DPWM_CLAMP_C_LOW;
    }
}

/**
 * @brief 通过ODISR立即关闭Timer A/B/C六路功率输出
 */
void Inverter_DPWM_Disable(void)
{
    hhrtim1.Instance->sCommonRegs.ODISR =
        INVERTER_DPWM_ALL_OUTPUTS;

    inverter_dpwm_state.outputs_enabled = 0U;
    inverter_dpwm_state.clamp =
        INVERTER_DPWM_CLAMP_NONE;
}
