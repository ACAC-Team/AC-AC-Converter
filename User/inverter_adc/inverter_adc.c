#include "inverter_adc.h"
#include "inverter_adc_watchdog.h"

/** 三相逆变ADC采样对外运行数据。 */
volatile Inverter_ADC_StateTypeDef inverter_adc_state;

/** ADC1六Rank循环DMA缓冲区首地址。 */
static const volatile uint16_t *inverter_adc1_dma_buffer;

/**
 * @brief          初始化三相逆变的 ADC 采样及过流保护
 * @param[in]      adc1_dma_buffer ADC1 DMA 缓冲区地址，
 *                                 与 PFC 采样共用同一个 ADC1 DMA 缓冲区
 * @retval         HAL_OK    初始化并启动成功
 * @retval         HAL_ERROR ADC、模拟看门狗或 DMA 启动失败
 * @note           必须在 MX_DMA_Init()、MX_ADC1_Init()、
 *                 MX_ADC2_Init() 和 MX_HRTIM1_Init() 执行完成后调用
 */
HAL_StatusTypeDef Inverter_App_Init(uint16_t *adc1_dma_buffer)
{
    /* 绑定 ADC1 DMA 缓冲区，并初始化三相逆变采样状态。 */
    if (Inverter_ADC_Init(adc1_dma_buffer) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 配置并启用 ADC2 模拟看门狗，用于两路逆变电流过流保护。 */
    if (Inverter_ADC_Watchdog_Init() != HAL_OK) {
        return HAL_ERROR;
    }

    /*
     * 启动 ADC2 DMA。
     * ADC2 只负责同步采集两路线电流，不产生 DMA 半传输和全传输中断。
     */
    if (Inverter_ADC_Start() != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief          初始化三相逆变ADC采样模块
 * @param[in]      adc1_dma_buffer ADC1六Rank循环DMA缓冲区首地址
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR adc1_dma_buffer为空
 */
HAL_StatusTypeDef Inverter_ADC_Init(
    const volatile uint16_t *adc1_dma_buffer)
{
    if (adc1_dma_buffer == NULL) {
        return HAL_ERROR;
    }

    inverter_adc_state =
        (Inverter_ADC_StateTypeDef){0};

    inverter_adc1_dma_buffer = adc1_dma_buffer;

    return HAL_OK;
}

/**
 * @brief          启动ADC2两Rank循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Start(void)
{
    HAL_StatusTypeDef status;

    /*
     * 停止可能残留的ADC2 DMA。
     * 本模块按要求不调用HAL_ADCEx_Calibration_Start()。
     */
    (void)HAL_ADC_Stop_DMA(&hadc2);

    inverter_adc_state.dma =
        (Inverter_ADC_DmaBufferTypeDef){0};

    /*
     * 启动ADC2两Rank循环DMA：
     * Rank1=PA6/ADC2_IN3，Rank2=PA7/ADC2_IN4。
     * ADC2等待HRTIM_TRG1上升沿，不会自由运行。
     */
    status = HAL_ADC_Start_DMA(
        &hadc2,
        (uint32_t *)inverter_adc_state.dma.sample,
        INVERTER_ADC2_DMA_LENGTH);

    if (status != HAL_OK) {
        return status;
    }

    /*
     * ADC2只负责硬件触发和DMA搬运。
     * 当前IOC未使能DMA1 Channel4 NVIC，这里再同时关闭DMA通道的
     * HT、TC、TE中断源和NVIC入口，避免后续配置变化引入ADC2 DMA中断。
     * ADC2模拟看门狗走ADC1_2_IRQn，不受这里影响。
     */
    __HAL_DMA_DISABLE_IT(
        hadc2.DMA_Handle,
        DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);

    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);

    __HAL_DMA_CLEAR_FLAG(
        hadc2.DMA_Handle,
        __HAL_DMA_GET_GI_FLAG_INDEX(
            hadc2.DMA_Handle));

    HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);

    return HAL_OK;
}

/**
 * @brief          停止ADC2循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Stop(void)
{
    return HAL_ADC_Stop_DMA(&hadc2);
}

/**
 * @brief          处理ADC1六Rank DMA全传输完成事件
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_ProcessFullTransfer(void)
{
    uint16_t voltage_1_raw;
    uint16_t voltage_2_raw;
    uint16_t current_1_raw;
    uint16_t current_2_raw;

    if (inverter_adc1_dma_buffer == NULL) {
        return;
    }

    /*
     * ADC1 Rank4和Rank5为两路逆变电压。
     * Rank6为占位采样，保证ADC1全传输事件发生在后三个Rank完成后。
     */
    voltage_1_raw =
        inverter_adc1_dma_buffer[
            INVERTER_ADC_VOLTAGE_1_INDEX];

    voltage_2_raw =
        inverter_adc1_dma_buffer[
            INVERTER_ADC_VOLTAGE_2_INDEX];

    /*
     * ADC1和ADC2由同一个HRTIM_TRG1同时触发。
     * ADC2只有两个Rank，转换早于ADC1六Rank完成，因此此处直接锁存。
     */
    current_1_raw =
        inverter_adc_state.dma.sample[
            INVERTER_ADC_CURRENT_1_INDEX];

    current_2_raw =
        inverter_adc_state.dma.sample[
            INVERTER_ADC_CURRENT_2_INDEX];

    inverter_adc_state.raw.voltage_1 = voltage_1_raw;
    inverter_adc_state.raw.voltage_2 = voltage_2_raw;
    inverter_adc_state.raw.current_1 = current_1_raw;
    inverter_adc_state.raw.current_2 = current_2_raw;

    /* 根据固定偏置和手动标定倍率换算两路逆变电压。 */
    inverter_adc_state.measurement.voltage_1_v =
        ((float)voltage_1_raw -
         INVERTER_ADC_VOLTAGE_1_OFFSET_COUNT) *
        INVERTER_ADC_VOLTAGE_1_V_PER_COUNT;

    inverter_adc_state.measurement.voltage_2_v =
        ((float)voltage_2_raw -
         INVERTER_ADC_VOLTAGE_2_OFFSET_COUNT) *
        INVERTER_ADC_VOLTAGE_2_V_PER_COUNT;

    /* 根据固定偏置和手动标定增益换算两路逆变电流。 */
    inverter_adc_state.measurement.current_1_a =
        ((float)current_1_raw -
         INVERTER_ADC_CURRENT_1_OFFSET_COUNT) *
        INVERTER_ADC_CURRENT_1_A_PER_COUNT;

    inverter_adc_state.measurement.current_2_a =
        ((float)current_2_raw -
         INVERTER_ADC_CURRENT_2_OFFSET_COUNT) *
        INVERTER_ADC_CURRENT_2_A_PER_COUNT;

    inverter_adc_state.update_count++;
    inverter_adc_state.data_ready = 1U;
}

/**
 * @brief          清除三相逆变采样结果和更新标志
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_ClearData(void)
{
    inverter_adc_state.raw =
        (Inverter_ADC_RawDataTypeDef){0};

    inverter_adc_state.measurement =
        (Inverter_ADC_MeasurementTypeDef){0};

    inverter_adc_state.update_count = 0U;
    inverter_adc_state.data_ready = 0U;
}

/**
 * @brief          ADC规则组DMA全传输完成普通HAL回调
 * @param[in]      hadc ADC句柄地址
 * @retval         none
 *
 * @note           工程中未启用HAL动态回调注册。
 * @note           ADC2 DMA的HT/TC中断已关闭，因此正常情况下只处理ADC1。
 */
void HAL_ADC_ConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    Inverter_ADC_ProcessFullTransfer();
}
