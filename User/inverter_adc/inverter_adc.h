#ifndef INVERTER_ADC_H
#define INVERTER_ADC_H

#include <stdint.h>
#include "adc.h"

/** ADC参考电压，单位为V。 */
#define INVERTER_ADC_VREF                       3.3f

/** 12位ADC满量程计数值。 */
#define INVERTER_ADC_FULL_SCALE                 4095.0f

/** ADC2规则组Rank数量，同时也是ADC2循环DMA缓冲区长度。 */
#define INVERTER_ADC2_DMA_LENGTH                2U

/** ADC1六Rank DMA中第一路逆变电压索引：Rank4/ADC1_IN6/PC0。 */
#define INVERTER_ADC_VOLTAGE_1_INDEX            3U

/** ADC1六Rank DMA中第二路逆变电压索引：Rank5/ADC1_IN7/PC1。 */
#define INVERTER_ADC_VOLTAGE_2_INDEX            4U

/** ADC2两Rank DMA中第一路逆变电流索引：Rank1/ADC2_IN3/PA6。 */
#define INVERTER_ADC_CURRENT_1_INDEX            0U

/** ADC2两Rank DMA中第二路逆变电流索引：Rank2/ADC2_IN4/PA7。 */
#define INVERTER_ADC_CURRENT_2_INDEX            1U

/*
 * 下面的偏置和倍率均为固定手动标定参数。
 * 本模块不会执行ADC内部自动校准，也不会在上电时自动计算零点。
 */

/** 第一路逆变电压采样偏置，单位为V。 */
#define INVERTER_ADC_VOLTAGE_1_OFFSET_V         1.749f

/** 第二路逆变电压采样偏置，单位为V。 */
#define INVERTER_ADC_VOLTAGE_2_OFFSET_V         1.7582f

/** 第一路逆变电压采样还原倍率：实际电压/ADC引脚电压。 */
#define INVERTER_ADC_VOLTAGE_1_SCALE            60.51f

/** 第二路逆变电压采样还原倍率：实际电压/ADC引脚电压。 */
#define INVERTER_ADC_VOLTAGE_2_SCALE            59.84f

/** 第一路逆变电流采样偏置，单位为V。 */
#define INVERTER_ADC_CURRENT_1_OFFSET_V         1.628f

/** 第二路逆变电流采样偏置，单位为V。 */
#define INVERTER_ADC_CURRENT_2_OFFSET_V         1.628f

/** 第一路逆变电流采样增益，单位为V/A。 */
#define INVERTER_ADC_CURRENT_1_GAIN_V_PER_A     0.08334f

/** 第二路逆变电流采样增益，单位为V/A。 */
#define INVERTER_ADC_CURRENT_2_GAIN_V_PER_A     0.08334f

/** 第一路逆变电压偏置对应的ADC计数。 */
#define INVERTER_ADC_VOLTAGE_1_OFFSET_COUNT \
    (INVERTER_ADC_VOLTAGE_1_OFFSET_V * \
     INVERTER_ADC_FULL_SCALE / INVERTER_ADC_VREF)

/** 第二路逆变电压偏置对应的ADC计数。 */
#define INVERTER_ADC_VOLTAGE_2_OFFSET_COUNT \
    (INVERTER_ADC_VOLTAGE_2_OFFSET_V * \
     INVERTER_ADC_FULL_SCALE / INVERTER_ADC_VREF)

/** 第一路逆变电流偏置对应的ADC计数。 */
#define INVERTER_ADC_CURRENT_1_OFFSET_COUNT \
    (INVERTER_ADC_CURRENT_1_OFFSET_V * \
     INVERTER_ADC_FULL_SCALE / INVERTER_ADC_VREF)

/** 第二路逆变电流偏置对应的ADC计数。 */
#define INVERTER_ADC_CURRENT_2_OFFSET_COUNT \
    (INVERTER_ADC_CURRENT_2_OFFSET_V * \
     INVERTER_ADC_FULL_SCALE / INVERTER_ADC_VREF)

/** 第一路逆变电压每个ADC计数对应的实际电压，单位为V/count。 */
#define INVERTER_ADC_VOLTAGE_1_V_PER_COUNT \
    (INVERTER_ADC_VREF * INVERTER_ADC_VOLTAGE_1_SCALE / \
     INVERTER_ADC_FULL_SCALE)

/** 第二路逆变电压每个ADC计数对应的实际电压，单位为V/count。 */
#define INVERTER_ADC_VOLTAGE_2_V_PER_COUNT \
    (INVERTER_ADC_VREF * INVERTER_ADC_VOLTAGE_2_SCALE / \
     INVERTER_ADC_FULL_SCALE)

/** 第一路逆变电流每个ADC计数对应的实际电流，单位为A/count。 */
#define INVERTER_ADC_CURRENT_1_A_PER_COUNT \
    (INVERTER_ADC_VREF / INVERTER_ADC_FULL_SCALE / \
     INVERTER_ADC_CURRENT_1_GAIN_V_PER_A)

/** 第二路逆变电流每个ADC计数对应的实际电流，单位为A/count。 */
#define INVERTER_ADC_CURRENT_2_A_PER_COUNT \
    (INVERTER_ADC_VREF / INVERTER_ADC_FULL_SCALE / \
     INVERTER_ADC_CURRENT_2_GAIN_V_PER_A)

/**
 * @brief ADC2两Rank循环DMA缓冲区
 */
typedef struct
{
    uint16_t sample[INVERTER_ADC2_DMA_LENGTH];
    /**< sample[0]=ADC2_IN3，sample[1]=ADC2_IN4。 */
} Inverter_ADC_DmaBufferTypeDef;

/**
 * @brief 三相逆变四路采样原始ADC快照
 */
typedef struct
{
    uint16_t voltage_1; /**< ADC1 Rank4/ADC1_IN6/PC0原始值。 */
    uint16_t voltage_2; /**< ADC1 Rank5/ADC1_IN7/PC1原始值。 */
    uint16_t current_1; /**< ADC2 Rank1/ADC2_IN3/PA6原始值。 */
    uint16_t current_2; /**< ADC2 Rank2/ADC2_IN4/PA7原始值。 */
} Inverter_ADC_RawDataTypeDef;

/**
 * @brief 三相逆变采样物理量
 */
typedef struct
{
    float voltage_1_v; /**< 第一路逆变电压，单位为V。 */
    float voltage_2_v; /**< 第二路逆变电压，单位为V。 */
    float current_1_a; /**< 第一路逆变电流，单位为A。 */
    float current_2_a; /**< 第二路逆变电流，单位为A。 */
} Inverter_ADC_MeasurementTypeDef;

/**
 * @brief 三相逆变ADC采样运行状态
 */
typedef struct
{
    Inverter_ADC_DmaBufferTypeDef dma;
    /**< ADC2两路电流循环DMA缓冲区。 */

    Inverter_ADC_RawDataTypeDef raw;
    /**< 本控制周期锁存的四路原始值。 */

    Inverter_ADC_MeasurementTypeDef measurement;
    /**< 本控制周期换算后的四路物理量。 */

    uint32_t update_count;
    /**< ADC1六Rank全传输完成次数。 */

    uint8_t data_ready;
    /**< 四路采样已更新标志，1表示数据有效。 */
} Inverter_ADC_StateTypeDef;

/** 三相逆变ADC采样对外运行数据。 */
extern volatile Inverter_ADC_StateTypeDef inverter_adc_state;

/**
 * @brief          初始化三相逆变ADC采样模块
 * @param[in]      adc1_dma_buffer ADC1六Rank循环DMA缓冲区首地址
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR adc1_dma_buffer为空
 *
 * @note           本函数不启动ADC、不注册动态回调、不执行ADC校准。
 */
HAL_StatusTypeDef Inverter_ADC_Init(
    const volatile uint16_t *adc1_dma_buffer);

/**
 * @brief          启动ADC2两Rank循环DMA采样
 * @param[in]      none
 * @retval         HAL_OK 启动成功
 * @retval         其他HAL状态 ADC2 DMA启动失败
 *
 * @note           ADC2启动后等待HRTIM_TRG1上升沿。
 * @note           本函数不执行ADC内部自动校准。
 */
HAL_StatusTypeDef Inverter_ADC_Start(void);

/**
 * @brief          停止ADC2循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Stop(void);

/**
 * @brief          处理ADC1六Rank DMA全传输完成事件
 * @param[in]      none
 * @retval         none
 *
 * @note           该函数由HAL_ADC_ConvCpltCallback()自动调用。
 */
void Inverter_ADC_ProcessFullTransfer(void);

/**
 * @brief          清除三相逆变采样结果和更新标志
 * @param[in]      none
 * @retval         none
 *
 * @note           不停止ADC2 DMA，也不解除ADC1 DMA缓冲区绑定。
 */
void Inverter_ADC_ClearData(void);

/**
 * @brief          初始化三相逆变的 ADC 采样及过流保护
 * @param[in]      adc1_dma_buffer ADC1 DMA 缓冲区地址，
 *                                 与 PFC 采样共用同一个 ADC1 DMA 缓冲区
 * @retval         HAL_OK    初始化并启动成功
 * @retval         HAL_ERROR ADC、模拟看门狗或 DMA 启动失败
 * @note           必须在 MX_DMA_Init()、MX_ADC1_Init()、
 *                 MX_ADC2_Init() 和 MX_HRTIM1_Init() 执行完成后调用
 */
HAL_StatusTypeDef Inverter_App_Init(
    const volatile uint16_t *adc1_dma_buffer);

#endif /* INVERTER_ADC_H */
