# AC-AC-Converter
Three-phase AC-AC converter based on SPWM control for National Undergraduate Electronic Design Contest.
# Three-Phase AC-AC Converter Based on SPWM Control

<p align="center">
  <img src="./Images/system_photo.jpg" width="600">
</p>

<p align="center">
  2026 National Undergraduate Electronic Design Contest - AC-AC Converter Project
</p>


## 📌 Project Overview

本项目来源于2026年全国大学生电子设计竞赛A题《AC-AC变换电路》。

项目设计并实现了一套单相交流输入、三相交流输出的电力电子变换系统，实现交流电能变换及输出稳定控制。

系统采用：

- AC-DC无桥PWM整流
- DC-Link母线
- 三相逆变桥
- SPWM调制控制
- 输出滤波

实现三相对称交流输出。


---

## 🎯 Design Objectives

| Parameter | Requirement |
| :-- | :-- |
| Input Voltage | 36V AC |
| Input Frequency | 50Hz |
| Output Voltage | 32V Line Voltage |
| Output Frequency | 30Hz / 60Hz Adjustable |
| Load Current | 3A |
| Output Type | Three-phase AC |


---

# 🏗 System Architecture


<p align="center">
  <img src="./Images/system_block.png" width="700">
</p>


系统主要由以下模块组成：
Single-phase AC Input

    ↓

Rectifier Circuit

    ↓

DC-Link

    ↓

Three-phase Inverter Bridge

    ↓

LC Filter

    ↓

Three-phase AC Output



---

# 🔧 Hardware Design


## 1. Power Stage

主要设计内容：

- 整流电路设计
- DC母线电容选型
- MOSFET逆变桥设计
- 功率器件散热设计


<p align="center">
<img src="./Images/power_board.jpg" width="500">
</p>


---

## 2. Gate Driver Circuit

设计内容：

- MOSFET驱动电路
- 上下桥臂隔离驱动
- 死区时间控制


---

## 3. Output Filter

设计LC滤波网络：

- 降低PWM高频谐波
- 提高输出电压波形质量


---

# 💻 Software Design


## Control Algorithm

基于MCU实现：

- SPWM波形生成
- 三相相位控制
- 输出频率调节
- PWM占空比控制
- 系统保护逻辑


程序结构：


Main Loop

├── PWM Initialization

├── SPWM Generate

├── ADC Sampling

├── Protection Check

└── Output Control



---

# 📊 Test Result


## Output Waveform


<p align="center">
<img src="./Images/output_waveform.png" width="600">
</p>


测试结果：

| Test Item | Result |
| :-- | :-- |
| Output Voltage | 32V |
| Frequency | 60Hz |
| Load Current | 2A |
| THD | <2% |
| Efficiency | >95% |


---

# 🛠 Development Tools


## Hardware

- STM32 MCU
- MOSFET Power Stage
- Gate Driver IC
- Current/Voltage Sampling Circuit


## Software

- STM32CubeIDE
- Keil MDK
- Altium Designer
- MATLAB/Simulink


---

# 👨‍💻 Team Contribution


## My Responsibility

负责：

- 功率主电路设计
- PCB Layout
- MOSFET驱动设计
- 控制程序开发
- 系统调试与测试


Team:

| Member | Responsibility |
|-|-|
| Member A | Hardware Design |
| Member B | Software Development |
| Member C | Testing & Documentation |


---

# 📷 Prototype


<p align="center">
<img src="./Images/prototype.jpg" width="700">
</p>


---

# 📚 Project Background

Competition:

2026 National Undergraduate Electronic Design Contest

Topic:

A - AC-AC Converter


---

# ⭐ Highlights

- Complete power electronic system design
- Three-phase inverter implementation
- SPWM control algorithm
- Hardware prototype verification
- Engineering testing process
