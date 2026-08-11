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

- AC-DC整流
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
| Load Current | 2A |
| Output Type | Three-phase AC |


---

# 🏗 System Architecture


<p align="center">
  <img src="./Images/system_block.png" width="700">
</p>


系统主要由以下模块组成：
