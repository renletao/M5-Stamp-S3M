/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __STAMP_CASE_H
#define __STAMP_CASE_H

#ifdef __cplusplus

extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stamp_stm32.h"

void stamp_stm32_test_case_1(stamp_stm32_t *dev);  // 读取测试电池电压 ,轮询读取

void stamp_stm32_test_case_2(stamp_stm32_t *dev);  // 读取测试电池ADC ，轮询读取

void stamp_stm32_test_case_3(stamp_stm32_t *dev);  // 读取AW32001的状态、错误

void stamp_stm32_test_case_4(stamp_stm32_t *dev);  // 关闭esp32

void stamp_stm32_test_case_5(stamp_stm32_t *dev);  // VOUT输出参数

void stamp_stm32_test_case_6(stamp_stm32_t *dev);  // USB检测

void stamp_stm32_test_case_7(stamp_stm32_t *dev);  // 充电电压，充电电流，充电使能

void stamp_stm32_test_case_8(stamp_stm32_t *dev);  // 复位ESP32

void stamp_stm32_test_case_9(stamp_stm32_t *dev);  // 进入下载模式

void stamp_stm32_test_case_10(stamp_stm32_t *dev);  // 唤醒电平

void stamp_stm32_test_case_11(stamp_stm32_t *dev);  // 按键检测

void stamp_stm32_test_case_12(stamp_stm32_t *dev);  // 初始化状态

void stamp_stm32_test_case_13(stamp_stm32_t *dev);  // IAP版本号

void stamp_stm32_test_case_14(stamp_stm32_t *dev);  // 软件版本号

void stamp_stm32_test_case_15(stamp_stm32_t *dev);  // I2C addr 测试

void stamp_stm32_test_case_16(stamp_stm32_t *dev);  // 压力测试

void stamp_stm32_test_case_17(stamp_stm32_t *dev);  // KEY 映射测试

void stamp_stm32_test_case_18(stamp_stm32_t *dev);  // RTC测试

void stamp_stm32_test_case_19(stamp_stm32_t *dev);  // WIFI测试

void stamp_stm32_test_case_20(stamp_stm32_t *dev);  // GPIO测试

#ifdef __cplusplus
}
#endif

#endif /* __STAMP_CASE_H */