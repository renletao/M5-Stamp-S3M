/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __STAMP_STM32_H
#define __STAMP_STM32_H

#ifdef __cplusplus

extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define STAMP_STM32_DEFAULT_I2C_ADDRESS (0x32)  // Default I2C address

#define STAMP_STM32_BATTERY_VOLTAGE_BYTE0_REG (0x00)  // Battery voltage byte 0 (LSB)
#define STAMP_STM32_BATTERY_VOLTAGE_BYTE1_REG (0x01)  // Battery voltage byte 1
#define STAMP_STM32_BATTERY_VOLTAGE_BYTE2_REG (0x02)  // Battery voltage byte 2
#define STAMP_STM32_BATTERY_VOLTAGE_BYTE3_REG (0x03)  // Battery voltage byte 3 (MSB)
#define STAMP_STM32_BATTERY_ADC_BYTE0_REG     (0x04)  // Battery ADC value byte 0
#define STAMP_STM32_BATTERY_ADC_BYTE1_REG     (0x05)  // Battery ADC value byte 1
#define STAMP_STM32_AW32001_STATUS_REG        (0x06)  // AW32001 status
#define STAMP_STM32_AW32001_FAULT_REG         (0x07)  // AW32001 fault
#define STAMP_STM32_POWER_OFF_REG             (0x10)  // Power off command
#define STAMP_STM32_VOUT_REG                  (0x11)  // Output voltage control
#define STAMP_STM32_USB_DETECT_REG            (0x12)  // USB detection status
#define STAMP_STM32_CHARGE_VOLTAGE_REG        (0x13)  // Charging voltage setting
#define STAMP_STM32_CHARGE_CURRENT_REG        (0x14)  // Charging current setting
#define STAMP_STM32_CHARGE_ENABLE_REG         (0x15)  // Enable/disable charging
#define STAMP_STM32_RESET_REG                 (0x20)  // Reset command
#define STAMP_STM32_ENTER_DOWNLOAD_MODE_REG   (0x21)  // Enter download/DFU mode
#define STAMP_STM32_WKUP_LEVEL_REG            (0x22)  // Wakeup level setting
#define STAMP_STM32_BUTTON_REG                (0x30)  // Button status
#define STAMP_STM32_SYS_STATUS_REG            (0xFB)  // System status
#define STAMP_STM32_BOOTLOADER_VERSION_REG    (0xFC)  // Bootloader version
#define STAMP_STM32_FW_VERSION_REG            (0xFE)  // Firmware version
#define STAMP_STM32_I2C_ADDRESS_REG           (0xFF)  // Device I2C address

// 回调函数类型定义
typedef int (*stamp_stm32_read_reg_cb_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
typedef int (*stamp_stm32_write_reg_cb_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

// 驱动配置结构体
typedef struct {
    uint8_t dev_addr;                         // 设备I2C地址
    stamp_stm32_read_reg_cb_t read_reg_cb;    // 读寄存器回调函数
    stamp_stm32_write_reg_cb_t write_reg_cb;  // 写寄存器回调函数
} stamp_stm32_t;

/**
 * @brief 读取寄存器数据。
 *
 * 从指定设备的指定寄存器读取数据并存入提供的缓冲区。
 *
 * @param dev 设备结构体指针，包含设备的配置信息。
 * @param reg_addr 寄存器地址。
 * @param data 数据缓冲区，读取的数据将存储在此缓冲区中。
 * @param len 要读取的数据长度。
 *
 * @return 成功时返回 0，失败时返回负值。
 */
int stamp_stm32_reads_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);

/**
 * @brief 写入寄存器数据。
 *
 * 向指定设备的指定寄存器写入数据。
 *
 * @param dev 设备结构体指针，包含设备的配置信息。
 * @param reg_addr 寄存器地址。
 * @param data 要写入的数据。
 * @param len 要写入的数据长度。
 *
 * @return 成功时返回 0，失败时返回负值。
 */
int stamp_stm32_writes_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);

/**
 * @brief 初始化设备配置结构体。
 *
 * 该函数用于初始化 `stamp_stm32_t` 设备配置结构体，设置设备的 I2C 地址，
 * 并且配置读写寄存器的回调函数。该结构体将在后续的 I2C 操作中使用。
 *
 * @param dev 设备结构体指针，指向要初始化的 `stamp_stm32_t` 结构体。
 * @param dev_addr 设备的 I2C 地址，将用于设备通信。
 * @param read_reg_cb 读寄存器的回调函数指针，负责处理 I2C 读操作。
 * @param write_reg_cb 写寄存器的回调函数指针，负责处理 I2C 写操作。
 *
 * @return 成功时返回 0，失败时返回负值（如 -1）。
 */
int stamp_stm32_init(stamp_stm32_t *dev, uint8_t dev_addr, stamp_stm32_read_reg_cb_t read_reg_cb,
                     stamp_stm32_write_reg_cb_t write_reg_cb);

/**
 * @brief 获取电池电压
 * @param dev 设备指针
 * @param battery_value 存储电池电压的变量
 * @return 状态码
 */
int stamp_stm32_get_battery_voltage(stamp_stm32_t *dev, uint32_t *battery_value);

/**
 * @brief 获取电池ADC值
 * @param dev 设备指针
 * @param battery_adc 存储电池ADC值的变量
 * @return 状态码
 */
int stamp_stm32_get_battery_adc(stamp_stm32_t *dev, uint16_t *battery_adc);

/**
 * @brief 获取AW32001状态
 * @param dev 设备指针
 * @param status 存储状态的变量
 * @return 状态码
 */
int stamp_stm32_get_aw32001_status(stamp_stm32_t *dev, uint8_t *status);

/**
 * @brief 获取AW32001故障状态
 * @param dev 设备指针
 * @param fault 存储故障状态的变量
 * @return 状态码
 */
int stamp_stm32_get_aw32001_fault(stamp_stm32_t *dev, uint8_t *fault);

/**
 * @brief 设置进入下载模式配置
 * @param dev 设备指针
 * @return 状态码
 */
int stamp_stm32_set_power_off_config(stamp_stm32_t *dev);

/**
 * @brief 获取VOUT配置
 * @param dev 设备指针
 * @param vout 存储VOUT配置的变量
 * @return 状态码
 */
int stamp_stm32_get_vout_config(stamp_stm32_t *dev, uint8_t *vout);

/**
 * @brief 设置VOUT配置
 * @param dev 设备指针
 * @param vout VOUT配置
 * @return 状态码
 */
int stamp_stm32_set_vout_config(stamp_stm32_t *dev, uint8_t vout);

/**
 * @brief 获取USB检测配置
 * @param dev 设备指针
 * @param usb_detect USB检测状态
 * @return 状态码
 */
int stamp_stm32_get_usb_detect_config(stamp_stm32_t *dev, uint8_t *usb_detect);

/**
 * @brief 获取充电电压配置
 * @param dev 设备指针
 * @param voltage 存储充电电压的变量
 * @return 状态码
 */
int stamp_stm32_get_charge_voltage_config(stamp_stm32_t *dev, uint8_t *voltage);

/**
 * @brief 设置充电电压配置
 * @param dev 设备指针
 * @param voltage 充电电压
 * @return 状态码
 */
int stamp_stm32_set_charge_voltage_config(stamp_stm32_t *dev, uint8_t voltage);

/**
 * @brief 获取充电电流配置
 * @param dev 设备指针
 * @param current 存储充电电流的变量
 * @return 状态码
 */
int stamp_stm32_get_charge_current_config(stamp_stm32_t *dev, uint8_t *current);

/**
 * @brief 设置充电电流配置
 * @param dev 设备指针
 * @param current 充电电流
 * @return 状态码
 */
int stamp_stm32_set_charge_current_config(stamp_stm32_t *dev, uint8_t current);

/**
 * @brief 获取充电使能配置
 * @param dev 设备指针
 * @param charge_status 充电使能状态
 * @return 状态码
 */
int stamp_stm32_get_charge_enable_config(stamp_stm32_t *dev, uint8_t *charge_status);

/**
 * @brief 设置充电使能配置
 * @param dev 设备指针
 * @param charge_status 充电使能状态
 * @return 状态码
 */
int stamp_stm32_set_charge_enable_config(stamp_stm32_t *dev, uint8_t charge_status);

/**
 * @brief 设置复位配置
 * @param dev 设备指针
 * @return 状态码
 */
int stamp_stm32_set_reset_config(stamp_stm32_t *dev);

/**
 * @brief 设置进入下载模式配置
 * @param dev 设备指针
 * @return 状态码
 */
int stamp_stm32_set_enter_download_mode_config(stamp_stm32_t *dev);

/**
 * @brief 获取唤醒电平配置
 * @param dev 设备指针
 * @param wake_up 存储唤醒电平的变量
 * @return 状态码
 */
int stamp_stm32_get_wake_up_level_config(stamp_stm32_t *dev, uint8_t *wake_up);

/**
 * @brief 设置唤醒电平配置
 * @param dev 设备指针
 * @param wake_up 唤醒电平
 * @return 状态码
 */
int stamp_stm32_set_wake_up_level_config(stamp_stm32_t *dev, uint8_t wake_up);

/**
 * @brief 获取按键状态
 * @param dev 设备指针
 * @param button_status 存储按键状态的变量
 * @return 状态码
 */
int stamp_stm32_get_button_status(stamp_stm32_t *dev, uint8_t *button_status);

/**
 * @brief 获取系统状态
 * @param dev 设备指针
 * @param sys_status 存储系统状态的变量
 * @return 状态码
 */
int stamp_stm32_get_sys_status(stamp_stm32_t *dev, uint8_t *sys_status);

/**
 * @brief 获取Bootloader版本
 * @param dev 设备指针
 * @param ver 存储版本的变量
 * @return 状态码
 */
int stamp_stm32_get_bootloader_version(stamp_stm32_t *dev, uint8_t *ver);

/**
 * @brief 获取固件版本
 * @param dev 设备指针
 * @param ver 存储版本的变量
 * @return 状态码
 */
int stamp_stm32_get_firmware_version(stamp_stm32_t *dev, uint8_t *ver);

/**
 * @brief 获取I2C地址
 * @param dev 设备指针
 * @param addr 存储I2C地址的变量
 * @return 状态码
 */
int stamp_stm32_get_i2c_addr(stamp_stm32_t *dev, uint8_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* __STAMP_STM32_H */