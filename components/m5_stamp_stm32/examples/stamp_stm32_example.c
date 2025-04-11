#include <stdio.h>
#include "../include/stamp_stm32.h"

// 示例I2C读寄存器回调函数
static int example_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    printf("读取设备地址: 0x%02X, 寄存器地址: 0x%02X, 长度: %d\n", dev_addr, reg_addr, len);

    // 在实际应用中，这里应该是真正的I2C读取操作
    // 这里仅为演示，填充一些示例数据
    if (data != NULL && len > 0) {
        for (int i = 0; i < len; i++) {
            data[i] = i + 1;  // 示例数据
        }
    }

    return 0;  // 返回0表示成功
}

// 示例I2C写寄存器回调函数
static int example_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    printf("写入设备地址: 0x%02X, 寄存器地址: 0x%02X, 长度: %d, 数据: ", dev_addr, reg_addr, len);

    // 在实际应用中，这里应该是真正的I2C写入操作
    // 这里仅为演示，打印要写入的数据
    if (data != NULL && len > 0) {
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", data[i]);
        }
    }
    printf("\n");

    return 0;  // 返回0表示成功
}

int main()
{
    int ret;
    stamp_stm32_t stamp_stm32_dev;

    uint8_t fw_version       = 0;
    uint32_t battery_voltage = 0;
    uint8_t aw32001_status   = 0;
    uint8_t aw32001_fault    = 0;
    uint8_t usb_status       = 0;

    printf("Stamp-Stm32驱动示例开始\n");

    // 初始化Stamp-Stm32设备
    ret = unit_hexstep_init(&stamp_stm32_dev, STAMP_STM32_DEFAULT_I2C_ADDRESS, example_i2c_read_reg,
                            example_i2c_write_reg);
    if (ret != 0) {
        printf("Stamp-Stm32初始化失败\n");
        return -1;
    }

    printf("Stamp-Stm32初始化成功\n");

    // 获取固件版本
    ret = stamp_stm32_get_firmware_version(&stamp_stm32_dev, &fw_version);
    if (ret == 0) {
        printf("固件版本: 0x%02X\n", fw_version);
    } else {
        printf("获取固件版本失败\n");
    }

    // 获取电池电压
    ret = stamp_stm32_get_battery_voltage(&stamp_stm32_dev, &battery_voltage);
    if (ret == 0) {
        printf("电池电压: %lumV\n", battery_voltage);
    } else {
        printf("获取电池电压失败\n");
    }

    // 获取AW32001的状态
    ret = stamp_stm32_get_aw32001_status(&stamp_stm32_dev, &aw32001_status);
    if (ret == 0) {
        printf("AW32001的状态: %d\n", aw32001_status);
    } else {
        printf("获取AW32001的状态失败\n");
    }

    // 获取AW32001的错误
    ret = stamp_stm32_get_aw32001_fault(&stamp_stm32_dev, &aw32001_fault);
    if (ret == 0) {
        printf("AW32001的错误: %d\n", aw32001_status);
    } else {
        printf("获取AW32001的错误失败\n");
    }

    // 获取USB的状态
    ret = stamp_stm32_get_usb_detect_config(&stamp_stm32_dev, &usb_status);
    if (ret == 0) {
        if (usb_status) {
            printf("USB插入\n");
        } else {
            printf("USB未插入\n");
        }

    } else {
        printf("获取USB的状态失败\n");
    }

    printf("Stamp-Stm32驱动示例结束\n");

    return 0;
}