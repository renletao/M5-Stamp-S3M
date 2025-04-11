#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "stamp_stm32.h"
#include "stamp_case.h"
#include "i2c_bus.h"
#include "driver/gpio.h"

#define PORT_NUM  I2C_NUM_0
#define PORT_SCL  47
#define PORT_SDA  48
#define PORT_FREQ 400000

stamp_stm32_t stamp_stm32_dev;
i2c_bus_handle_t i2c_handle;
i2c_bus_device_handle_t stamp_stm32_dev_handle;

static int i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int ret = 1;
    for (int i = 0; i < 3; i++)  // retry 3 times
    {
        ret = i2c_bus_read_bytes(stamp_stm32_dev_handle, reg_addr, len, data);
        // ESP_LOGI(TAG, "读取寄存器: 0x%02X, 数据: 0x%02X\n", reg_addr, data[0]);
        if (ret == 0) {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return ret;
}

static int i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int ret = 1;
    for (int i = 0; i < 3; i++)  // retry 3 times
    {
        ret = i2c_bus_write_bytes(stamp_stm32_dev_handle, reg_addr, len, data);
        // ESP_LOGI(TAG, "写入寄存器: 0x%02X, 数据: 0x%02X\n", reg_addr, data[0]);
        if (ret == 0) {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return ret;
}

void app_main(void)
{
    gpio_set_drive_capability(PORT_SDA, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(PORT_SCL, GPIO_DRIVE_CAP_3);

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PORT_SDA,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = PORT_SCL,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = PORT_FREQ,
    };
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    i2c_handle      = i2c_bus_create(PORT_NUM, &conf);
    uint8_t address = 0;


    // i2c_bus_scan(i2c_handle, &address, 1);

    stamp_stm32_dev_handle = i2c_bus_device_create(i2c_handle, STAMP_STM32_DEFAULT_I2C_ADDRESS, 0);

    int ret = stamp_stm32_init(&stamp_stm32_dev, STAMP_STM32_DEFAULT_I2C_ADDRESS, i2c_read_reg, i2c_write_reg);
    if (ret != 0) {
        // 初始化失败处理
    }

    // stamp_stm32_test_case_1(&stamp_stm32_dev); // 读取测试电池电压 ,轮询读取
    // stamp_stm32_test_case_2(&stamp_stm32_dev); // 读取测试电池ADC ，轮询读取
    // stamp_stm32_test_case_3(&stamp_stm32_dev); // 读取AW32001的状态、错误
    // stamp_stm32_test_case_4(&stamp_stm32_dev); // 关闭esp32
    // stamp_stm32_test_case_5(&stamp_stm32_dev); // VOUT输出参数
    // stamp_stm32_test_case_6(&stamp_stm32_dev); // USB检测
    // stamp_stm32_test_case_7(&stamp_stm32_dev); // 充电电压，充电电流，充电使能
    // stamp_stm32_test_case_8(&stamp_stm32_dev); // 复位ESP32
    // stamp_stm32_test_case_9(&stamp_stm32_dev); // 进入下载模式
    // stamp_stm32_test_case_10(&stamp_stm32_dev); // 唤醒电平
    // stamp_stm32_test_case_11(&stamp_stm32_dev); // 按键检测
    // stamp_stm32_test_case_12(&stamp_stm32_dev); // 初始化状态
    // stamp_stm32_test_case_13(&stamp_stm32_dev); // IAP版本号
    // stamp_stm32_test_case_14(&stamp_stm32_dev); // 软件版本号
    // stamp_stm32_test_case_15(&stamp_stm32_dev); // I2C addr 测试
    stamp_stm32_test_case_16(&stamp_stm32_dev); // 压力测试
    // stamp_stm32_test_case_17(&stamp_stm32_dev); // KEY 映射测试
    // stamp_stm32_test_case_18(&stamp_stm32_dev); // RTC测试
    // stamp_stm32_test_case_19(&stamp_stm32_dev); // WIFI测试
    // stamp_stm32_test_case_20(&stamp_stm32_dev);  // GPIO测试
}
