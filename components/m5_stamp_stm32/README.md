# M5-Stamp-C6M 驱动组件

这是一个用于M5-Stamp-S3设备的STM32驱动组件，通过I2C协议实现通信。

## 特性

- 使用回调函数机制实现I2C通信，适配不同平台
- 在通信前检查回调函数是否正常初始化
- 支持电池电压读取，充电设置，按键读取，电源管理等功能
- 支持I2C修改，配置保存到Flash

## 文件结构

```
components/m5_stamp_stm32/
├── include/
│   └── stamp_stm32.h      # 头文件，包含接口定义和常量
├── src/
│   └── stamp_stm32.c      # 源文件，实现驱动功能
└── examples/
    └── stamp_stm32_example.c  # 使用示例
```

## 使用方法

### 1. 初始化

首先需要定义读写寄存器的回调函数，然后初始化设备：

```c
// 定义I2C读写回调函数
int my_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
int my_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

// 定义设备句柄
stamp_stm32_t stamp_stm32_dev;

// 初始化设备
int ret = stamp_stm32_init(&stamp_stm32_dev, STAMP_STM32_DEFAULT_I2C_ADDRESS, 
                           my_i2c_read_reg, my_i2c_write_reg);
if (ret != 0) {
    // 初始化失败处理
}
```

### 2. 基本功能

```c
// 获取电池电压
int stamp_stm32_get_battery_voltage(stamp_stm32_t *dev, uint32_t *battery_value);

// 获取电池ADC值
int stamp_stm32_get_battery_adc(stamp_stm32_t *dev, uint16_t *battery_adc);

// 获取AW32001状态
int stamp_stm32_get_aw32001_status(stamp_stm32_t *dev, uint8_t *status);

// 获取AW32001故障状态
int stamp_stm32_get_aw32001_fault(stamp_stm32_t *dev, uint8_t *fault);

// 设置进入下载模式配置
int stamp_stm32_set_power_off_config(stamp_stm32_t *dev);

// 获取VOUT配置
int stamp_stm32_get_vout_config(stamp_stm32_t *dev, uint8_t *vout);

// 设置VOUT配置
int stamp_stm32_set_vout_config(stamp_stm32_t *dev, uint8_t vout);

// 获取USB检测配置
int stamp_stm32_get_usb_detect_config(stamp_stm32_t *dev, uint8_t *usb_detect);

// 获取充电电压配置
int stamp_stm32_get_charge_voltage_config(stamp_stm32_t *dev, uint8_t *voltage);

// 设置充电电压配置
int stamp_stm32_set_charge_voltage_config(stamp_stm32_t *dev, uint8_t voltage);

// 获取充电电流配置
int stamp_stm32_get_charge_current_config(stamp_stm32_t *dev, uint8_t *current)

// 设置充电电流配置
int stamp_stm32_set_charge_current_config(stamp_stm32_t *dev, uint8_t current);

// 获取充电使能配置
int stamp_stm32_get_charge_enable_config(stamp_stm32_t *dev, uint8_t *charge_status);

// 设置充电使能配置
int stamp_stm32_set_charge_enable_config(stamp_stm32_t *dev, uint8_t charge_status);

// 设置复位配置
int stamp_stm32_set_reset_config(stamp_stm32_t *dev);

// 设置进入下载模式配置
int stamp_stm32_set_enter_download_mode_config(stamp_stm32_t *dev);

// 获取唤醒电平配置
int stamp_stm32_get_wake_up_level_config(stamp_stm32_t *dev, uint8_t *wake_up);

// 设置唤醒电平配置
int stamp_stm32_set_wake_up_level_config(stamp_stm32_t *dev, uint8_t wake_up);

// 获取按键状态
int stamp_stm32_get_button_status(stamp_stm32_t *dev, uint8_t *button_status);

// 获取系统状态
int stamp_stm32_get_sys_status(stamp_stm32_t *dev, uint8_t *sys_status);

// 获取Bootloader版本
int stamp_stm32_get_bootloader_version(stamp_stm32_t *dev, uint8_t *ver);

// 获取固件版本
int stamp_stm32_get_firmware_version(stamp_stm32_t *dev, uint8_t *ver);

// 获取I2C地址
int stamp_stm32_get_i2c_addr(stamp_stm32_t *dev, uint8_t *addr);
```

## 回调函数实现示例

以下是回调函数实现的示例（需根据实际平台进行适配）：

```c
// ESP32平台I2C读寄存器实现示例
int esp32_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // 发送寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
    // 读取数据
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK) ? 0 : -1;
}

// ESP32平台I2C写寄存器实现示例
int esp32_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK) ? 0 : -1;
}
```

## 注意事项

- 使用前必须先初始化设备并设置有效的回调函数
- 所有通信函数都会在执行前检查回调函数是否已正确初始化
- 回调函数需要根据实际使用的硬件平台和I2C驱动进行适配 