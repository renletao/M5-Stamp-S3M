#include "../include/stamp_stm32.h"
#include <string.h>

static int stamp_stm32_read_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);
static int stamp_stm32_write_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);

int stamp_stm32_init(stamp_stm32_t *dev, uint8_t dev_addr, stamp_stm32_read_reg_cb_t read_reg_cb,
                     stamp_stm32_write_reg_cb_t write_reg_cb)
{
    if (dev == NULL || read_reg_cb == NULL || write_reg_cb == NULL) {
        return -1;
    }

    memset(dev, 0, sizeof(stamp_stm32_t));
    dev->dev_addr     = dev_addr;
    dev->read_reg_cb  = read_reg_cb;
    dev->write_reg_cb = write_reg_cb;
    return 0;
}

static int stamp_stm32_check_callbacks(stamp_stm32_t *dev)
{
    if (dev == NULL || dev->read_reg_cb == NULL || dev->write_reg_cb == NULL) {
        return -1;
    }
    return 0;
}

static int stamp_stm32_read_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (stamp_stm32_check_callbacks(dev) != 0 || data == NULL) {
        return -1;
    }

    return dev->read_reg_cb(dev->dev_addr, reg_addr, data, len);
}

static int stamp_stm32_write_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if (stamp_stm32_check_callbacks(dev) != 0 || data == NULL) {
        return -1;
    }

    return dev->write_reg_cb(dev->dev_addr, reg_addr, data, len);
}

int stamp_stm32_reads_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    return stamp_stm32_read_reg(dev, reg_addr, data, len);
}
int stamp_stm32_writes_reg(stamp_stm32_t *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    return stamp_stm32_write_reg(dev, reg_addr, data, len);
}

int stamp_stm32_get_battery_voltage(stamp_stm32_t *dev, uint32_t *battery_value)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_BATTERY_VOLTAGE_BYTE0_REG, (uint8_t *)battery_value, 4);
}

int stamp_stm32_get_battery_adc(stamp_stm32_t *dev, uint16_t *battery_adc)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_BATTERY_ADC_BYTE0_REG, (uint8_t *)battery_adc, 2);
}

int stamp_stm32_get_aw32001_status(stamp_stm32_t *dev, uint8_t *status)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_AW32001_STATUS_REG, status, 1);
}

int stamp_stm32_get_aw32001_fault(stamp_stm32_t *dev, uint8_t *fault)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_AW32001_FAULT_REG, fault, 1);
}

int stamp_stm32_set_power_off_config(stamp_stm32_t *dev)
{
    uint8_t power_off_value = 0x01;
    return stamp_stm32_write_reg(dev, STAMP_STM32_POWER_OFF_REG, &power_off_value, 1);
}

int stamp_stm32_get_vout_config(stamp_stm32_t *dev, uint8_t *vout)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_VOUT_REG, vout, 1);
}

int stamp_stm32_set_vout_config(stamp_stm32_t *dev, uint8_t vout)
{
    return stamp_stm32_write_reg(dev, STAMP_STM32_VOUT_REG, &vout, 1);
}

int stamp_stm32_get_usb_detect_config(stamp_stm32_t *dev, uint8_t *usb_detect)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_USB_DETECT_REG, usb_detect, 1);
}

int stamp_stm32_get_charge_voltage_config(stamp_stm32_t *dev, uint8_t *voltage)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_CHARGE_VOLTAGE_REG, voltage, 1);
}

int stamp_stm32_set_charge_voltage_config(stamp_stm32_t *dev, uint8_t voltage)
{
    return stamp_stm32_write_reg(dev, STAMP_STM32_CHARGE_VOLTAGE_REG, &voltage, 1);
}

int stamp_stm32_get_charge_current_config(stamp_stm32_t *dev, uint8_t *current)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_CHARGE_CURRENT_REG, current, 1);
}

int stamp_stm32_set_charge_current_config(stamp_stm32_t *dev, uint8_t current)
{
    return stamp_stm32_write_reg(dev, STAMP_STM32_CHARGE_CURRENT_REG, &current, 1);
}

int stamp_stm32_get_charge_enable_config(stamp_stm32_t *dev, uint8_t *charge_status)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_CHARGE_ENABLE_REG, charge_status, 1);
}

int stamp_stm32_set_charge_enable_config(stamp_stm32_t *dev, uint8_t charge_status)
{
    return stamp_stm32_write_reg(dev, STAMP_STM32_CHARGE_ENABLE_REG, &charge_status, 1);
}

int stamp_stm32_set_reset_config(stamp_stm32_t *dev)
{
    uint8_t reset_value = 0x01;
    return stamp_stm32_write_reg(dev, STAMP_STM32_RESET_REG, &reset_value, 1);
}

int stamp_stm32_set_enter_download_mode_config(stamp_stm32_t *dev)
{
    uint8_t download_mode_value = 0x01;
    return stamp_stm32_write_reg(dev, STAMP_STM32_ENTER_DOWNLOAD_MODE_REG, &download_mode_value, 1);
}

int stamp_stm32_get_wake_up_level_config(stamp_stm32_t *dev, uint8_t *wake_up)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_WKUP_LEVEL_REG, wake_up, 1);
}

int stamp_stm32_set_wake_up_level_config(stamp_stm32_t *dev, uint8_t wake_up)
{
    return stamp_stm32_write_reg(dev, STAMP_STM32_WKUP_LEVEL_REG, &wake_up, 1);
}

int stamp_stm32_get_button_status(stamp_stm32_t *dev, uint8_t *button_status)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_BUTTON_REG, button_status, 1);
}

int stamp_stm32_get_sys_status(stamp_stm32_t *dev, uint8_t *sys_status)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_SYS_STATUS_REG, sys_status, 1);
}

int stamp_stm32_get_bootloader_version(stamp_stm32_t *dev, uint8_t *ver)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_BOOTLOADER_VERSION_REG, ver, 1);
}

int stamp_stm32_get_firmware_version(stamp_stm32_t *dev, uint8_t *ver)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_FW_VERSION_REG, ver, 1);
}

int stamp_stm32_get_i2c_addr(stamp_stm32_t *dev, uint8_t *addr)
{
    return stamp_stm32_read_reg(dev, STAMP_STM32_I2C_ADDRESS_REG, addr, 1);
}
