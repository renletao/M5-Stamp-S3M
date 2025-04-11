#include "stamp_case.h"
#include "stamp_stm32.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_random.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "string.h"
#include "esp_timer.h"
#include "soc/rtc.h"

#define TAG "TEST_CASE"

#define DEFAULT_SCAN_LIST_SIZE 15
#define GPIO_INPUT_IO_1        (1)
#define TIMEOUT_PERIOD         (1000000)

esp_timer_handle_t periodic_timer;

esp_timer_create_args_t timer_args = {.callback = NULL, .arg = NULL, .name = "32k_timer"};

void stamp_stm32_test_case_1(stamp_stm32_t *dev)
{
    uint32_t battery_value = 0;
    while (1) {
        if (stamp_stm32_get_battery_voltage(dev, &battery_value) != -1) {
            ESP_LOGI(TAG, "battery_voltage: %lumV\n", battery_value);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read battery voltage error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_2(stamp_stm32_t *dev)
{
    uint16_t battery_adc = 0;
    while (1) {
        if (stamp_stm32_get_battery_adc(dev, &battery_adc) != -1) {
            ESP_LOGI(TAG, "battery_voltage: %u\n", battery_adc);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read battery adc error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

static void log_binary(const char *tag, const char *label, uint8_t value)
{
    char bin[9];
    for (int i = 7; i >= 0; i--) {
        bin[7 - i] = (value >> i) & 1 ? '1' : '0';
    }
    bin[8] = '\0';
    ESP_LOGI(tag, "%s: %s", label, bin);
}

void stamp_stm32_test_case_3(stamp_stm32_t *dev)
{
    uint8_t aw32001_status = 0;
    uint8_t aw32001_fault  = 0;
    while (1) {
        if (stamp_stm32_get_aw32001_status(dev, &aw32001_status) != -1) {
            log_binary(TAG, "aw32001_status", aw32001_status);
        } else {
            ESP_LOGE(TAG, "read aw32001 status error");
            while (1) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        if (stamp_stm32_get_aw32001_fault(dev, &aw32001_fault) != -1) {
            log_binary(TAG, "aw32001_fault", aw32001_fault);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read aw32001 fault error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_4(stamp_stm32_t *dev)
{
    uint8_t count = 10;

    for (uint8_t i = count; i >= 1; i--) {
        ESP_LOGI(TAG, "Shutting down in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    stamp_stm32_set_power_off_config(dev);

    while (1) {
        ESP_LOGE(TAG, "Shutting down error");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void stamp_stm32_test_case_5(stamp_stm32_t *dev)
{
    uint8_t vout_status = 0;
    uint8_t read_byte   = 0;
    while (1) {
        vout_status = 1 - vout_status;
        if (stamp_stm32_set_vout_config(dev, vout_status) != -1) {
            if (stamp_stm32_get_vout_config(dev, &read_byte) != -1) {
                if (read_byte == vout_status) {
                    ESP_LOGI(TAG, "vout_status %d ", vout_status);
                } else {
                    while (1) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "vout read error");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "vout write error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void stamp_stm32_test_case_6(stamp_stm32_t *dev)
{
    
    gpio_config_t io_conf = {.mode         = GPIO_MODE_OUTPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};
    
    io_conf.pin_bit_mask = 1ULL << GPIO_NUM_8;
   
    gpio_config(&io_conf);

    uint8_t usb_status = 0;
    while (1) {
        if (stamp_stm32_get_usb_detect_config(dev, &usb_status) != -1) {
            ESP_LOGI(TAG, "usb_status: %u\n", usb_status);
            if(usb_status){
                gpio_set_level(GPIO_NUM_8, 1);
            }else{
                gpio_set_level(GPIO_NUM_8, 0);
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "read usb status error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_7(stamp_stm32_t *dev)
{
    uint8_t buffer[2][2] = {
        {0b10001011, 0b00001001},  // 4.110V，80mA
        {0b10100011, 0b00001111}   // 4.20V，128mA
    };

    uint8_t charge_voltage = 0;
    uint8_t charge_current = 0;
    uint8_t charge_enable  = 0;

    uint8_t index_flag = 0;

    while (1) {
        ESP_LOGI(TAG, "*****************************************");
        index_flag = 1 - index_flag;
        // 设置电压
        if (stamp_stm32_set_charge_voltage_config(dev, buffer[index_flag][0]) != -1) {
            if (stamp_stm32_get_charge_voltage_config(dev, &charge_voltage) != -1) {
                if (charge_voltage == buffer[index_flag][0]) {
                    log_binary(TAG, "charge_voltage", charge_voltage);
                    if (index_flag == 1) {
                        ESP_LOGI(TAG, "4.20V");
                    } else {
                        ESP_LOGI(TAG, "4.110V");
                    }
                } else {
                    while (1) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "read charge voltage error");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "write charge voltage error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        // 设置电流
        if (stamp_stm32_set_charge_current_config(dev, buffer[index_flag][1]) != -1) {
            if (stamp_stm32_get_charge_current_config(dev, &charge_current) != -1) {
                if (charge_current == buffer[index_flag][1]) {
                    log_binary(TAG, "charge_current", charge_current);
                    if (index_flag == 1) {
                        ESP_LOGI(TAG, "128mA");
                    } else {
                        ESP_LOGI(TAG, "80mA");
                    }
                } else {
                    while (1) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "read charge current error");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "write charge current error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        // 设置使能
        if (stamp_stm32_set_charge_enable_config(dev, 0) != -1) {
            if (stamp_stm32_get_charge_enable_config(dev, &charge_enable) != -1) {
                if (charge_enable == 0) {
                    ESP_LOGI(TAG, "charge enable");
                } else {
                    while (1) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "read charge enable error");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "write charge enable config");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        vTaskDelay(20000 / portTICK_PERIOD_MS);
        // 设置失能
        if (stamp_stm32_set_charge_enable_config(dev, 1) != -1) {
            if (stamp_stm32_get_charge_enable_config(dev, &charge_enable) != -1) {
                if (charge_enable == 1) {
                    ESP_LOGI(TAG, "charge disable");
                } else {
                    while (1) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "read charge enable error");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "write charge enable config");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

void stamp_stm32_test_case_8(stamp_stm32_t *dev)
{
    uint8_t count = 10;

    for (uint8_t i = count; i >= 1; i--) {
        ESP_LOGI(TAG, "Reset ESP32 in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    stamp_stm32_set_reset_config(dev);

    while (1) {
        ESP_LOGE(TAG, "Reset ESP32 error");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void stamp_stm32_test_case_9(stamp_stm32_t *dev)
{
    uint8_t count = 10;

    for (uint8_t i = count; i >= 1; i--) {
        ESP_LOGI(TAG, "ESP32 Enter download in %d seconds...", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    stamp_stm32_set_enter_download_mode_config(dev);

    while (1) {
        ESP_LOGE(TAG, "ESP32 Enter download error");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void stamp_stm32_test_case_10(stamp_stm32_t *dev)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    uint8_t wake_up_level = 1;
    uint8_t read_byte     = 0;

    if (stamp_stm32_set_wake_up_level_config(dev, wake_up_level) != -1) {
        if (stamp_stm32_get_wake_up_level_config(dev, &read_byte) != -1) {
            if (wake_up_level == read_byte) {
                if (wake_up_level) {
                    ESP_LOGI(TAG, "High Level wakeup");
                } else {
                    ESP_LOGI(TAG, "Low Level wakeup");
                }
            } else {
                while (1) {
                    ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "read wake up level error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    } else {
        while (1) {
            ESP_LOGE(TAG, "write wake up level config");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    stamp_stm32_test_case_4(dev);  // 关闭esp32
}

void stamp_stm32_test_case_11(stamp_stm32_t *dev)
{
    uint8_t button_status = 0;
    while (1) {
        if (stamp_stm32_get_button_status(dev, &button_status) != -1) {
            if (button_status) {
                ESP_LOGI(TAG, "button not press");
            } else {
                ESP_LOGI(TAG, "button press");
            }
        } else {
            while (1) {
                ESP_LOGE(TAG, "read button status error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_12(stamp_stm32_t *dev)
{
    uint8_t sys_status = 0;
    while (1) {
        if (stamp_stm32_get_sys_status(dev, &sys_status) != -1) {
            ESP_LOGI(TAG, "sys status: %d\n", sys_status);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read sys status error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_13(stamp_stm32_t *dev)
{
    uint8_t boot_ver = 0;
    while (1) {
        if (stamp_stm32_get_bootloader_version(dev, &boot_ver) != -1) {
            ESP_LOGI(TAG, "boot ver: %d\n", boot_ver);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read boot ver error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}
void stamp_stm32_test_case_14(stamp_stm32_t *dev)
{
    uint8_t fw_ver = 0;
    while (1) {
        if (stamp_stm32_get_firmware_version(dev, &fw_ver) != -1) {
            ESP_LOGI(TAG, "fw ver: %d\n", fw_ver);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read fw ver error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_15(stamp_stm32_t *dev)
{
    uint8_t address = 0;
    while (1) {
        if (stamp_stm32_get_i2c_addr(dev, &address) != -1) {
            ESP_LOGI(TAG, "address: %d\n", address);
        } else {
            while (1) {
                ESP_LOGE(TAG, "read address error");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void stamp_stm32_test_case_16(stamp_stm32_t *dev)
{
    // 0x00~0x07寄存器的值
    uint8_t buffer1[8]     = {0};
    uint32_t battery_value = 0;
    uint16_t battery_adc   = 0;
    uint8_t aw32001_status = 0;
    uint8_t aw32001_fault  = 0;

    // 0x10~0x15寄存器的值
    uint8_t buffer2[6]   = {0};
    uint8_t vout_status  = 0;
    uint8_t usb_status   = 0;
    uint8_t buffer[2][2] = {
        {0b10001011, 0b00001001},  // 4.110V，80mA
        {0b10100011, 0b00001111}   // 4.20V，128mA
    };
    uint8_t charge_voltage = 0;
    uint8_t charge_current = 0;
    uint8_t charge_enable  = 0;
    uint8_t index_flag     = 0;

    // 寄存器0x22
    uint8_t wake_up_level = 0;
    uint8_t read_byte     = 0;

    // 0x30
    uint8_t button_status = 0;

    // 0xFB
    uint8_t sys_status = 0;

    // 0xFC
    uint8_t boot_ver = 0;

    // 0xFE
    uint8_t fw_ver = 0;

    // 0xFF
    uint8_t address = 0;

    while (1) {
        // ESP_LOGI(TAG, ">>>>>>>>>>>>>>>>>>>>>>>***<<<<<<<<<<<<<<<<<<<<");
        // 读0x00~0x07寄存器的值
        if (stamp_stm32_reads_reg(dev, 0x00, buffer1, 8) != -1) {
            battery_value = ((uint32_t)buffer1[0]) | ((uint32_t)buffer1[1] << 8) | ((uint32_t)buffer1[2] << 16) |
                            ((uint32_t)buffer1[3] << 24);
            battery_adc    = ((uint16_t)buffer1[4]) | ((uint16_t)buffer1[5] << 8);
            aw32001_status = buffer1[6];
            aw32001_fault  = buffer1[7];
            // ESP_LOGI(TAG, "battery_voltage: %lumV", battery_value);
            // ESP_LOGI(TAG, "battery_voltage: %u", battery_adc);
            // log_binary(TAG, "aw32001_status", aw32001_status);
            // log_binary(TAG, "aw32001_fault", aw32001_fault);
        } else {
            ESP_LOGE(TAG, "read operation failed starting at address 0x00.");
        }

        uint8_t write_buffer[6] = {0};

        vout_status     = 1 - vout_status;
        write_buffer[1] = vout_status;
        index_flag      = 1 - index_flag;
        write_buffer[2] = 1;
        write_buffer[3] = buffer[index_flag][0];
        write_buffer[4] = buffer[index_flag][1];
        charge_enable   = 1 - charge_enable;
        write_buffer[5] = charge_enable;

        if (stamp_stm32_writes_reg(dev, 0x10, write_buffer, 3) != -1) {
            if (stamp_stm32_reads_reg(dev, 0x10, buffer2, 3) != -1) {
                for (uint8_t i = 0; i < 3; i++) {
                    if (buffer2[i] != write_buffer[i]) {
                        ESP_LOGE(TAG, "Data verification failed: written and read values do not match. addr 0x%02X ",
                                 0x10 + i);
                    }
                }
            } else {
                ESP_LOGE(TAG, "read operation failed starting at address 0x10.");
            }
        } else {
            ESP_LOGE(TAG, "write operation failed starting at address 0x10.");
        }

        wake_up_level = 1 - wake_up_level;
        if (stamp_stm32_set_wake_up_level_config(dev, wake_up_level) != -1) {
            if (stamp_stm32_get_wake_up_level_config(dev, &read_byte) != -1) {
                if (wake_up_level == read_byte) {
                    // if (wake_up_level) {
                    //     ESP_LOGI(TAG, "High Level wakeup");
                    // } else {
                    //     ESP_LOGI(TAG, "Low Level wakeup");
                    // }
                } else {
                    ESP_LOGE(TAG, "Data verification failed: written and read values do not match.");
                }
            } else {
                ESP_LOGE(TAG, "read wake up level error");
            }
        } else {
            ESP_LOGE(TAG, "write wake up level config");
        }

        if (stamp_stm32_get_button_status(dev, &button_status) != -1) {
            if (button_status) {
            } else {
                ESP_LOGE(TAG, "button press status error");
            }
        } else {
            ESP_LOGE(TAG, "read button status error");
        }

        if (stamp_stm32_get_sys_status(dev, &sys_status) != -1) {
        } else {
            ESP_LOGE(TAG, "read sys status error");
        }

        if (stamp_stm32_get_bootloader_version(dev, &boot_ver) != -1) {
            if (boot_ver) {
            } else {
                ESP_LOGE(TAG, "boot ver error");
            }
        } else {
            ESP_LOGE(TAG, "read boot ver error");
        }

        if (stamp_stm32_get_firmware_version(dev, &fw_ver) != -1) {
            if (fw_ver) {
            } else {
                ESP_LOGE(TAG, "fw ver error");
            }
        } else {
            ESP_LOGE(TAG, "read fw ver error");
        }

        if (stamp_stm32_get_i2c_addr(dev, &address) != -1) {
        } else {
            ESP_LOGE(TAG, "read address error");
        }
        // vTaskDelay(10 / portTICK_PERIOD_MS);

        // ESP_LOGI(TAG, ">>>>>>>>>>>>>>>>>>>>>>>***<<<<<<<<<<<<<<<<<<<<");
    }
}
void stamp_stm32_test_case_17(stamp_stm32_t *dev)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_INPUT_IO_1),  // 选择GPIO2
        .mode         = GPIO_MODE_INPUT,            // 设置为输入模式
        .pull_up_en   = GPIO_PULLUP_DISABLE,        // 根据需要启用/禁用上下拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE  // 不使用中断
    };
    gpio_config(&io_conf);
    while (1) {
        int level = gpio_get_level(GPIO_INPUT_IO_1);
        ESP_LOGI("GPIO", "GPIO2 level = %d", level);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// 回调函数每秒触发
void periodic_timer_callback(void *arg)
{
    static uint32_t counter = 0;
    ESP_LOGI("Timer", "Current counter: %lu", ++counter);
}

void stamp_stm32_test_case_18(stamp_stm32_t *dev)
{
    /// 创建周期性定时器
    timer_args.callback = periodic_timer_callback;
    esp_timer_create(&timer_args, &periodic_timer);

    // 启动定时器，周期1秒
    esp_timer_start_periodic(periodic_timer, TIMEOUT_PERIOD);

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 让任务周期性执行
    }
}

static void print_auth_mode(int authmode)
{
    switch (authmode) {
        case WIFI_AUTH_OPEN:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
            break;
        case WIFI_AUTH_OWE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OWE");
            break;
        case WIFI_AUTH_WEP:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
            break;
        case WIFI_AUTH_WPA_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
            break;
        case WIFI_AUTH_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
            break;
        case WIFI_AUTH_WPA_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
            break;
        case WIFI_AUTH_ENTERPRISE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_ENTERPRISE");
            break;
        case WIFI_AUTH_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
            break;
        case WIFI_AUTH_WPA3_ENT_192:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_ENT_192");
            break;
        default:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
            break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_AES_CMAC128:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_AES_CMAC128");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }

    switch (group_cipher) {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
void wifi_scan_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_scan(void)
{
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);
    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG, "=======================================================================");
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    ESP_LOGI(TAG, "The best is %s(%d)", ap_info[0].ssid, ap_info[0].rssi);
    for (int i = 0; i < number; i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        print_auth_mode(ap_info[i].authmode);
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
    }
    ESP_LOGI(TAG, "=======================================================================");
}

void stamp_stm32_test_case_19(stamp_stm32_t *dev)
{
    wifi_scan_init();
    wifi_scan();
}

void stamp_stm32_test_case_20(stamp_stm32_t *dev)
{
    static const gpio_num_t led_pins[] = {GPIO_NUM_2, GPIO_NUM_3,  GPIO_NUM_4,  GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7,
                                          GPIO_NUM_8, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9};
    static const int led_count         = sizeof(led_pins) / sizeof(led_pins[0]);
    static const int delay_time_ms     = 600;  // 毫秒

    // 配置所有引脚为输出模式
    gpio_config_t io_conf = {.mode         = GPIO_MODE_OUTPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};

    for (int i = 0; i < led_count; i++) {
        io_conf.pin_bit_mask = 1ULL << led_pins[i];
        gpio_config(&io_conf);
        gpio_set_level(led_pins[i], 0);  // 初始为低电平（熄灭）
    }

    // 主循环：执行正反向流水灯效果
    while (1) {
        // 正向流水灯（左到右）
        for (int i = 0; i < led_count; i++) {
            gpio_set_level(led_pins[i], 1);  // 点亮
            vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
            gpio_set_level(led_pins[i], 0);  // 熄灭
        }

        // 反向流水灯（右到左）
        for (int i = led_count - 1; i >= 0; i--) {
            gpio_set_level(led_pins[i], 1);
            vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
            gpio_set_level(led_pins[i], 0);
        }

        // TODO: 如果你后续还要加按键，button1.tick(); 等类似功能可在此插入
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}