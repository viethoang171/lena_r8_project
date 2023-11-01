/*****************************************************************************
 *
 * @file 	bee_rs485.c
 * @author 	tuha
 * @date 	5 Sep 2023
 * @brief	module for project rs485 commuication
 *
 ***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_mac.h"

#include "bee_cJSON.h"
#include "bee_rs485.h"

#define TAG "RS485"

data_3pha_t data_3pha;
bool check_data_flag = 0;

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

static uint16_t combine_2Bytes_unsigned(uint8_t highByte, uint8_t lowByte)
{
    if (highByte == 0xff && lowByte == 0xff)
    {
        return 0;
    }
    return ((uint16_t)highByte << 8) | lowByte;
}

static uint32_t combine_4Bytes_unsingned(uint8_t highByte1, uint8_t lowByte1, uint8_t highByte2, uint8_t lowByte2)
{
    if (highByte1 == 0xff && lowByte1 == 0xff && highByte2 == 0xff && lowByte2 == 0xff)
    {
        return 0;
    }
    return ((uint32_t)highByte1 << 24) | (lowByte1 << 16) | (highByte2 << 8) | lowByte2;
}

static int32_t combine_4Bytes_singned(uint8_t highByte1, uint8_t lowByte1, uint8_t highByte2, uint8_t lowByte2)
{
    int32_t result = ((int32_t)highByte1 << 24) | lowByte1 << 16 | highByte2 << 8 | lowByte2;
    if (highByte1 == 0x7f && lowByte1 == 0xff && highByte2 == 0xff && lowByte2 == 0xff)
    {
        return 0;
    }
    return result;
}

static uint16_t MODBUS_CRC16(uint8_t *buf, uint16_t len)
{
    static const uint16_t table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

    uint8_t xor = 0;
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        xor = (*buf++) ^ crc;
        crc >>= 8;
        crc ^= table[xor];
    }
    return crc;
}

static void read_data_holding_registers(uint8_t *dtmp_buf)
{
    data_3pha.voltage3pha = combine_4Bytes_unsingned(dtmp_buf[3], dtmp_buf[4], dtmp_buf[5], dtmp_buf[6]);
    data_3pha.voltageL1 = combine_4Bytes_unsingned(dtmp_buf[7], dtmp_buf[8], dtmp_buf[9], dtmp_buf[10]);
    data_3pha.voltageL2 = combine_4Bytes_unsingned(dtmp_buf[11], dtmp_buf[12], dtmp_buf[13], dtmp_buf[14]);
    data_3pha.voltageL3 = combine_4Bytes_unsingned(dtmp_buf[15], dtmp_buf[16], dtmp_buf[17], dtmp_buf[18]);

    data_3pha.voltageL1L2 = combine_4Bytes_unsingned(dtmp_buf[19], dtmp_buf[20], dtmp_buf[21], dtmp_buf[22]);
    data_3pha.voltageL3L2 = combine_4Bytes_unsingned(dtmp_buf[23], dtmp_buf[24], dtmp_buf[25], dtmp_buf[26]);
    data_3pha.voltageL1L3 = combine_4Bytes_unsingned(dtmp_buf[27], dtmp_buf[28], dtmp_buf[29], dtmp_buf[30]);

    data_3pha.current3pha = combine_4Bytes_unsingned(dtmp_buf[31], dtmp_buf[32], dtmp_buf[33], dtmp_buf[34]);
    data_3pha.currentL1 = combine_4Bytes_unsingned(dtmp_buf[35], dtmp_buf[36], dtmp_buf[37], dtmp_buf[38]);
    data_3pha.currentL2 = combine_4Bytes_unsingned(dtmp_buf[39], dtmp_buf[40], dtmp_buf[41], dtmp_buf[42]);
    data_3pha.currentL3 = combine_4Bytes_unsingned(dtmp_buf[43], dtmp_buf[44], dtmp_buf[45], dtmp_buf[46]);
    data_3pha.currentN = combine_4Bytes_unsingned(dtmp_buf[47], dtmp_buf[48], dtmp_buf[49], dtmp_buf[50]);

    data_3pha.actpower3pha = combine_4Bytes_singned(dtmp_buf[55], dtmp_buf[56], dtmp_buf[57], dtmp_buf[58]);
    data_3pha.actpowerL1 = combine_4Bytes_singned(dtmp_buf[59], dtmp_buf[60], dtmp_buf[61], dtmp_buf[62]);
    data_3pha.actpowerL2 = combine_4Bytes_singned(dtmp_buf[63], dtmp_buf[64], dtmp_buf[65], dtmp_buf[66]);
    data_3pha.actpowerL3 = combine_4Bytes_singned(dtmp_buf[67], dtmp_buf[68], dtmp_buf[69], dtmp_buf[70]);

    data_3pha.ractpower3pha = combine_4Bytes_singned(dtmp_buf[71], dtmp_buf[72], dtmp_buf[73], dtmp_buf[74]);
    data_3pha.ractpowerL1 = combine_4Bytes_singned(dtmp_buf[75], dtmp_buf[76], dtmp_buf[77], dtmp_buf[78]);
    data_3pha.ractpowerL2 = combine_4Bytes_singned(dtmp_buf[79], dtmp_buf[80], dtmp_buf[81], dtmp_buf[82]);
    data_3pha.ractpowerL3 = combine_4Bytes_singned(dtmp_buf[83], dtmp_buf[84], dtmp_buf[85], dtmp_buf[86]);

    data_3pha.aprtpower3pha = combine_4Bytes_singned(dtmp_buf[87], dtmp_buf[88], dtmp_buf[89], dtmp_buf[90]);
    data_3pha.aprtpowerL1 = combine_4Bytes_singned(dtmp_buf[91], dtmp_buf[92], dtmp_buf[93], dtmp_buf[94]);
    data_3pha.aprtpowerL2 = combine_4Bytes_singned(dtmp_buf[95], dtmp_buf[96], dtmp_buf[97], dtmp_buf[98]);
    data_3pha.aprtpowerL3 = combine_4Bytes_singned(dtmp_buf[99], dtmp_buf[100], dtmp_buf[101], dtmp_buf[102]);

    data_3pha.Frequency = combine_2Bytes_unsigned(dtmp_buf[103], dtmp_buf[104]);
}

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

void rs485_init()
{
    const int uart_num = UART_PORT_2;
    uart_config_t uart_config =
        {
            .baud_rate = BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
        };

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "Start RS485 application and configure UART.");

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUFF_SIZE * 2, 0, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, RTS_PIN, CTS_PIN));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, RX_READ_TOUT));
}

void TX(const int port, const char *str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length)
    {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

void RX_task(void *pvParameters)
{
    uint8_t *dtmp = (uint8_t *)malloc(BUFF_SIZE);
    for (;;)
    {
        int len = uart_read_bytes(UART_PORT_2, dtmp, BUFF_SIZE, PACKET_READ_TICS);
        if (len > 0)
        {
            // Tính toán CRC16 cho dữ liệu gốc
            uint16_t crc_caculated = MODBUS_CRC16(dtmp, len - 2);
            uint16_t crc_received = combine_2Bytes_unsigned(dtmp[len - 1], dtmp[len - 2]);
            if (crc_caculated == crc_received)
            {
                // In chuỗi nhận được theo dạng hexa
                printf("str RX: ");
                for (int i = 0; i < len; i++)
                {
                    printf("%02X ", dtmp[i]);
                }
                printf("\n");
                ESP_LOGI(TAG, "Slave address: %02X", dtmp[0]);
                if (dtmp[1] == 0x03)
                {
                    check_data_flag = 1; // have new data
                    ESP_LOGI(TAG, "Funtion: Read holding registers");
                    ESP_LOGI(TAG, "Byte count: %d", dtmp[2]);
                    read_data_holding_registers(dtmp);

                    uart_flush(UART_PORT_2);
                }
            }
        }
        else
        {
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_2, 10));
        }
    }
}

char *read_holding_registers(uint8_t slave_addr)
{
    uint8_t tx_str[8];

    tx_str[0] = slave_addr;
    tx_str[1] = 0x03;
    tx_str[2] = 0x5b;
    tx_str[3] = 0x00;
    tx_str[4] = 0x00;
    tx_str[5] = 0x33;

    // Tính CRC của chuỗi tx_str.
    uint16_t crc = MODBUS_CRC16(tx_str, 6);

    // Thêm CRC vào chuỗi tx_str.
    tx_str[6] = crc & 0xFF;        // Byte thấp của CRC
    tx_str[7] = (crc >> 8) & 0xFF; // Byte cao của CRC

    // Sao chép chuỗi tx_str vào một vùng nhớ mới.
    char *new_tx_str = (char *)malloc(sizeof(tx_str) + 1);

    if (new_tx_str == NULL)
    {
        // Xử lý lỗi nếu không thể cấp phát bộ nhớ.
        return NULL;
    }

    memcpy(new_tx_str, tx_str, sizeof(tx_str));
    new_tx_str[sizeof(tx_str)] = '\0'; // Đặt ký tự null ở cuối chuỗi.

    return new_tx_str;
}

uint8_t trans_code = 0;
char *pack_json_3pha_data(void)
{
    char mac_str[13];
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    cJSON *json_3pha_data = cJSON_CreateObject();
    cJSON_AddStringToObject(json_3pha_data, "thing_token", mac_str);
    cJSON_AddStringToObject(json_3pha_data, "cmd_name", "Bee.Nag_data");
    cJSON_AddStringToObject(json_3pha_data, "object_type", "Bee.Nag_vrf");

    cJSON *json_values = cJSON_AddObjectToObject(json_3pha_data, "values");
    cJSON_AddNumberToObject(json_values, "voltage3pha", data_3pha.voltage3pha);
    cJSON_AddNumberToObject(json_values, "voltageL1", data_3pha.voltageL1);
    cJSON_AddNumberToObject(json_values, "voltageL2", data_3pha.voltageL2);
    cJSON_AddNumberToObject(json_values, "voltageL3", data_3pha.voltageL3);

    cJSON_AddNumberToObject(json_values, "voltageL1L2", data_3pha.voltageL1L2);
    cJSON_AddNumberToObject(json_values, "voltageL3L2", data_3pha.voltageL3L2);
    cJSON_AddNumberToObject(json_values, "voltageL1L3", data_3pha.voltageL1L3);

    cJSON_AddNumberToObject(json_values, "current3pha", data_3pha.current3pha);
    cJSON_AddNumberToObject(json_values, "currentL1", data_3pha.currentL1);
    cJSON_AddNumberToObject(json_values, "currentL2", data_3pha.currentL2);
    cJSON_AddNumberToObject(json_values, "currentL3", data_3pha.currentL3);
    cJSON_AddNumberToObject(json_values, "currentN", data_3pha.currentN);

    cJSON_AddNumberToObject(json_values, "actpower3pha", data_3pha.actpower3pha);
    cJSON_AddNumberToObject(json_values, "actpowerL1", data_3pha.actpowerL1);
    cJSON_AddNumberToObject(json_values, "actpowerL2", data_3pha.actpowerL2);
    cJSON_AddNumberToObject(json_values, "actpowerL3", data_3pha.actpowerL3);

    cJSON_AddNumberToObject(json_values, "ractpower3pha", data_3pha.ractpower3pha);
    cJSON_AddNumberToObject(json_values, "ractpowerL1", data_3pha.ractpowerL1);
    cJSON_AddNumberToObject(json_values, "ractpowerL2", data_3pha.ractpowerL2);
    cJSON_AddNumberToObject(json_values, "ractpowerL3", data_3pha.ractpowerL3);

    cJSON_AddNumberToObject(json_values, "aprtpower3pha", data_3pha.aprtpower3pha);
    cJSON_AddNumberToObject(json_values, "aprtpowerL1", data_3pha.aprtpowerL1);
    cJSON_AddNumberToObject(json_values, "aprtpowerL2", data_3pha.aprtpowerL2);
    cJSON_AddNumberToObject(json_values, "aprtpowerL3", data_3pha.aprtpowerL3);

    cJSON_AddNumberToObject(json_values, "Frequency", data_3pha.Frequency);

    cJSON_AddNumberToObject(json_3pha_data, "trans_code", trans_code++);

    char *json_str = cJSON_Print(json_3pha_data);

    // Giải phóng bộ nhớ cho JSON object
    cJSON_Delete(json_3pha_data);

    return json_str;
}

static void TX_task(void *pvParameters)
{
    static TickType_t last_time_transmit = 0;
    char *str_tx = read_holding_registers(0x01);
    for (;;)
    {
        if (xTaskGetTickCount() - last_time_transmit > pdMS_TO_TICKS(BEE_TIME_TRANSMIT_DATA_RS485))
        {
            if (str_tx != NULL)
            {
                printf("str TX: ");
                for (int i = 0; i < 8; i++)
                {
                    printf("%02X ", (unsigned char)str_tx[i]);
                }
                printf("\n");
                TX(2, str_tx, 8);
            }
            last_time_transmit = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void rs485_start()
{
    xTaskCreate(RX_task, "RX_task", RX_TASK_STACK_SIZE * 2, NULL, 31, NULL);
    xTaskCreate(TX_task, "TX_task", 4096 * 2, NULL, 30, NULL);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/