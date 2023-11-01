/**
 * @file bee_Lena_r8.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API, init, connect MQTT broker, configure parameters and publish/subscribe data sensor
 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_mac.h"
#include <stdio.h>
#include <string.h>

#include "bee_cJSON.h"
#include "bee_Uart.h"
#include "bee_rs485.h"
#include "bee_Lena_r8.h"

// QueueHandle_t queue_message_response; // queue for task subscribe

extern bool check_data_flag;

static char BEE_TOPIC_SUBSCRIBE[100];
static char BEE_TOPIC_PUBLISH[100];

static uint8_t u8Mac_address[6] = {0xb8, 0xd6, 0x1a, 0x6b, 0x2d, 0xe8};
static char mac_address[13];

static char message_publish[BEE_LENGTH_AT_COMMAND];
// static char message_publish_content_for_publish_mqtt_binary[BEE_LENGTH_AT_COMMAND];
static char message_publish_content_for_publish_mqtt_binary_rs485[BEE_LENGTH_AT_COMMAND_RS485];

static void lena_vConfigure_credential()
{
    char command_AT[BEE_LENGTH_AT_COMMAND] = {};

    // config client Id
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=0,%s\r\n", BEE_MQTT_CLIENT_ID);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(2000));

    // config IP broker and port
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=3,%s,%s\r\n", BEE_MQTT_BROKER_URL, BEE_BROKER_PORT);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(2000));

    // config broker user name and password
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=4,%s,%s\r\n", BEE_USER_NAME, BEE_USER_PASSWORD);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(2000));
}

static void lena_vConnect_mqtt_broker()
{
    char command_AT[BEE_LENGTH_AT_COMMAND] = {};

    // Query MQTT's credentials
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT?\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(5000));

    // CGACT
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+CGACT=1,1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(2000));

    // AT connect
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(5000));

    // create AT command to subscribe topic on broker
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=4,0,%s\r\n", BEE_TOPIC_SUBSCRIBE);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(2000));
}

static void lena_vPublish_data_rs485()
{
    // Create AT command to publish json message rs485
    char *message_json_rs485 = (char *)calloc(BEE_LENGTH_AT_COMMAND_RS485, sizeof(char));
    message_json_rs485 = pack_json_3pha_data();

    snprintf(message_publish, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=9,0,0,%s,%d\r\n", BEE_TOPIC_PUBLISH, strlen(message_json_rs485) + 1);
    snprintf(message_publish_content_for_publish_mqtt_binary_rs485, BEE_LENGTH_AT_COMMAND_RS485, "%s\r\n", message_json_rs485);

    // Send AT command
    uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));

    // Send content to publish
    uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary_rs485, strlen(message_publish_content_for_publish_mqtt_binary_rs485) + 1);
}

static void mqtt_vPublish_task()
{
    static TickType_t last_time_publish = 0;
    lena_vConfigure_credential();
    lena_vConnect_mqtt_broker();

    for (;;)
    {
        if (xTaskGetTickCount() - last_time_publish >= pdMS_TO_TICKS(BEE_TIME_PUBLISH_DATA_RS485))
        {
            if (check_data_flag == 1) // new data
            {
                lena_vPublish_data_rs485();
                check_data_flag = 0; // reset data's status
            }
            last_time_publish = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#if (0) // parse json for task subscribe
static void mqtt_vParse_json(char *rxBuffer)
{
    cJSON *root = cJSON_Parse(rxBuffer);
    if (root != NULL)
    {
        char *device_id;
        char *cmd_name;
        char *object_type;

        device_id = cJSON_GetObjectItemCaseSensitive(root, "thing_token")->valuestring;
        cmd_name = cJSON_GetObjectItemCaseSensitive(root, "cmd_name")->valuestring;

        cJSON_Delete(root);
    }
}
#endif

#if (0) // Add task subscribe
static void mqtt_vSubscribe_command_server_task()
{

    char list_message_subscribe[BEE_LENGTH_AT_COMMAND] = {};
    char *dtmp = (char *)malloc(200);
    char command_AT[BEE_LENGTH_AT_COMMAND] = {};
    snprintf(command_AT, 16, "AT+UMQTTC=6,1\r\n");
    uart_event_t uart_event;

    for (;;)
    {
        // If broker publish message for module
        uart_read_bytes(EX_UART_NUM, list_message_subscribe, BEE_LENGTH_AT_COMMAND, (TickType_t)0);

        if (strstr(list_message_subscribe, "+UUMQTTC: 6") != NULL)
        {
            uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

            if (xQueueReceive(queue_message_response, (void *)&uart_event, (TickType_t)portMAX_DELAY))
            {
                // Read message AT command from broker
                uart_read_bytes(EX_UART_NUM, list_message_subscribe, BEE_LENGTH_AT_COMMAND, (TickType_t)TICK_TIME_TO_SUBSCRIBE_FULL_MESSAGE);

                // Filter message json
                list_message_subscribe[strlen(list_message_subscribe) - 9] = '\0';
                char *message_json_subscribe;
                message_json_subscribe = strstr(list_message_subscribe, "{");

                // parse json
                mqtt_vParse_json(message_json_subscribe);

                // Publish message json's data sensor
                snprintf(message_publish, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=9,0,0,%s,%d\r\n", BEE_TOPIC_PUBLISH, strlen(message_publish_content_for_publish_mqtt_binary));
                uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
                uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, BEE_LENGTH_AT_COMMAND);

                for (uint16_t u8Index = 0; u8Index < BEE_LENGTH_AT_COMMAND; u8Index++)
                {
                    list_message_subscribe[u8Index] = '\0';
                    message_publish_content_for_publish_mqtt_binary[u8Index] = '\0';
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(dtmp);
    dtmp = NULL;
}
#endif

void mqtt_vLena_r8_start()
{
    // Get mac address
    esp_efuse_mac_get_default(u8Mac_address);
    snprintf(mac_address, sizeof(mac_address), "%02x%02x%02x%02x%02x%02x", u8Mac_address[0], u8Mac_address[1], u8Mac_address[2], u8Mac_address[3], u8Mac_address[4], u8Mac_address[5]);
    snprintf(BEE_TOPIC_PUBLISH, sizeof(BEE_TOPIC_PUBLISH), "\"VB/DMP/VBEEON/BEE/SMH/%s/telemetry\"", mac_address);
    snprintf(BEE_TOPIC_SUBSCRIBE, sizeof(BEE_TOPIC_SUBSCRIBE), "\"VB/DMP/VBEEON/BEE/SMH/%s/command\"", mac_address);

    xTaskCreate(mqtt_vPublish_task, "mqtt_vPublish_task", 1024 * 3, NULL, 3, NULL);

    // xTaskCreate(mqtt_vSubscribe_command_server_task, "mqtt_vSubscribe_command_server_task", 1024 * 3, NULL, 4, NULL);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/