/**
 * @file bee_Lena_r8.c
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API, init, connect MQTT broker, configure parameters and publish/subscribe data sensor
 */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

#include "bee_cJSON.h"
#include "bee_Sht3x.h"
#include "bee_Uart.h"
#include "bee_Led.h"
#include "bee_rs485.h"
#include "bee_Lena_r8.h"

QueueHandle_t queue_message_response;
TaskHandle_t xHandle_fade_mode;
TaskHandle_t xHandle_smooth_mode;
TaskHandle_t xHandle_mix_fade_mode;

float f_Sht3x_temp;
float f_Sht3x_humi;

static uint8_t u8Flag_exist_task_fade_mode = 0;
static uint8_t u8Flag_exist_task_smooth_mode = 0;
static uint8_t u8Flag_exist_task_mix_fade_mode = 0;
static uint8_t u8Trans_code = 0;
static uint8_t u8Mac_address[6] = {0xb8, 0xd6, 0x1a, 0x6b, 0x2d, 0xe8};
static char mac_address[13];

static char message_publish[BEE_LENGTH_AT_COMMAND];
static char message_publish_content_for_publish_mqtt_binary[BEE_LENGTH_AT_COMMAND];
static char message_publish_content_for_publish_mqtt_binary_rs485[BEE_LENGTH_AT_COMMAND_RS485];

static void mqtt_vCreate_content_message_json_data(uint8_t u8Flag_temp_humi, float f_Value)
{
    char *message_json_publish;
    u8Trans_code++;
    cJSON *root;

    root = cJSON_CreateObject();
    if (root != NULL)
    {
        cJSON_AddItemToObject(root, "thing_token", cJSON_CreateString(mac_address));
        cJSON_AddItemToObject(root, "cmd_name", cJSON_CreateString("Bee.data"));

        if (u8Flag_temp_humi == FLAG_TEMPERATURE)
        {
            cJSON_AddItemToObject(root, "object_type", cJSON_CreateString("temperature"));
        }
        else
        {
            cJSON_AddItemToObject(root, "object_type", cJSON_CreateString("humidity"));
        }

        cJSON_AddItemToObject(root, "values", cJSON_CreateNumber(f_Value));
        cJSON_AddItemToObject(root, "trans_code", cJSON_CreateNumber(u8Trans_code));
    }
    message_json_publish = cJSON_Print(root);
    cJSON_Delete(root);

    if (message_json_publish != NULL)
    {
        snprintf(message_publish, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=9,0,0,%s,%d\r\n", BEE_TOPIC_PUBLISH, strlen(message_json_publish) + 1);
        snprintf(message_publish_content_for_publish_mqtt_binary, BEE_LENGTH_AT_COMMAND, "%s\r\n", message_json_publish);
    }
}

static void lena_vConfigure_credential()
{
    char command_AT[BEE_LENGTH_AT_COMMAND] = {};

    // config client Id
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=0,%s\r\n", BEE_MQTT_CLIENT_ID);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config IP broker and port
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=3,%s,%s\r\n", BEE_MQTT_BROKER_URL, BEE_BROKER_PORT);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));

    // config broker user name and password
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTT=4,%s,%s\r\n", BEE_USER_NAME, BEE_USER_PASSWORD);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
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

    // AT connect
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=1\r\n");
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
    vTaskDelay(pdMS_TO_TICKS(5000));

    // create AT command to subscribe topic on broker
    snprintf(command_AT, BEE_LENGTH_AT_COMMAND, "AT+UMQTTC=4,0,%s\r\n", BEE_TOPIC_SUBSCRIBE);
    uart_write_bytes(EX_UART_NUM, command_AT, strlen(command_AT));
}

static void lena_vPublish_data_sensor()
{
    // publish temperature value
    mqtt_vCreate_content_message_json_data(FLAG_TEMPERATURE, f_Sht3x_temp);
    uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
    uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, strlen(message_publish_content_for_publish_mqtt_binary) + 1);

    // publish humidity value
    mqtt_vCreate_content_message_json_data(FLAG_HUMIDITY, f_Sht3x_humi);
    uart_write_bytes(EX_UART_NUM, message_publish, strlen(message_publish));
    uart_write_bytes(EX_UART_NUM, message_publish_content_for_publish_mqtt_binary, strlen(message_publish_content_for_publish_mqtt_binary) + 1);
}

static void lena_vPublish_data_rs485()
{
    // Create AT command to publish json message rs485
    char *message_json_rs485 = pack_3pha_data();

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
        if (xTaskGetTickCount() - last_time_publish > pdMS_TO_TICKS(BEE_TIME_PUBLISH_DATA_SENSOR))
        {
            lena_vPublish_data_sensor();
            last_time_publish = xTaskGetTickCount();
        }
        if (xTaskGetTickCount() - last_time_publish > pdMS_TO_TICKS(BEE_TIME_PUBLISH_DATA_RS485))
        {
            lena_vPublish_data_rs485();
            last_time_publish = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

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

        if (device_id != NULL && cmd_name != NULL)
        {
            if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.data") == 0)
            {
                object_type = cJSON_GetObjectItemCaseSensitive(root, "object_type")->valuestring;
                if (object_type != NULL)
                {
                    if (strcmp(object_type, OBJECT_TYPE_TEMP) == 0)
                    {
                        mqtt_vCreate_content_message_json_data(FLAG_TEMPERATURE, f_Sht3x_temp);
                    }
                    else if (strcmp(object_type, OBJECT_TYPE_HUM) == 0)
                    {
                        mqtt_vCreate_content_message_json_data(FLAG_HUMIDITY, f_Sht3x_humi);
                    }
                }
            }
            else if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.control_led_fade_mode") == 0)
            {
                if (u8Flag_exist_task_smooth_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_smooth_mode);
                }
                if (u8Flag_exist_task_mix_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_mix_fade_mode);
                }

                if (u8Flag_exist_task_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskResume(xHandle_fade_mode);
                }
                else
                {
                    u8Flag_exist_task_fade_mode = BEE_EXIST_MODE_LED;
                    xTaskCreate(ledc_fade_mode_task, "ledc_fade_mode_task", 1024, NULL, 1, &xHandle_fade_mode);
                }
            }
            else if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.control_led_smooth_mode") == 0)
            {
                if (u8Flag_exist_task_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_fade_mode);
                }
                if (u8Flag_exist_task_mix_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_mix_fade_mode);
                }

                if (u8Flag_exist_task_smooth_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskResume(xHandle_smooth_mode);
                }
                else
                {
                    u8Flag_exist_task_smooth_mode = BEE_EXIST_MODE_LED;
                    xTaskCreate(ledc_smooth_mode_task, "ledc_smooth_mode_task", 1024, NULL, 1, &xHandle_smooth_mode);
                }
            }
            else if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.control_led_mix_fade_mode") == 0)
            {
                if (u8Flag_exist_task_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_fade_mode);
                }
                if (u8Flag_exist_task_smooth_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskSuspend(xHandle_smooth_mode);
                }

                if (u8Flag_exist_task_mix_fade_mode == BEE_EXIST_MODE_LED)
                {
                    vTaskResume(xHandle_mix_fade_mode);
                }
                else
                {
                    u8Flag_exist_task_mix_fade_mode = BEE_EXIST_MODE_LED;
                    xTaskCreate(ledc_mix_fade_mode_task, "ledc_mix_fade_mode_task", 1024, NULL, 1, &xHandle_mix_fade_mode);
                }
            }

            else if (strcmp(device_id, mac_address) == 0 && strcmp(cmd_name, "Bee.control_rgb") == 0)
            {
                cJSON *values = cJSON_GetObjectItem(root, "values");
                if (values != NULL)
                {
                    cJSON *red = cJSON_GetObjectItem(values, "red");
                    cJSON *green = cJSON_GetObjectItem(values, "green");
                    cJSON *blue = cJSON_GetObjectItem(values, "blue");

                    if (red != NULL && green != NULL && blue != NULL)
                    {
                        // Lấy các giá trị red, green, blue
                        uint8_t red_value = red->valueint;
                        uint8_t green_value = green->valueint;
                        uint8_t blue_value = blue->valueint;

                        // Pause other modes
                        if (u8Flag_exist_task_fade_mode)
                        {
                            vTaskSuspend(xHandle_fade_mode);
                        }
                        if (u8Flag_exist_task_smooth_mode)
                        {
                            vTaskSuspend(xHandle_smooth_mode);
                        }
                        if (u8Flag_exist_task_mix_fade_mode)
                        {
                            vTaskSuspend(xHandle_mix_fade_mode);
                        }

                        ledc_set_duty_rgb(red_value, green_value, blue_value);
                    }
                }
            }
        }
        cJSON_Delete(root);
    }
}

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

void mqtt_vLena_r8_start()
{
    snprintf(mac_address, sizeof(mac_address), "%02x%02x%02x%02x%02x%02x", u8Mac_address[0], u8Mac_address[1], u8Mac_address[2], u8Mac_address[3], u8Mac_address[4], u8Mac_address[5]);
    ledc_init_hardware();

    xTaskCreate(mqtt_vPublish_task, "mqtt_vPublish_task", 1024 * 3, NULL, 4, NULL);
    xTaskCreate(mqtt_vSubscribe_command_server_task, "mqtt_vSubscribe_command_server_task", 1024 * 3, NULL, 5, NULL);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/