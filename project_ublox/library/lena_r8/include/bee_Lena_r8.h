/**
 * @file bee_Lena_r8.h
 * @author nguyen__viet_hoang
 * @date 21 September 2023
 * @brief module SIM LENA-R8 with API init, connect MQTT broker, publish data sensor
 */
#ifndef __LENA_R8_H__
#define __LENA_R8_H__

#define BEE_MQTT_BROKER_URL "\"61.28.238.97\""
#define BEE_BROKER_PORT "1993"
#define BEE_USER_NAME "\"VBeeHome\""
#define BEE_USER_PASSWORD "\"123abcA@!\""
#define BEE_MQTT_CLIENT_ID "\"device:029f567e-767a-4250-bd7b-6dfa6191f38b\""

#define BEE_TIME_PUBLISH_DATA_RS485 10000
#define BEE_TIME_PUBLISH_DATA_KEEP_ALIVE 60000
#define BEE_QUEUE_LENGTH 30

#define BEE_LENGTH_AT_COMMAND 250
#define BEE_LENGTH_AT_COMMAND_RS485 900
#define BEE_LENGTH_MESSAGE_RESPONSE 250

#define TICK_TIME_TO_SUBSCRIBE_FULL_MESSAGE 10
#define BEE_COUNT_MAX_CONNECTED_FAIL 20

void mqtt_vLena_r8_start();

#endif

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/