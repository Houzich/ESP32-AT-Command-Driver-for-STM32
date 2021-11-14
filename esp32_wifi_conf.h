/**
 ******************************************************************************
 * @file    esp32_wifi_conf.h
 * @author
 * @version
 * @date
 * @brief
 ******************************************************************************
 */
#ifndef ESP32_WIFI_CONF_H
#define ESP32_WIFI_CONF_H
#define IPC_MASTER

#define ESP32_WIFI_USE_UART_INTERRUPT                  1
#ifdef HAS_RTOS
#define ESP32_WIFI_USE_CMSIS_OS                        1
#endif
#ifndef ESP32_WIFI_USE_CMSIS_OS
#define ESP32_WIFI_USE_CMSIS_OS                        (0)
#endif

#define ESP32_WIFI_DATA_SIZE                           (1024*2)

#define ESP32_WIFI_MAX_LISTED_AP						20
#define ESP32_WIFI_MAX_DETECTED_AP						20


#define ESP32_WIFI_TIMEOUT                             (10000)
#define ESP32_WIFI_MAX_SOCKET_NBR                      (8)


#define ESP32_WIFI_MAX_SSID_NAME_SIZE                  32
#define ESP32_WIFI_MAX_PSWD_NAME_SIZE                  64

#define ESP32_WIFI_PRODUCT_NAME_SIZE                   32
#define ESP32_WIFI_PRODUCT_ID_SIZE                     32



#ifdef __cplusplus
 extern "C" {
#endif  

   
#ifdef __cplusplus
}
#endif
#endif /* ESP32_WIFI_CONF_H */

/*****************************END OF FILE****/
