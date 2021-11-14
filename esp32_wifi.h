/**
 ******************************************************************************
 * @file    esp32_wifi.h
 * @author
 * @brief   Header for esp32_wifi.c module
 ******************************************************************************
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ESP32_WIFI_H
#define ESP32_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
//#include "net_types.h"

#include "esp32_wifi_conf.h"

#if ESP32_WIFI_USE_CMSIS_OS

#include "cmsis_os.h"

#endif

/*******************************************************************************
 * ESP32 Wi-Fi Module Defines
 ******************************************************************************/
// common defines
/**
 * @brief Wi-Fi mode
 */
enum {
	ESP32_SOFTAP, /**< wifi softap mode. */
	ESP32_STATION, /**< wifi station mode. */
};
typedef uint8_t mc_wifi_if_t; /**< wifi mode. */

/**
 * @brief Wi-Fi scan mode
 */
enum {
	ESP32_SCAN_PASSIVE = 0, /**< wifi passive scan mode. */
	ESP32_SCAN_ACTIVE = 1 /**< wifi active scan mode. */
};
typedef uint8_t mc_wifi_scan_mode_t; /**< wifi scan mode. */

/*******************************************************************************
 * STM32Cube Driver API Defines
 ******************************************************************************/
// status code
#define ESP32_WIFI_STATUS_OK           				(0)     /**< status code success. */
#define ESP32_WIFI_STATUS_ERROR        				(-1)    /**< status code common error. */
#define ESP32_WIFI_STATUS_TIMEOUT      				(-2)    /**< status code timeout. */
#define ESP32_WIFI_STATUS_IO_ERROR     				(-3)    /**< status code I/O error. */
#define ESP32_WIFI_STATUS_PARAM_ERROR  				(-4)    /**< status code bad argument error. */
#define ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET	(-5)
#define ESP32_WIFI_STATUS_MODULE_CRASH   			(-6)
#define ESP32_WIFI_Status_t            int32_t /**< status code. */

// macro
#define ESP32_WIFI_MAC_SIZE            (6)     /**< max length of MAC address. */
#define ESP32_WIFI_SCAN_BUF_SIZE       (2000)  /**< max size of scan buffer. */

//#define MIN(a, b)  ( ((a) < (b)) ? (a) : (b))  /**< helper function: get minimum. */

/* Security settings for wifi network */
typedef enum {
	ESP32_WIFI_SEC_OPEN = 0x00, /*!< Wifi is open */
	ESP32_WIFI_SEC_WEP = 0x01, /*!< Wired Equivalent Privacy option for wifi security. \note This mode can't be used when setting up ES_WIFI wifi */
	ESP32_WIFI_SEC_WPA_PSK = 0x02, /*!< Wi-Fi Protected Access */
	ESP32_WIFI_SEC_WPA2_PSK = 0x03, /*!< Wi-Fi Protected Access 2 */
	ESP32_WIFI_SEC_WPA_WPA2_PSK = 0x04, /*!< Wi-Fi Protected Access with both modes */
	ESP32_WIFI_SEC_WPA2_ENTERPRISE = 0x05, /*!< Wi-Fi Protected Access */
	ESP32_WIFI_SEC_WPA3_PSK = 0x06, /*!< Wi-Fi Protected Access 3 */
	ESP32_WIFI_SEC_WPA2_WPA3_PSK = 0x07, /*!< Wi-Fi Protected Access */
	ESP32_WIFI_SEC_WAPI_PSK = 0x08, /*!< Wi-Fi Protected Access */
	ESP32_WIFI_SEC_UNKNOWN = 0xFF, /*!< Wi-Fi Unknown Security mode */
} ESP32_WIFI_SecurityType_t;

/* Pairwise cipher type for Aps */
typedef enum {
	ESP32_WIFI_PAIRWISE_NONE = 0x01, /*!<  */
	ESP32_WIFI_PAIRWISE_WEP40 = 0x02, /*!<  */
	ESP32_WIFI_PAIRWISE_WEP104 = 0x03, /*!<  */
	ESP32_WIFI_PAIRWISE_TKIP = 0x04, /*!<  */
	ESP32_WIFI_PAIRWISE_CCMP = 0x05, /*!<  */
	ESP32_WIFI_PAIRWISE_TKIP_AND_CCMP = 0x06, /*!<  */
	ESP32_WIFI_PAIRWISE_AES_CMAC_128 = 0x07, /*!<  */
	ESP32_WIFI_PAIRWISE_UNKNOWN = 0x08, /*!<  */
} ESP32_WIFI_CipherType_t;

typedef struct {
	ESP32_WIFI_SecurityType_t Security; /*!< Security of Wi-Fi spot. */
	uint8_t SSID[ESP32_WIFI_MAX_SSID_NAME_SIZE + 1]; /*!< Service Set Identifier value.Wi-Fi spot name */
	int16_t RSSI; /*!< Signal strength of Wi-Fi spot */
	uint8_t MAC[6]; /*!< MAC address of spot */
	uint8_t Channel; /*!< Wi-Fi channel */
	int16_t Freq_Offset; /*!<  */
	int16_t Freqcal_Val; /*!<  */
	ESP32_WIFI_CipherType_t Pairwise_Cipher; /*!<  */
	ESP32_WIFI_CipherType_t Group_cipher; /*!<  */
	uint8_t BGN; /*!<  */
	uint8_t WPS; /*!<  */
} ESP32_WIFI_AP_t;

typedef struct {
	ESP32_WIFI_AP_t AP[ESP32_WIFI_MAX_DETECTED_AP];
	uint8_t nbr;
} ESP32_WIFI_APs_t;

///**
//  * @brief Receive data mode for IO receive function
//  */
//typedef enum
//{
//  ESP32_WIFI_RECEIVEDATAMODE_WITHOUT_MAXLEN = 0,       /* */
//  ESP32_WIFI_RECEIVEDATAMODE_WITH_MAXLEN,       /* */
//
//} ESP32_WIFI_ReceiveDataMode_t;

typedef int8_t (*IO_Init_Func)(uint16_t); /**< I/O interface init function. */
typedef int8_t (*IO_DeInit_Func)(void); /**< I/O interface deinit function. */
typedef void (*IO_Delay_Func)(uint32_t); /**< I/O interface delay function. */
typedef int16_t (*IO_Send_Func)(uint8_t*, uint16_t len, uint32_t); /**< I/O interface send function. */
typedef int16_t (*IO_Receive_Func)(uint8_t*, uint16_t len, uint32_t); /**< I/O interface receive function. */

/**
 * @brief Wi-Fi low level I/O interface operation handles
 */
typedef struct {
	IO_Init_Func IO_Init; /**< I/O interface init function. */
	IO_DeInit_Func IO_DeInit; /**< I/O interface deinit function. */
	IO_Delay_Func IO_Delay; /**< I/O interface delay function. */
	IO_Send_Func IO_Send; /**< I/O interface send function. */
	IO_Receive_Func IO_Receive; /**< I/O interface receive function. */
} ESP32_WIFI_IO_t;

/**
 * @brief Wi-Fi station info
 */
typedef struct {
	uint8_t SSID[ESP32_WIFI_MAX_SSID_NAME_SIZE + 1]; /**< Wi-Fi station SSID. */
	uint8_t pswd[ESP32_WIFI_MAX_PSWD_NAME_SIZE + 1]; /**< Wi-Fi station passwd. */
	ESP32_WIFI_SecurityType_t Security; /**< Wi-Fi station security. */
	uint8_t DHCP_IsEnabled; /**< Wi-Fi station DHCP. */

	int8_t IsConnected; /**< Wi-Fi station connection status. */

	uint8_t IP_Addr[4]; /**< Wi-Fi station IP address. */
	uint8_t IP_Mask[4]; /**< Wi-Fi station IP mask. */
	uint8_t Gateway_Addr[4]; /**< Wi-Fi station gateway. */
	uint8_t DNS1[4]; /**< Wi-Fi station DNS server. */
} ESP32_WIFI_Network_t;

/**
 * @brief Wi-Fi runtime info
 */
typedef struct {
	uint32_t Timeout; /**< Wi-Fi cmd timeout in ms. */

	//esp32_wifi_status_callback_t status_cb;    /**< Wi-Fi status callback. */
	//void *callback_arg;                     /**< Wi-Fi status callback argument. */

	uint8_t scan_result[ESP32_WIFI_SCAN_BUF_SIZE]; /**< Wi-Fi scan result buffer. */
	uint8_t scan_number; /**< Num of Wi-Fi scan result to get. */
} ESP32_WIFI_Runtime_t;

/**
 * @brief Wi-Fi Wi-Fi object handle
 */
typedef struct {
	// HW IO
	ESP32_WIFI_IO_t fops; /**< Wi-Fi low level I/O operation handles. */

	// network info
	ESP32_WIFI_Network_t NetSettings; /**< Wi-Fi station info. */

	// run time data
	ESP32_WIFI_Runtime_t Runtime; /**< Wi-Fi runtime info. */

	// wifi obj lock
#if ESP32_WIFI_USE_CMSIS_OS
  osMutexId wifi_mutex_id;            /**< Wi-Fi object lock id for RTOS. */
  osMutexDef_t wifi_mutex;            /**< Wi-Fi object lock mutex for RTOS. */
#endif
} ESP32_WIFIObject_t;

/* Exported functions --------------------------------------------------------*/
#if ESP32_WIFI_USE_CMSIS_OS
#define ESP32_WIFI_LOCK(a)         (osOK == osMutexWait(a->wifi_mutex_id, ESP32_WIFI_TIMEOUT))    /**< lock Wi-Fi object. */
#define ESP32_WIFI_UNLOCK(a)       (osMutexRelease(a->wifi_mutex_id))  /**< unlock Wi-Fi object. */
#else
#define ESP32_WIFI_LOCK(a)         (true)  /**< lock Wi-Fi object. */
#define ESP32_WIFI_UNLOCK(a)               /**< unlock Wi-Fi object. */
#endif

/**
 * @brief Register low level IO interface.
 * @param Obj wifi object handle.
 * @param IO_Init IO init function
 * @param IO_DeInit IO de-init function
 * @param IO_Delay IO delay function in ms
 * @param IO_Send IO send data function
 * @param IO_Receive IO receive data function
 *
 * @return result
 * @retval ESP32_WIFI_STATUS_OK sucess
 * @retval others failure
 */
ESP32_WIFI_Status_t ESP32_WIFI_RegisterBusIO(ESP32_WIFIObject_t *Obj,
		IO_Init_Func IO_Init, IO_DeInit_Func IO_DeInit, IO_Delay_Func IO_Delay,
		IO_Send_Func IO_Send, IO_Receive_Func IO_Receive);
/**
 * @brief Reset wifi module by hardware IO.
 * @param Obj wifi object handle.
 *
 * @return result
 * @retval ESP32_WIFI_STATUS_OK sucess
 * @retval others failure
 */
ESP32_WIFI_Status_t ESP32_WIFI_HardResetModule(ESP32_WIFIObject_t *Obj);

/**
 * @brief Wi-Fi module init type
 */
typedef enum {
	ESP32_WIFI_INIT = 0, /**< Wi-Fi module init(not reboot). */
	ESP32_WIFI_RESET = 1 /**< Wi-Fi module reset(reboot). */
} ESP32_WIFI_InitMode_t;

/**
 * @brief  Initialize WIFI module and get module fw & mac info.
 * @param  Obj: pointer to module handle
 * @retval Operation Status.
 */
ESP32_WIFI_Status_t ESP32_WIFI_Init(ESP32_WIFIObject_t *Obj);

/**
 * @brief  DeInitialize WIFI module.
 * @param  Obj: pointer to module handle
 * @retval Operation Status.
 */
ESP32_WIFI_Status_t ESP32_WIFI_DeInit(ESP32_WIFIObject_t *Obj);

/**
 * @brief
 * @param
 * @retval Operation Status.
 */
ESP32_WIFI_Status_t ESP32_WIFI_Start(void);
ESP32_WIFI_Status_t ESP32_WIFI_Scan_AP(ESP32_WIFI_APs_t *aps);
ESP32_WIFI_Status_t ESP32_WIFI_Connect(char *ssid, char *password);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_WIFI_H */

/*****************************END OF FILE****/
