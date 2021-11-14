/**
 ******************************************************************************
 * @file    esp32_wifi.c
 * @author  MCD Application Team
 * @brief   Host driver API of ESP32 Wi-Fi component.
 ******************************************************************************
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "esp32_wifi.h"
//#include "net_address.h"
#include <string.h>
#include <stdlib.h>
extern ESP32_WIFIObject_t* wifi_obj_get(void);
extern int32_t wifi_probe(void);

#define ESP32_WIFI_API_DEBUG

/* Private defines -----------------------------------------------------------*/
#ifdef ESP32_WIFI_API_DEBUG
#define DEBUG_LOG(M, ...)  printf(M, ##__VA_ARGS__);
#define DEBUG_CODE     printf("%s:%d :",__FILE__,__LINE__);printf
#define DEBUG_CMD  printf
#else
#define DEBUG_LOG(M, ...)
#define DEBUG_CODE(...)
#define DEBUG_CMD(...)
#endif

#define AT_COMMAND_AT (uint8_t *)"AT\r\n"
#define AT_COMMAND_ATE0 (uint8_t *)"ATE0\r\n"	//Switch echo off
#define AT_COMMAND_CWMODE_3 (uint8_t *)"AT+CWMODE=3\r\n" //Set the Wi-Fi mode to Station+SoftAP
#define AT_COMMAND_CWLAP (uint8_t *)"AT+CWLAP\r\n" //List Available APs

#define AT_OK_STRING "\r\nOK\r\n"
#define AT_OK_STRING_LEN sizeof(AT_OK_STRING)
#define AT_READY_STRING "\r\nready\r\n"

#define AT_ERROR_STRING "\r\nERROR"

#define AT_DELIMETER_STRING "\r\n> "
#define AT_DELIMETER_LEN        4

#define CHARISHEXNUM(x)                 (((x) >= '0' && (x) <= '9') || \
                                         ((x) >= 'a' && (x) <= 'f') || \
                                         ((x) >= 'A' && (x) <= 'F'))

#define CHARISNUM(x)                    ((x) >= '0' && (x) <= '9')
#define CHAR2NUM(x)                     ((x) - '0')

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Execute AT command.
 * @param  Obj: pointer to module handle
 * @param  cmd: pointer to command string
 * @param  pdata: pointer to returned data
 * @retval Operation Status.
 */
static ESP32_WIFI_Status_t AT_ExecuteCommand(ESP32_WIFIObject_t *Obj,
		uint8_t *cmd) {
	int ret = 0;
	int16_t recv_len = 0;
	uint8_t *pdata = Obj->Runtime.scan_result;
	DEBUG_CMD("%s\n", cmd);
	ret = Obj->fops.IO_Send(cmd, strlen((char*) cmd), Obj->Runtime.Timeout);

	if (ret > 0) {
		recv_len = Obj->fops.IO_Receive(pdata, (ESP32_WIFI_SCAN_BUF_SIZE - 1),
				Obj->Runtime.Timeout);
		*(pdata + recv_len) = 0;
		DEBUG_CMD("%s\n", cmd);

		if (strstr((char*) pdata, AT_OK_STRING)) {
			return ESP32_WIFI_STATUS_OK;
		} else if (strstr((char*) pdata, AT_ERROR_STRING)) {
			return ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
		}

		if (recv_len == ESP32_WIFI_STATUS_IO_ERROR) {
			return ESP32_WIFI_STATUS_MODULE_CRASH;
		}
	}
	return ESP32_WIFI_STATUS_IO_ERROR;
}

static ESP32_WIFI_Status_t AT_ReadReady(ESP32_WIFIObject_t *Obj) {
	//int ret = 0;
	int16_t recv_len = 0;
	uint8_t *pdata = Obj->Runtime.scan_result;
	recv_len = Obj->fops.IO_Receive(pdata, (ESP32_WIFI_SCAN_BUF_SIZE - 1),
			Obj->Runtime.Timeout);
	if (recv_len > 0) {
		*(pdata + recv_len) = 0;
		DEBUG_CMD("%s\n", pdata);
		if (strstr((char*) pdata, AT_READY_STRING)) {
			return ESP32_WIFI_STATUS_OK;
		} else if (strstr((char*) pdata, AT_ERROR_STRING)) {
			return ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
		}
	}
	if (recv_len == ESP32_WIFI_STATUS_IO_ERROR) {
		return ESP32_WIFI_STATUS_MODULE_CRASH;
	}

	return ESP32_WIFI_STATUS_IO_ERROR;
}

/**
 * @brief  Parses Received data.
 * @param  Obj: pointer to module handle
 * @param  cmd:command formatted string
 * @param  pdata: payload
 * @param  ReqlenMax : requested Data max length.
 * @param  ReadData : pointer to received data length.
 * @retval Operation Status.
 */
static ESP32_WIFI_Status_t AT_ReceiveListAvailableAPs(ESP32_WIFIObject_t *Obj,
		uint8_t *cmd, uint16_t *ReadData) {

	uint8_t *pdata = Obj->Runtime.scan_result;
	int len;
	if (Obj->fops.IO_Send(cmd, strlen((char*) cmd), Obj->Runtime.Timeout) > 0) {
		len = Obj->fops.IO_Receive(pdata, (ESP32_WIFI_SCAN_BUF_SIZE - 1),
				Obj->Runtime.Timeout);
		if (len == ESP32_WIFI_STATUS_IO_ERROR) {
			return ESP32_WIFI_STATUS_MODULE_CRASH;
		}
		*ReadData = len;
		*(pdata + len) = 0;
		DEBUG_CMD("%s\n", cmd);
		return ESP32_WIFI_STATUS_OK;
	}
	return ESP32_WIFI_STATUS_IO_ERROR;
}

/**
 * @brief  Execute AT command with data.
 * @param  Obj: pointer to module handle
 * @param  cmd: pointer to command string
 * @param  pcmd_data: pointer to binary data
 * @param  len: binary data length
 * @param  pdata: pointer to returned data
 * @retval Operation Status.
 */

ESP32_WIFI_Status_t AT_RequestSendData(ESP32_WIFIObject_t *Obj,
		uint8_t *cmd, uint8_t *pcmd_data, uint16_t len, uint8_t *pdata) {
	int16_t send_len = 0;
	int16_t recv_len = 0;
	uint16_t cmd_len = 0;
	uint16_t n;

	cmd_len = strlen((char*) cmd);

	/* can send only even number of byte on first send */
	if (cmd_len & 1)
		return ESP32_WIFI_STATUS_ERROR;
	n = Obj->fops.IO_Send(cmd, cmd_len, Obj->Runtime.Timeout);
	if (n == cmd_len) {
		send_len = Obj->fops.IO_Send(pcmd_data, len, Obj->Runtime.Timeout);
		if (send_len == len) {
			recv_len = Obj->fops.IO_Receive(pdata, 0, Obj->Runtime.Timeout);
			if (recv_len > 0) {
				*(pdata + recv_len) = 0;
				if (strstr((char*) pdata, AT_OK_STRING)) {
					return ESP32_WIFI_STATUS_OK;
				} else if (strstr((char*) pdata, AT_ERROR_STRING)) {
					return ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
				} else {
					return ESP32_WIFI_STATUS_ERROR;
				}
			}
			if (recv_len == ESP32_WIFI_STATUS_IO_ERROR) {
				return ESP32_WIFI_STATUS_MODULE_CRASH;
			}
			return ESP32_WIFI_STATUS_ERROR;
		} else {
			return ESP32_WIFI_STATUS_ERROR;
		}
	}
	return ESP32_WIFI_STATUS_IO_ERROR;
}

/**
 * @brief  Parses Received data.
 * @param  Obj: pointer to module handle
 * @param  cmd:command formatted string
 * @param  pdata: payload
 * @param  Reqlen : requested Data length.
 * @param  ReadData : pointer to received data length.
 * @retval Operation Status.
 */
ESP32_WIFI_Status_t AT_RequestReceiveData(ESP32_WIFIObject_t *Obj,
		uint8_t *cmd, char *pdata, uint16_t Reqlen, uint16_t *ReadData) {
	int len;
	uint8_t *p = Obj->Runtime.scan_result;

	if (Obj->fops.IO_Send(cmd, strlen((char*) cmd), Obj->Runtime.Timeout) > 0) {
		len = Obj->fops.IO_Receive(p, 0, Obj->Runtime.Timeout);
		if (len == ESP32_WIFI_STATUS_IO_ERROR) {
			return ESP32_WIFI_STATUS_MODULE_CRASH;
		}
		if ((p[0] != '\r') || (p[1] != '\n')) {
			return ESP32_WIFI_STATUS_IO_ERROR;
		}
		len -= 2;
		p += 2;
		if (len >= AT_OK_STRING_LEN) {
			while (len && (p[len - 1] == 0x15))
				len--;
			p[len] = '\0';
			if (strstr((char*) p + len - AT_OK_STRING_LEN, AT_OK_STRING)) {
				*ReadData = len - AT_OK_STRING_LEN;
				if (*ReadData > Reqlen) {
					*ReadData = Reqlen;
				}
				memcpy(pdata, p, *ReadData);
				return ESP32_WIFI_STATUS_OK;
			} else if (memcmp((char*) p + len - AT_DELIMETER_LEN,
			AT_DELIMETER_STRING, AT_DELIMETER_LEN) == 0) {
				*ReadData = 0;
				return ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
			}

			*ReadData = 0;
			return ESP32_WIFI_STATUS_UNEXPECTED_CLOSED_SOCKET;
		}
	}
	return ESP32_WIFI_STATUS_IO_ERROR;
}

ESP32_WIFI_Status_t ESP32_WIFI_RegisterBusIO(ESP32_WIFIObject_t *Obj,
		IO_Init_Func IO_Init, IO_DeInit_Func IO_DeInit, IO_Delay_Func IO_Delay,
		IO_Send_Func IO_Send, IO_Receive_Func IO_Receive) {
	ESP32_WIFI_Status_t rc;

	if ((NULL == Obj) || (NULL == IO_Init) || (NULL == IO_DeInit)
			|| (NULL == IO_Send) || (NULL == IO_Receive)
			|| (NULL == IO_Delay)) {
		rc = ESP32_WIFI_STATUS_ERROR;
	} else {
		Obj->fops.IO_Init = IO_Init;
		Obj->fops.IO_DeInit = IO_DeInit;
		Obj->fops.IO_Send = IO_Send;
		Obj->fops.IO_Receive = IO_Receive;
		Obj->fops.IO_Delay = IO_Delay;
		rc = ESP32_WIFI_STATUS_OK;
	}
	return rc;
}

ESP32_WIFI_Status_t ESP32_WIFI_HardResetModule(ESP32_WIFIObject_t *Obj) {
	ESP32_WIFI_Status_t rc;
	int32_t ret;

	if (NULL == Obj) {
		rc = ESP32_WIFI_STATUS_ERROR;
	} else {
		ret = Obj->fops.IO_Init(ESP32_WIFI_RESET);
		if ((int32_t) 0 == ret) {
			rc = ESP32_WIFI_STATUS_OK;
		} else {
			rc = ESP32_WIFI_STATUS_ERROR;
		}
	}

	return rc;
}

#if ESP32_WIFI_USE_CMSIS_OS
#define ESP32_WIFI_THREAD_STACK_SIZE (0x500)
static osThreadId ESP32_WIFI_RecvThreadId = NULL;

extern ESP32_WIFIObject_t *wifi_obj_get(void);
static void _ESP32_WIFI_RecvThread(void const *argument)
{
	ESP32_WIFIObject_t *Obj = wifi_obj_get();

	if (NULL != Obj)
	{
		while (true)
		{
			(void)ESP32_WIFI_IO_YIELD(Obj, 500);
			(void)osDelay(1);
		}
	}

	// Delete the Thread
	(void)osThreadTerminate(NULL);
}

#if (osCMSIS >= 0x20000U )
static const osThreadAttr_t attr =
{
	.name = "MxWifiRecvThread",
	.priority = osPriorityBelowNormal,
	. stack_size = ESP32_WIFI_THREAD_STACK_SIZE
};
#endif

static ESP32_WIFI_Status_t _mx_wifi_start_recv_task(ESP32_WIFIObject_t *Obj)
{
	ESP32_WIFI_Status_t rc;

#if (osCMSIS < 0x20000U )
	osThreadDef(mx_wifi_recv, _ESP32_WIFI_RecvThread, osPriorityBelowNormal, 0, ESP32_WIFI_THREAD_STACK_SIZE);
	ESP32_WIFI_RecvThreadId = osThreadCreate(osThread(mx_wifi_recv), Obj);
#else
	ESP32_WIFI_RecvThreadId = osThreadNew((osThreadFunc_t)_ESP32_WIFI_RecvThread, Obj, &attr);
#endif

	if (NULL == ESP32_WIFI_RecvThreadId)
	{
		rc = ESP32_WIFI_STATUS_ERROR;
	}
	else
	{
		rc = ESP32_WIFI_STATUS_OK;
	}

	return rc;
}

static ESP32_WIFI_Status_t _mx_wifi_stop_recv_task(ESP32_WIFIObject_t *Obj)
{
	ESP32_WIFI_Status_t rc;

	(void)Obj;

	if (osOK == osThreadTerminate(ESP32_WIFI_RecvThreadId))
	{
		rc = ESP32_WIFI_STATUS_OK;
	}
	else
	{
		rc = ESP32_WIFI_STATUS_ERROR;
	}

	return rc;
}
#endif

ESP32_WIFI_Status_t ESP32_WIFI_Init(ESP32_WIFIObject_t *Obj) {
	ESP32_WIFI_Status_t ret = ESP32_WIFI_STATUS_ERROR;
	if (NULL == Obj) {
		ret = ESP32_WIFI_STATUS_PARAM_ERROR;
	} else {
#if ESP32_WIFI_USE_CMSIS_OS
#if (osCMSIS < 0x20000U )
    Obj->wifi_mutex_id = osMutexCreate(&(Obj->wifi_mutex));
#else
    Obj->wifi_mutex_id = osMutexNew(&(Obj->wifi_mutex));
#endif /* osCMSIS < 0x20000U */

    if (!ESP32_WIFI_LOCK((Obj)))
    {
      ret = ESP32_WIFI_STATUS_TIMEOUT;
    }
    else
#endif /* ESP32_WIFI_USE_CMSIS_OS */
		{
			// 0. set cmd timeout
			Obj->Runtime.Timeout = ESP32_WIFI_TIMEOUT;

			// 1. init wifi low level IO
			if (0 == Obj->fops.IO_Init(ESP32_WIFI_INIT)) {
				ret = ESP32_WIFI_STATUS_OK;
			} else {
				ret = ESP32_WIFI_STATUS_IO_ERROR;
			}

#if ESP32_WIFI_USE_CMSIS_OS
      (void)ESP32_WIFI_UNLOCK((Obj));
#endif
		}
	}

	return ret;
}

ESP32_WIFI_Status_t ESP32_WIFI_DeInit(ESP32_WIFIObject_t *Obj) {
	ESP32_WIFI_Status_t ret;

	if (NULL == Obj) {
		ret = ESP32_WIFI_STATUS_PARAM_ERROR;
	} else {
#if ESP32_WIFI_USE_CMSIS_OS
    if (!ESP32_WIFI_LOCK((Obj)))
    {
      ret = ESP32_WIFI_STATUS_TIMEOUT;
    }
    else
    {
      if (ESP32_WIFI_STATUS_OK == _mx_wifi_stop_recv_task(Obj))
      {
        (void)mipc_deinit();
        Obj->fops.IO_DeInit();
        ret = ESP32_WIFI_STATUS_OK;
      }
      else
      {
        ret = ESP32_WIFI_STATUS_ERROR;
      }

      (void)ESP32_WIFI_UNLOCK((Obj));
    }
#else
		ret = ESP32_WIFI_STATUS_OK;
#endif
	}

	return ret;
}

ESP32_WIFI_Status_t ESP32_WIFI_Start(void) {
	ESP32_WIFI_Status_t ret = ESP32_WIFI_STATUS_ERROR;
	ESP32_WIFIObject_t *Obj = wifi_obj_get();
	if (NULL == Obj) {
		ret = ESP32_WIFI_STATUS_PARAM_ERROR;
	} else {
		if (wifi_probe() == 0) {
			ret = ESP32_WIFI_Init(Obj);
			if (ret != ESP32_WIFI_STATUS_OK)
				return ret;
			Obj->fops.IO_Init(ESP32_WIFI_RESET);
			ret = AT_ReadReady(Obj);
			if (ret != ESP32_WIFI_STATUS_OK) {
				return ret;
			}
			ret = AT_ExecuteCommand(Obj, AT_COMMAND_ATE0);
			if (ret != ESP32_WIFI_STATUS_OK)
				return ret;
			ret = AT_ExecuteCommand(Obj, AT_COMMAND_AT);
			if (ret != ESP32_WIFI_STATUS_OK)
				return ret;
			ret = AT_ExecuteCommand(Obj, AT_COMMAND_CWMODE_3);
			if (ret != ESP32_WIFI_STATUS_OK)
				return ret;
		}
	}
	return ret;
}

/**
  * @brief  Convert char in Hex format to integer.
  * @param  a: character to convert
  * @retval integer value.
  */

static  uint8_t Hex2Num(char a)
{
    if (a >= '0' && a <= '9') {                             /* Char is num */
        return a - '0';
    } else if (a >= 'a' && a <= 'f') {                      /* Char is lowercase character A - Z (hex) */
        return (a - 'a') + 10;
    } else if (a >= 'A' && a <= 'F') {                      /* Char is uppercase character A - Z (hex) */
        return (a - 'A') + 10;
    }

    return 0;
}

/**
  * @brief  Extract a hex number from a string.
  * @param  ptr: pointer to string
  * @param  cnt: pointer to the number of parsed digit
  * @retval Hex value.
  */
static uint32_t ParseHexNumber(char* ptr, uint8_t* cnt)
{
    uint32_t sum = 0;
    uint8_t i = 0;

    while (CHARISHEXNUM(*ptr)) {                    		/* Parse number */
        sum <<= 4;
        sum += Hex2Num(*ptr);
        ptr++;
        i++;
    }

    if (cnt != NULL) {                               		/* Save number of characters used for number */
        *cnt = i;
    }
    return sum;                                        		/* Return number */
}

/**
  * @brief  Parses and returns MAC address.
  * @param  ptr: pointer to string
  * @param  arr: pointer to MAC array
  * @retval None.
  */
static void ParseMAC(char* ptr, uint8_t* arr)
{
  uint8_t hexnum = 0, hexcnt;

  while(* ptr) {
    hexcnt = 1;
    if(*ptr != ':')
    {
      arr[hexnum++] = ParseHexNumber(ptr, &hexcnt);
    }
    ptr = ptr + hexcnt;
  }
}

static void AT_ParseAP(char *pdata, ESP32_WIFI_AP_t *ap) {
	char *ptr = pdata;
	uint8_t num = 0;
	char *saveptr = NULL;

	ptr = strtok_r(pdata, ",", &saveptr);
	while (ptr != NULL) {
		switch (num++) {
		case 0: /* */
			ap->Security = atoi(ptr);
			break;
		case 1:
			ptr[strlen(ptr) - 1] = 0;
			strncpy((char*) ap->SSID, ptr + 1, ESP32_WIFI_MAX_SSID_NAME_SIZE + 1);
			break;
		case 2:
			ap->RSSI = atoi(ptr);
			break;
		case 3:
			ptr[strlen(ptr) - 1] = 0;
			ParseMAC(ptr + 1, ap->MAC);
			break;
		case 4:
			ap->Channel = atoi(ptr);
			break;
		case 5:
			ap->Freq_Offset = atoi(ptr);
			break;
		case 6:
			ap->Freqcal_Val = atoi(ptr);
			break;
		case 7:
			ap->Pairwise_Cipher = atoi(ptr);
			break;
		case 8:
			ap->Group_cipher = atoi(ptr);
			break;
		case 9:
			ap->BGN = atoi(ptr);
			break;
		case 10:
			ap->WPS = atoi(ptr);
			return;
			break;
		default:
			break;
		}
		ptr = strtok_r(NULL, ",", &saveptr);
	}
}

/**
 * @brief  Parses Access points configuration.
 * @param  pdata: pointer to string
 * @param  aps: Access points structure
 * @retval None.
 */
static void AT_ParseAPs(char *pdata, uint32_t max_len, ESP32_WIFI_APs_t *aps) {
#ifndef SIMULATOR
	char *ptr_start = pdata;
	char *ptr_end = pdata;
	aps->nbr = 0;

#define AT_START_LISTAPs_FRAME_STRING "+CWLAP:("
#define AT_START_LISTAPs_FRAME_STRING_LEN (sizeof(AT_START_LISTAPs_FRAME_STRING) - 1)
#define AT_END_LISTAPs_FRAME_STRING ")\r\n"
#define AT_END_LISTAPs_FRAME_STRING_LEN (sizeof(AT_END_LISTAPs_FRAME_STRING) - 1)

	while ((ptr_start != NULL) && (ptr_end != NULL)
			&& (aps->nbr < ESP32_WIFI_MAX_DETECTED_AP)) {
		ptr_start = strnstr(ptr_start, AT_START_LISTAPs_FRAME_STRING,
				max_len - (uint32_t) (ptr_start - pdata));
		if (ptr_start != NULL) {
			ptr_end = strnstr(ptr_start, AT_END_LISTAPs_FRAME_STRING,
					max_len - (uint32_t) (ptr_start - pdata));
		}
		if ((ptr_start == 0) || (ptr_end == 0))
			break;
		memset(ptr_end, 0, AT_END_LISTAPs_FRAME_STRING_LEN);
		ptr_start += AT_START_LISTAPs_FRAME_STRING_LEN;
		printf("%s\r", ptr_start);
		AT_ParseAP(ptr_start, &aps->AP[aps->nbr]);
		ptr_start = ptr_end + (AT_END_LISTAPs_FRAME_STRING_LEN);
		aps->nbr++;
	}

#else
    uint8_t num = 0;
    APs->nbr = 0;
    while (APs->nbr < ES_WIFI_MAX_DETECTED_AP) {
        switch (num++) {
        case 0: /* Ignore index */
        case 4: /* Ignore Max Rate */
        case 5: /* Ignore Network Type */
        case 7: /* Ignore Radio Band */
            break;

        case 1:
            strncpy_s((char*)APs->AP[APs->nbr].SSID, _countof(APs->AP[0].SSID), "ANTON 1", ES_WIFI_MAX_SSID_NAME_SIZE + 1);
            break;

        case 2:
            APs->AP[APs->nbr].MAC[0] = 0;
            APs->AP[APs->nbr].MAC[1] = 1;
            APs->AP[APs->nbr].MAC[2] = 2;
            APs->AP[APs->nbr].MAC[3] = 3;
            APs->AP[APs->nbr].MAC[4] = 4;
            APs->AP[APs->nbr].MAC[5] = 5;
            break;

        case 3:
            APs->AP[APs->nbr].RSSI = 1;
            break;

        case 6:
            APs->AP[APs->nbr].Security = ES_WIFI_SEC_WPA2;
            break;

        case 8:
            APs->AP[APs->nbr].Channel = 1;
            APs->nbr++;
            num = 1;
            break;

        default:
            break;
        }
    }

#endif //SIMULATOR
}

ESP32_WIFI_Status_t ESP32_WIFI_Scan_AP(ESP32_WIFI_APs_t *aps) {
	ESP32_WIFI_Status_t ret = ESP32_WIFI_STATUS_ERROR;
	ESP32_WIFIObject_t *Obj = wifi_obj_get();
	uint16_t readDataLen;
	if (NULL == Obj) {
		ret = ESP32_WIFI_STATUS_PARAM_ERROR;
	} else {
        memset(Obj->Runtime.scan_result, 0, ESP32_WIFI_SCAN_BUF_SIZE);
		ret = AT_ReceiveListAvailableAPs(Obj, AT_COMMAND_CWLAP, &readDataLen);
		if (ret != ESP32_WIFI_STATUS_OK)
			return ret;
		AT_ParseAPs((char*) Obj->Runtime.scan_result, ESP32_WIFI_SCAN_BUF_SIZE,
				aps);
	}

	return ret;
}


ESP32_WIFI_Status_t ESP32_WIFI_Connect(char *ssid, char *password) {
	char wifi_cmd_buff[100];
	ESP32_WIFI_Status_t ret = ESP32_WIFI_STATUS_ERROR;
	ESP32_WIFIObject_t *Obj = wifi_obj_get();
	if (NULL == Obj) {
		ret = ESP32_WIFI_STATUS_PARAM_ERROR;
	} else {
		memset(wifi_cmd_buff, 0, sizeof(wifi_cmd_buff));
		snprintf(wifi_cmd_buff, sizeof(wifi_cmd_buff), "AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password);
		memset(Obj->Runtime.scan_result, 0, ESP32_WIFI_SCAN_BUF_SIZE);
		Obj->Runtime.Timeout = 30000;
		ret = AT_ExecuteCommand(Obj, (uint8_t *)wifi_cmd_buff);
		Obj->Runtime.Timeout = ESP32_WIFI_TIMEOUT;
		if (ret != ESP32_WIFI_STATUS_OK)
			return ret;
	}

	return ret;
}


/*****************************END OF FILE****/
