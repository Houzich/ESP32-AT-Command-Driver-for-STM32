/**
 ******************************************************************************
 * @file    esp32_wifi_io.c
 * @author  MCD Application Team
 * @brief
 ******************************************************************************
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <string.h>
#include "esp32_wifi.h"
#include "cmsis_os.h"
#if (ESP32_WIFI_USE_SPI == 0)

#define ESP32_WIFI_IO_DEBUG

/* Private define ------------------------------------------------------------*/
#ifdef ESP32_WIFI_IO_DEBUG
#define DEBUG_LOG(M, ...)  printf(M, ##__VA_ARGS__);
#define DEBUG_CODE     printf("%s:%d :",__FILE__,__LINE__);printf
#define DEBUG_CMD  printf
#else
#define DEBUG_LOG(M, ...)
#define DEBUG_CODE(...)
#define DEBUG_CMD(...)
#endif

#define esp_uart	huart1

#if (osCMSIS < 0x20000U )
#define OSSEMAPHOREWAIT osSemaphoreWait
#else
#define OSSEMAPHOREWAIT osSemaphoreAcquire
#endif /* osCMSIS */

#define ESP32_WIFI_RESET_MODULE()      do{\
                                        HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, GPIO_PIN_RESET);\
                                        HAL_Delay(10);\
                                        HAL_GPIO_WritePin(WIFI_EN_GPIO_Port, WIFI_EN_Pin, GPIO_PIN_SET);\
                                        HAL_Delay(10);\
                                      }while(0);

#if ESP32_WIFI_USE_UART_INTERRUPT
#define RING_BUFFER_SIZE          (ESP32_WIFI_DATA_SIZE + 500)
typedef struct {
	uint8_t data[RING_BUFFER_SIZE];
	__IO uint16_t tail;
	__IO uint16_t head;
} RingBuffer_t;
__IO ITStatus UartReady = RESET;
#endif /* ESP32_WIFI_USE_UART_INTERRUPT */

ESP32_WIFIObject_t* wifi_obj_get(void);

static ESP32_WIFIObject_t Esp32WifiObj;

#if ESP32_WIFI_USE_UART_INTERRUPT
static __IO RingBuffer_t WiFiRxBuffer;
#endif /* ESP32_WIFI_USE_UART_INTERRUPT */
#ifdef NET_USE_RTOS
osSemaphoreId wifi_uart_rx_sem;
osSemaphoreDef(wifi_uart_rx_sem);
#endif /* NET_USE_RTOS */

/* Private variables ---------------------------------------------------------*/

ESP32_WIFIObject_t* wifi_obj_get(void) {
	return &Esp32WifiObj;
}

static void ESP32_WIFI_IO_DELAY(uint32_t ms) {
#if ESP32_WIFI_USE_CMSIS_OS
  osDelay(ms);
#else
	HAL_Delay(ms);
#endif /* ESP32_WIFI_USE_CMSIS_OS */
}

/**
 * @brief  Initialize the ESP32 WIFI module without hardware
 * @param  None
 * @retval None
 */
static int8_t ESP32_WIFI_IO_Init(uint16_t mode) {
	int8_t rc = 0;

	if (ESP32_WIFI_RESET == mode) {
		ESP32_WIFI_RESET_MODULE();
	} else {

#if  ESP32_WIFI_USE_UART_INTERRUPT
		WiFiRxBuffer.head = 0;
		WiFiRxBuffer.tail = 0;
#endif /* ESP32_WIFI_USE_UART_INTERRUPT */

#ifdef NET_USE_RTOS
#if (osCMSIS < 0x20000U )
    wifi_uart_rx_sem = osSemaphoreCreate(osSemaphore(wifi_uart_rx_sem), 1);
#else
    wifi_uart_rx_sem = osSemaphoreNew(1, 1, NULL);
#endif /* osCMSIS */
    OSSEMAPHOREWAIT(wifi_uart_rx_sem, 1);
#else

#endif /* NET_USE_RTOS */

#if ESP32_WIFI_USE_UART_INTERRUPT
		HAL_UART_Receive_IT(&esp_uart,
				(uint8_t*) &WiFiRxBuffer.data[WiFiRxBuffer.tail], 1);
#endif /* ESP32_WIFI_USE_UART_INTERRUPT */
	}

	return rc;
}

/**
 * @brief  Deinitialize the ESP32 WIFI module without hardware
 * @param  None
 * @retval None
 */
static int8_t ESP32_WIFI_IO_DeInit(void) {
	int8_t rc = 0;

#ifdef NET_USE_RTOS
  osSemaphoreDelete(wifi_uart_rx_sem);
#endif /* NET_USE_RTOS */

	return rc;
}

#if  ESP32_WIFI_USE_UART_INTERRUPT
/**
 * @brief  Rx Callback when new data is received on the UART.
 * @param  UartHandle: Uart handle receiving the data.
 * @retval None.
 */
void ESP32_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void ESP32_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	uint32_t tail;
	/* WIFI UART */
	/* If ring buffer end is reached reset tail pointer to start of buffer */
	if (++WiFiRxBuffer.tail >= RING_BUFFER_SIZE) {
		WiFiRxBuffer.tail = 0;
	}

	/* ringbuffer full, overlap head */
	tail = WiFiRxBuffer.tail;
	if (tail == WiFiRxBuffer.head) {
		WiFiRxBuffer.head++;
	}

	// fired next reception
	HAL_UART_Receive_IT(UartHandle,
			(uint8_t*) &WiFiRxBuffer.data[WiFiRxBuffer.tail], 1);

#ifdef NET_USE_RTOS
  osSemaphoreRelease(wifi_uart_rx_sem);
#endif /* NET_USE_RTOS */
}

#endif /* ESP32_WIFI_USE_UART_INTERRUPT */

/**
 * @brief  Send wifi Data thru UART
 * @param  pdata : pointer to data
 * @param  len : Data length
 * @param  timeout : send timeout in mS
 * @retval Length of sent data
 */
static int16_t ESP32_WIFI_UART_SendData(uint8_t *pdata, uint16_t len,
		uint32_t timeout) {
	int16_t rc = 0;

	if (HAL_UART_Transmit(&esp_uart, pdata, len, timeout) != HAL_OK) {
		return ESP32_WIFI_STATUS_IO_ERROR;
	}

//	/*## Wait for the end of the transfer ###################################*/
//	while (UartReady != SET) {
//	}
//	/* Reset transmission flag */
//	UartReady = RESET;

	rc = len;
	/*DEBUG_LOG("ESP32_WIFI_TX: %d bytes, [%.*s]\r\n", len, len, pdata);*/

	return rc;
}

int16_t ESP32_WIFI_UART_Wait_While_Receive(uint32_t timeout, uint16_t max_len) {
	int16_t readData = 0;
	uint32_t timeOut = timeout;
	uint16_t head = WiFiRxBuffer.head;
	uint32_t tickStart = HAL_GetTick();
	/* Loop until data received */
	do {
		if (head != WiFiRxBuffer.tail) {
			//if receiving data
			tickStart = HAL_GetTick();
			timeOut = 20;

			head++;
			if (head >= RING_BUFFER_SIZE) {
				/* wrap */
				head = 0;
			}
			readData++;
		}
	} while ((HAL_GetTick() - tickStart) < timeOut);
	return readData;
}

static int16_t ESP32_WIFI_UART_ReceiveData(uint8_t *pdata, uint16_t max_len, uint32_t timeout) {
	int16_t len = 0;

#if    ESP32_WIFI_USE_UART_INTERRUPT==0
  len = request_len;
  if (HAL_UART_Receive(&esp_uart, pdata, len, timeout) != HAL_OK)
  {
    return ESP32_WIFI_STATUS_IO_ERROR;
  }
  /*DEBUG_LOG("ESP32_WIFI_RX: %d bytes, [%.*s]\r\n", len, len, pdata);*/
#else
	int32_t tail;
	ESP32_WIFI_UART_Wait_While_Receive(timeout, max_len);

#ifdef NET_USE_RTOS
  OSSEMAPHOREWAIT(wifi_uart_rx_sem, 1);
#endif /* NET_USE_RTOS */
	tail = WiFiRxBuffer.tail;
	len = ((RING_BUFFER_SIZE + tail - WiFiRxBuffer.head) % RING_BUFFER_SIZE);
	if (len == 0) {
		return 0;
	}

	if ((len > max_len) && (max_len != 0)) {
		len = max_len;
	}

	/*copy from buffer */
	for (uint32_t i = 0; i < len; i++) {
		*pdata++ = WiFiRxBuffer.data[WiFiRxBuffer.head++];
		if (WiFiRxBuffer.head >= RING_BUFFER_SIZE) {
			/* wrap */
			WiFiRxBuffer.head = 0;
		}
	} DEBUG_LOG("ESP32_WIFI_RX: %d bytes, %s\r\n", len, pdata);
#endif /* ESP32_WIFI_USE_UART_INTERRUPT */

	return len;
}

/**
 * @brief  probe function to register wifi to connectivity framwotk
 * @param  None
 * @retval None
 */
int32_t wifi_probe(void) {
	if (ESP32_WIFI_RegisterBusIO(&Esp32WifiObj, ESP32_WIFI_IO_Init,
			ESP32_WIFI_IO_DeInit, ESP32_WIFI_IO_DELAY, ESP32_WIFI_UART_SendData,
			ESP32_WIFI_UART_ReceiveData) == 0) {
		return 0;
	}

	return -1;
}

void ESP32_WIFI_UART_StartReceive_IT(void)
{

}

extern void  ESP32_BOOT_UART_ErrorCallback(UART_HandleTypeDef *UartHandle);
extern void  ESP32_BOOT_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
extern void  ESP32_BOOT_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);

/**
 * @brief  Rx Callback when new data is received on the UART.
 * @param  UartHandle: Uart handle receiving the data.
 * @retval None.
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	if (UartHandle == &esp_uart) {
		ESP32_UART_RxCpltCallback(UartHandle);
	}
#ifdef USE_ESP32_BOOTLOADER
	ESP32_BOOT_UART_RxCpltCallback(UartHandle);
#endif /* USE_ESP32_BOOTLOADER */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	if (UartHandle == &esp_uart) {
		/* Set transmission flag: transfer complete */
		UartReady = SET;
	}
#ifdef USE_ESP32_BOOTLOADER
	ESP32_BOOT_UART_TxCpltCallback(UartHandle);
#endif /* USE_ESP32_BOOTLOADER */
}

/**
 * @brief  Function called when error happens on the UART.
 * @param  UartHandle: Uart handle receiving the data.
 * @retval None.
 */

//  *            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in mute mode)
//  *            @arg @ref UART_FLAG_SBKF  Send Break flag
//  *            @arg @ref UART_FLAG_CMF   Character match flag
//  *            @arg @ref UART_FLAG_BUSY  Busy flag
//  *            @arg @ref UART_FLAG_ABRF  Auto Baud rate detection flag
//  *            @arg @ref UART_FLAG_ABRE  Auto Baud rate detection error flag
//  *            @arg @ref UART_FLAG_CTS   CTS Change flag
//  *            @arg @ref UART_FLAG_LBDF  LIN Break detection flag
//  *            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
//  *            @arg @ref UART_FLAG_TC    Transmission Complete flag
//  *            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
//  *            @arg @ref UART_FLAG_RTOF  Receiver Timeout flag
//  *            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
//  *            @arg @ref UART_FLAG_ORE   Overrun Error flag
//  *            @arg @ref UART_FLAG_NE    Noise Error flag
//  *            @arg @ref UART_FLAG_FE    Framing Error flag
//  *            @arg @ref UART_FLAG_PE    Parity Error flag
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
	if(UartHandle == &esp_uart){
	// Dummy data read to clear the FE, PE, SR flag
	UartHandle->Instance->RDR;
	//uint32_t error = HAL_UART_GetError(UartHandle);
	__HAL_UART_CLEAR_OREFLAG(UartHandle);
	__HAL_UART_CLEAR_NEFLAG(UartHandle);
	__HAL_UART_CLEAR_FEFLAG(UartHandle);
	__HAL_UART_CLEAR_PEFLAG(UartHandle);
	UartReady = RESET;
	} else{
#ifdef USE_ESP32_BOOTLOADER
	ESP32_BOOT_UART_ErrorCallback(UartHandle);
#endif /* USE_ESP32_BOOTLOADER */
	}
	/* Call  the WIFI_Handler() to deinitialize the UART Interface. */
	//assert_param(1);
	//WIFI_Handler();
}



typedef enum {
	ESP32_WIFI_SPI_TRANSFER_WAIT,
	ESP32_WIFI_SPI_TRANSFER_COMPLETE,
	ESP32_WIFI_SPI_TRANSFER_ERROR
}ESP32_WIFI_SPI_StatusType_t;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Buffer used for transmission */
const uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on DMA **** SPI Message ********* SPI Message *********";

/* Buffer used for reception */
#define BUFFER_ALIGNED_SIZE (((BUFFERSIZE+31)/32)*32)
ALIGN_32BYTES(uint8_t aRxBuffer[BUFFER_ALIGNED_SIZE]);

/* transfer state */
__IO ESP32_WIFI_SPI_StatusType_t wTransferState = ESP32_WIFI_SPI_TRANSFER_WAIT;
int ESP32_WIFI_SPI_TransmitReceive_DMA(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	  wTransferState = ESP32_WIFI_SPI_TRANSFER_WAIT;
	  if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK)
	  {
	    /* Transfer error in transmission process */
	    return -3;
	  }

	  while (wTransferState == ESP32_WIFI_SPI_TRANSFER_WAIT)
	  {
		  osDelay(1);
	  }
	    /* Invalidate cache prior to access by CPU */
	  SCB_InvalidateDCache_by_Addr ((uint32_t *)aRxBuffer, BUFFERSIZE);
	  return 0;
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Transfer in transmission process is complete */
  /* Transfer in reception process is complete */
  wTransferState = ESP32_WIFI_SPI_TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = ESP32_WIFI_SPI_TRANSFER_ERROR;
}





#endif /* ESP32_WIFI_USE_SPI */
/*****************************END OF FILE****/
