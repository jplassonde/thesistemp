#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ff.h"
#include <cstring>
#include "project.h"
#include "Jpeg.h"
#include "jpeg_utils.h"

// NOT USED ATM.
// Could be the start of an addtion, to display Album Image from ID3 tags on the screen.
//

/*
#define COVER_ADDR 0xC02EE000
#define DECODE_ADDR 0xC02EE000 + 300*300*4

#define FSBUFFER_SZ 1024

extern SemaphoreHandle_t xFatFsMutex;

SemaphoreHandle_t xInputNeededSema;
SemaphoreHandle_t xOutputNeededSema;
SemaphoreHandle_t xWorkDone;

QueueHandle_t xJpgBufferQueue;
QueueHandle_t xJpgDecodeQueue;
QueueHandle_t xJpgOutputBuffQueue;
QueueHandle_t xJpgRGBConvertQueue;

void JpegBufferingTask(void *pvParameters);
void JpegDecodingTask(void *pvParameters);
void JpegConvertingTask(void *pvParameters);

JPEG_HandleTypeDef JPEG_Handle;
DMA_HandleTypeDef hdmaIn;
DMA_HandleTypeDef hdmaOut;

JPEG_Data_Info_t * JPEG_Info;
InputBuffer_t * inBuffer;
OutputBuffer_t * outBuffer;

void initJpegHw() {


	__HAL_RCC_JPEG_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	NVIC_SetPriority(JPEG_IRQn, 0x0F);
	HAL_NVIC_EnableIRQ(JPEG_IRQn);

	hdmaIn.Init.Channel = DMA_CHANNEL_9;
	hdmaIn.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdmaIn.Init.PeriphInc = DMA_PINC_DISABLE;
	hdmaIn.Init.MemInc = DMA_MINC_ENABLE;
	hdmaIn.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdmaIn.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdmaIn.Init.Mode = DMA_NORMAL;
	hdmaIn.Init.Priority = DMA_PRIORITY_HIGH;
	hdmaIn.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdmaIn.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdmaIn.Init.MemBurst = DMA_MBURST_INC4;
	hdmaIn.Init.PeriphBurst = DMA_PBURST_INC4;
	hdmaIn.Instance = DMA2_Stream7;

	HAL_DMA_DeInit(&hdmaIn);
	HAL_DMA_Init(&hdmaIn);

	__HAL_LINKDMA(&JPEG_Handle, hdmain, hdmaIn);

	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0x07, 0x0F);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);


	hdmaOut.Init.Channel = DMA_CHANNEL_9;
	hdmaOut.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdmaOut.Init.PeriphInc = DMA_PINC_DISABLE;
	hdmaOut.Init.MemInc = DMA_MINC_ENABLE;
	hdmaOut.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdmaOut.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdmaOut.Init.Mode = DMA_NORMAL;
	hdmaOut.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdmaOut.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdmaOut.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdmaOut.Init.MemBurst = DMA_MBURST_INC4;
	hdmaOut.Init.PeriphBurst = DMA_PBURST_INC4;
	hdmaOut.Instance = DMA2_Stream4;

	HAL_DMA_DeInit(&hdmaOut);
	HAL_DMA_Init(&hdmaOut);

	__HAL_LINKDMA(&JPEG_Handle, hdmaout, hdmaOut);

	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0x07, 0x0F);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	HAL_JPEG_Init(&JPEG_Handle);
}

void processJpeg() {
	FIL fp;
	FATFS fs;
	JPEG_Info = (JPEG_Data_Info_t*)pvPortMalloc(sizeof(JPEG_Data_Info_t));
	JPEG_Info->jpegDecoded = false;

	xSemaphoreTake(xFatFsMutex, portMAX_DELAY); // Keep Mutex until done with the whole image transfer
	f_mount(&fs, "", 0);
	f_chdir("files");
	f_open(&JPEG_Info->fp, "dub.mp3", FA_READ);

	if (!getJpegInfos()) {
		f_close(&JPEG_Info->fp);
		f_mount(&fs, NULL, 0);
		xSemaphoreGive(xFatFsMutex);
		vPortFree(JPEG_Info);
		return;
	}

	inBuffer = (InputBuffer_t *)pvPortMalloc(sizeof(InputBuffer_t));
	outBuffer = (OutputBuffer_t *)pvPortMalloc(sizeof(OutputBuffer_t));

	xJpgBufferQueue = xQueueCreate(2, sizeof(uint8_t*));
	xJpgDecodeQueue = xQueueCreate(2, sizeof(uint8_t*));
	xJpgOutputBuffQueue = xQueueCreate(2, sizeof(uint8_t*));
	xJpgRGBConvertQueue = xQueueCreate(2, sizeof(uint8_t*));
	xInputNeededSema = xSemaphoreCreateBinary();
	xWorkDone = xSemaphoreCreateBinary();

	TaskHandle_t bufferingTask;
	TaskHandle_t decodingTask;
	TaskHandle_t convertingTask;

    xTaskCreate(&JpegBufferingTask, "JpegBuffering", configMINIMAL_STACK_SIZE, NULL, 3, &bufferingTask);
    xTaskCreate(&JpegDecodingTask, "JpegDecoding", configMINIMAL_STACK_SIZE, NULL, 3, &decodingTask);
	xTaskCreate(&JpegConvertingTask, "JpegConverting", configMINIMAL_STACK_SIZE, NULL, 3, &convertingTask);

	xSemaphoreTake(xWorkDone, portMAX_DELAY);

	vTaskDelete(bufferingTask);
	vTaskDelete(decodingTask);
	vTaskDelete(convertingTask);

	f_close(&JPEG_Info->fp);
	f_mount(&fs, NULL, 0);
	xSemaphoreGive(xFatFsMutex);


	vSemaphoreDelete(xInputNeededSema);

	vQueueDelete(xJpgBufferQueue);
	vQueueDelete(xJpgDecodeQueue);
	vQueueDelete(xJpgRGBConvertQueue);
	vQueueDelete(xJpgOutputBuffQueue);

	vPortFree(JPEG_Info);
	vPortFree(inBuffer);
	vPortFree(outBuffer);
}

bool getJpegInfos() {
	UINT numRead;
	uint8_t buffer[FSBUFFER_SZ];
	f_read(&JPEG_Info->fp, (void*)buffer, 512, &numRead);

	int i = 0;
	while(buffer[i] != 'A' || buffer[i+1] != 'P' || buffer[i+2] != 'I' || buffer[i+3] != 'C') {
		++i;
		if (i > 512) { // Probably shouldnt have to go that deep into the header but...
			return false; // No image embedded
		}
	}
	uint16_t headerStart = i;
	i+=4;

	JPEG_Info->size = __builtin_bswap64(*(uint64_t*)&buffer[i]);

	i+=6; // skip flags
	uint8_t encoding = buffer[i];
	++i;
	if (strcmp((const char *)&buffer[i], "image/jpeg") != 0) {
		// not a jpeg
		return false;
	}
	while (buffer[i] != 0) {
		++i;
	}
	++i;
	if (encoding == 0) {
		while(buffer[i] != 0) {
			++i;
		}
		++i;
	} else {
		while (*(uint16_t *)&buffer[i] != 0) {
			i+=2;
		}
		i+=3;
	}

	JPEG_Info->size = JPEG_Info->size - (i-(headerStart+10)); // Remove metadata from size
	JPEG_Info->startPos = i;
	JPEG_Info->currPos = i;

	return true;
}

void JpegBufferingTask(void *pvParameters) {
	InputBuffer_t * inBuff;
	while(true) {
		xQueueReceive(xJpgBufferQueue, (void*)inBuff, portMAX_DELAY);
		f_read(&JPEG_Info->fp, inBuff->buffer, IN_BUFFER_SIZE, (UINT*)&inBuff->numBytes);
		xQueueSend(xJpgDecodeQueue, (void*)inBuff, portMAX_DELAY);
	}
}


void JpegDecodingTask(void *pvParameters) {

	f_lseek(&JPEG_Info->fp, JPEG_Info->startPos);
	f_read(&JPEG_Info->fp, inBuffer->buffer, IN_BUFFER_SIZE, (UINT*)&inBuffer->numBytes);
	JPEG_Info->currPos += inBuffer->numBytes;
	JPEG_Info->currInputBuffer = inBuffer;

	HAL_JPEG_Decode_DMA(&JPEG_Handle, inBuffer->buffer, inBuffer->numBytes, outBuffer->buffer, OUT_BUFFER_SIZE);

	InputBuffer_t * inBuff;
	while(1) {
		xSemaphoreTake(xInputNeededSema, portMAX_DELAY);
		xQueueReceive(xJpgDecodeQueue, (void *)inBuff, portMAX_DELAY);
		HAL_JPEG_ConfigInputBuffer(&JPEG_Handle, inBuff->buffer, inBuff->numBytes);
		JPEG_Info->currInputBuffer = inBuff;
		HAL_JPEG_Pause(&JPEG_Handle, JPEG_PAUSE_RESUME_INPUT);
	}


}

void JpegConvertingTask(void *pvParameters) {
	OutputBuffer_t out;
	uint32_t blockIndex = 0;
	uint32_t ConvertedDataCount;
	while(1) {
		xQueueReceive(xJpgRGBConvertQueue, (void *)&out, portMAX_DELAY);
		blockIndex += convertFunction(out.buffer, (uint8_t*)DECODE_ADDR, blockIndex, out.numBytes, &ConvertedDataCount);

		if (blockIndex == JPEG_Info->nbMcu) {
			xSemaphoreGive(xWorkDone);
		}
	}


}


extern "C" {

// JPEG Callbacks

void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo) {
	JPEG_GetDecodeColorConvertFunc(pInfo, &convertFunction, &JPEG_Info->nbMcu);
}

void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	JPEG_Info->jpegDecoded = true;


	xSemaphoreGiveFromISR(xFatFsMutex, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken != pdFALSE) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

}

void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg) {
	while(1); // check error. Do better handling later on
}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData) {
	  if (NbDecodedData == JPEG_Info->currInputBuffer->numBytes) {
		  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		  xQueueSendFromISR(xJpgBufferQueue, (void *)JPEG_Info->currInputBuffer, &xHigherPriorityTaskWoken);
		  if (xHigherPriorityTaskWoken == pdTRUE) {
			  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		  }

		  InputBuffer_t * inBuff;
		  if (xQueueReceiveFromISR(xJpgDecodeQueue, (void *)inBuff, &xHigherPriorityTaskWoken) == pdPASS) {
			  JPEG_Info->currInputBuffer = inBuff;
		      HAL_JPEG_ConfigInputBuffer(hjpeg, inBuff->buffer, inBuff->numBytes);
		  } else {
			  HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
			  xSemaphoreGiveFromISR(xInputNeededSema, &xHigherPriorityTaskWoken);
		  } // Do not wake higher prio tasks in these cases.


	  } else {
		  // Does it ever gets here?
		  while(1);
		// HAL_JPEG_ConfigInputBuffer(hjpeg, JPEG_Info->currInputBuffer->buffer + NbDecodedData, JPEG_Info->currInputBuffer->numBytes - NbDecodedData);

	  }

}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	OutputBuffer_t * out = (OutputBuffer_t *)(((uint32_t *)pDataOut)-1);

	out->numBytes = OutDataLength;
	xQueueSendFromISR(xJpgRGBConvertQueue, (void *)out, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	if (xQueueReceiveFromISR(xJpgOutputBuffQueue, (void *)out, &xHigherPriorityTaskWoken) == pdPASS) {
		HAL_JPEG_ConfigOutputBuffer(hjpeg, out->buffer, out->numBytes);
	} else {
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
		xSemaphoreGiveFromISR(xOutputNeededSema, &xHigherPriorityTaskWoken);
	} // Do not wake higher prio tasks in these cases.

}

// JPEG HW & DMA IRQ Handlers

void JPEG_IRQHandler() {
	HAL_JPEG_IRQHandler(&JPEG_Handle);
}

void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(JPEG_Handle.hdmain);
}

void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(JPEG_Handle.hdmaout);
}
}

*/
