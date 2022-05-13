/****************************************************************************
 File:     main.c
 Info:     Generated by Atollic TrueSTUDIO(R) 9.1.0   2019-02-04

 The MIT License (MIT)
 Copyright (c) 2018 STMicroelectronics

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 ******************************************************************************/

void hardware_config();
/* Includes */
#include "project.h"
#include "Screen.h"
#include "FrameCounter.h"
#include "fbState.h"
#include "TouchEvent.h"
#include "PlayerQueue.h"
#include "IEvent.h"
#include "TrackTime.h"

#include "ControlReq.h"
#include "Wm8994.h"

// Task entry declaration
void TouchEventTask(void *pvParameters);
void MainTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void PlayerTask(void *pvParameters);

uint8_t ucHeap[configTOTAL_HEAP_SIZE * 3];

extern SemaphoreHandle_t xTsI2CSemaphore;
extern SemaphoreHandle_t xI2C4Mutex;
SemaphoreHandle_t xDma2dSemaphore;
SemaphoreHandle_t xDsiSemaphore;
SemaphoreHandle_t xVblankSema;  // More like VSync....
SemaphoreHandle_t xPlayTickSema;
SemaphoreHandle_t xIOXSemaphore;
SemaphoreHandle_t xFatFsMutex;
EventGroupHandle_t xFramebuffersState;
QueueHandle_t xTSEventQueue;
QueueHandle_t xMp3PlayerCmdQueue;
QueueHandle_t xPlayerCmdQueue;
QueueHandle_t xIeQueue;
extern QueueHandle_t xUsbhCtrlQueue;

FrameCounter * fc;
FrameCounter * vbc;
TrackTime * trackTime;


/********************************************************************
 Program entry:
 Init clocks and peripherals, freeRTOS handles and start the tasks.

 MainTask entry in mainEngine.cpp
 Display task entry in Screen.cpp
 TouchEvent task entry in TouchScreen.cpp
 PlayerTask entry in Player.cpp

 ********************************************************************/

// USB Test
extern void UsbTask(void *pvParameters);



int main(void) {
    hardware_config();
    HAL_Delay(200);

    //fc = new FrameCounter();
    //vbc = new FrameCounter();

    xVblankSema = xSemaphoreCreateBinary();
    xDma2dSemaphore = xSemaphoreCreateBinary();
    xDsiSemaphore = xSemaphoreCreateBinary();
    xPlayTickSema = xSemaphoreCreateBinary();
    xIOXSemaphore = xSemaphoreCreateBinary();
    xFatFsMutex = xSemaphoreCreateMutex();
    xFramebuffersState = xEventGroupCreate();
    xTsI2CSemaphore = xSemaphoreCreateBinary();
    xI2C4Mutex = xSemaphoreCreateMutex();

    xTSEventQueue = xQueueCreate(2, sizeof(TOUCH_EVENT_T));
    xMp3PlayerCmdQueue = xQueueCreate(1, sizeof(PLAYER_QUEUE_T));
    xPlayerCmdQueue = xQueueCreate(1, sizeof(PLAYER_QUEUE_T));
    xIeQueue = xQueueCreate(1, sizeof(IEVENT_t));
    xUsbhCtrlQueue = xQueueCreate(5, sizeof(ControlReq*));
    xEventGroupSetBits(xFramebuffersState, BB_AVAILABLE);
    xEventGroupClearBits(xFramebuffersState, BB_DRAWN);

    // IOX DMA Debug
    //vQueueAddToRegistry(xIOXSemaphore, "IOX DMA");

    trackTime = new TrackTime();

	Wm8994* wm = Wm8994::getInstance();
    xTaskCreate(&UsbTask, "UsbTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    xTaskCreate(&DisplayTask, "ScreenRefresh", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);
    xTaskCreate(&MainTask, "MainTask", configMINIMAL_STACK_SIZE * 8, NULL, 2, NULL);
    xTaskCreate(&TouchEventTask, "TsTask", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    xTaskCreate(&PlayerTask, "PlayerTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, NULL);
    vTaskStartScheduler();

    while (1);
}

//---------------------------------------------------------
// Functions overrides

extern "C" {
void HAL_Delay(uint32_t Delay) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        uint32_t tickstart = HAL_GetTick();
        uint32_t wait = Delay;

        if (wait < HAL_MAX_DELAY) {
            wait += (uint32_t)(HAL_TICK_FREQ_DEFAULT);
        }

        while ((HAL_GetTick() - tickstart) < wait) {
        }
    } else {
        vTaskDelay(Delay);
    }
}
}

void * operator new(size_t size) {
	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
		return malloc(size);
	}
    return pvPortMalloc(size);
}

void * operator new[](size_t size) {
    return pvPortMalloc(size);
}

void operator delete(void * ptr) {
    vPortFree(ptr);
}

// Is this eating my memory? No sign of it
void operator delete[](void * ptr) {
    vPortFree(ptr);
}

