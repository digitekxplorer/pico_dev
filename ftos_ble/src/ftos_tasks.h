// September 24, 2024
// FreeRTOS tasks for Pico W BLE project
// Allows us to use ulTaskNotifyTake() and xTaskNotifyGive() to execute 
// vehicle tasks in a separate task to allow BTstack to continue working
// its own tasks.


#ifndef FTOSTASK_H
#define FTOSTASK_H

// Handle for the BLE input task 
static TaskHandle_t xBLEinput_HandlerTask = NULL;   // used in service_implementation.h


#endif   // FTOSTASKS_H
