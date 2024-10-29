// September 24, 2024
// Pico W project to create a build with FreeRTOS and BTstack

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// ***************************
// LEDs and Switches
// ***************************
// External LEDs
#define LED_EX1  14
#define LED_EX2  15
#define LED_EX3  16
#define LED_EX4  17

// Buffer for Client commands; used in ftos_tasks.h and service_implementation.h
#define BLE_IN_SIZE 100
// char *  	characteristic_d_value ;
static char ble_input[BLE_IN_SIZE] ;      // commands from client (phone) placed here
