// September 24, 2024
// Pico W project to create a build with FreeRTOS and BTstack

// ****************************
// Pico
// ****************************

//#include <stdio.h>
//#include <stdlib.h>
#include "pico/stdlib.h"
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
//#ifdef CYW43_WL_GPIO_LED_PIN
//#include "pico/cyw43_arch.h"
//#endif

#include "defs.h"


// Setup basic pico IO; LEDs and user switches
void pico_io(void) {
    // ********************************************
    // Initialize pins for input and output
    gpio_init(LED_EX1);               // used for BTstack LED "LED Status and Control" characteristic_e
    gpio_set_dir(LED_EX1, GPIO_OUT);
    gpio_init(LED_EX2);
    gpio_set_dir(LED_EX2, GPIO_OUT);   
    gpio_init(LED_EX3);
    gpio_set_dir(LED_EX3, GPIO_OUT);
    gpio_init(LED_EX4);
    gpio_set_dir(LED_EX4, GPIO_OUT);
}

// Perform Pico or Pico W LED initialisation
/*
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}
*/

// Print Welcome message
void print_welcome(void) {
    // Add several seconds delay to allow USB port to be set
    for(int ii=0; ii< 4; ii=ii+1) {
        printf("Delay %d\n", ii);
        sleep_ms(1000);
    }
    // Welcome Message
    printf("Hello FreeRtos/BLE Server\n");
//    uart_puts(UART_ID, "\nHello, UART RX interrupts\n\r");
}

