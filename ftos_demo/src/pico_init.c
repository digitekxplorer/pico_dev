// July 23, 2024
// Support functions for the Combo project that includes:
// 1) Raspberry Pi Pico initialization

// ****************************
// Pico
// ****************************

//#include <stdio.h>
//#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
//#include "hardware/gpio.h"
#include "defs.h"


// Setup basic pico IO; LEDs and user switches
void pico_io(void) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;  // Pico default LED
    
    // Initialize the LED pin
    gpio_init(YLW_LED);
    // DIP Swithch#0 input
    gpio_init(SW0_GPIO);
    gpio_set_dir(SW0_GPIO, GPIO_IN);
    gpio_pull_down(SW0_GPIO);
    // DIP Swithch#1 input
    gpio_init(SW1_GPIO);
    gpio_set_dir(SW1_GPIO, GPIO_IN);
    gpio_pull_down(SW1_GPIO);

    // Set the LED pin as output
    gpio_set_dir(YLW_LED, GPIO_OUT);
    gpio_put(YLW_LED, 0);
    
    // Default Led output   
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Button input (not used)
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_down(BUTTON_GPIO);
}

// Pico UART Init
void pico_uart_init(void) {
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
//    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);
}

// Print Welcome message
void print_welcome(void) {
    // Add several seconds delay to allow USB port to be set
    uart_puts(UART_ID, "\n\rDelay");
    for(int ii=0; ii< 4; ii=ii+1) {
        uart_puts(UART_ID, ".");
//        printf("Delay %d\n", ii);
        sleep_ms(1000);
    }
    uart_puts(UART_ID, "\n\r");
    
    // Welcome Message
    uart_puts(UART_ID, "\nHello, UART RX interrupts\n\r");
}
