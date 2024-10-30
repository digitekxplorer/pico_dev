// June 12, 2024
// Combination display project using FreeRTOS
// There are three types of displays:
// 1) blinking LEDs
// 2) 7-segment display using 74HC595
// 3) SSD1306
// Also added a UART inteface.  Incorporated UART RX interrupt with deferred
// task to read input from the user.
// 4) UART
// Link:
// https://freertos.org/Documentation/RTOS_book.html
// Examples copied to:
// /home/arbz/aStore/ab_raspberryPi/FreeRTOS

// Note: supporting_functions.c found at:
// /home/arbz/aStore/ab_raspberryPi/FreeRTOS/source-code-for-book-examples.20240207/Win32-simulator-MSVC

// Useful commands:
// openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ftos_demo.elf reset exit"
// minicom -b 115200 -o -D /dev/ttyACM0
//
// Useful links:
// https://www.raspberrypi.com
// https://github.com/raspberrypi
// https://freertos.org/Documentation/RTOS_book.html

// Releases:
// 10/30/2024  A. Baeza  Initial release: V1.0.0

/*
 * June 14, 2024
 * xTaskCreate() vs xTimerCreate()
 * Meta.AI answers
 * 
 * xTaskCreate() and xTimerCreate() have different purposes, so it's not exactly fair to compare their 
 * efficiency directly. However, I can provide some insights:
 * xTaskCreate()
 *   - Creates a dedicated task that runs continuously, consuming CPU resources.
 *   - More suitable for tasks that need to run for extended periods or perform complex operations.
 *   - Has a higher overhead due to task creation, context switching, and stack allocation.
 * 
 * xTimerCreate()
 *   - Creates a timer that triggers a callback function at a specified interval.
 *   - More suitable for periodic tasks or tasks that need to run at specific intervals.
 *   - Has a lower overhead since the timer is managed by the FreeRTOS kernel, and the callback function is only executed when the timer expires.
 * 
 * Benefits of using one over the other:
 * Use xTaskCreate() when:
 *   - You need to run a task continuously or for an extended period.
 *   - The task requires a lot of processing power or complex operations.
 * Use xTimerCreate() when:
 *   - You need to run a task at a specific interval or periodically.
 *   - You want to minimize overhead and only execute a short callback function.
 * 
 * In general, if you need to perform a quick operation at a specific interval, 
 * xTimerCreate() is more efficient. If you need to run a more complex task 
 * continuously, xTaskCreate() is a better choice.
 */
 
/*
 * July 20, 2024
 * printf() doesn't work with current uart setup and we need current uart
 * setup to get uart RX to work with Putty. Minicom is able to transmit uart
 * data (i.e. TX works) so printf() works with Mincom.  However, we are unable
 * to receive uart data (i.e. RX does not work).  Putty TX and RX work but
 * does not work when we use stdio_init_all() to configure the default uart.
 * Had to replace printf() with uart_puts() to print text to Putty.
 *
 * Use UART RX to create an interrupt instead of using a Timer to create the
 * interrupt.  A Deferred task was also added to perform extended functions
 * to minimize time in the interrupt.
 */
 
 

// From this description, I decided to used xTimerCreate to blink the LEDs and
// to update the 7-segment display. 
// 
// June 15, 2024
// Created combo_7seg.c and combo_7seg.h.  Moved 7-segment function to
// combo_7seg.c.
// June 16, 2024
// Create combo_ssd1306.c and combo_ssd1306.h files. Moved ssd1306 functions
// to new files.
// June 17, 2024
// File cleanup; deleted unused code.  Added defs.h.
// June 23, 2024
// Moved SSD1306 functions to combo_ssd1306.c
// July 20, 2024
// Add UART RX interrupt to enable commands to be sent to Pico
// July 22, 2024
// Replaced Semaphore with Task Notification to synchronize deferred task to UART
// RX ISR. Remember, to set configUSE_TASK_NOTIFICATIONS to 1 in FreeRTOSConfig.h.
// Rename project to ftos_demo from combo_rxIntr

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"

#include "hardware/uart.h"
#include "hardware/irq.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

// ab
#include "pico_init.h"
#include "ftos_7segPlus.h"
#include "ftos_ssd1306.h"
#include "defs.h"

// FreeRTOS: The periods assigned to one-shot and auto-reload timers
#define mainONE_SHOT_TIMER_PERIOD       ( pdMS_TO_TICKS( 3333UL ) )
#define SE7_RELOAD_TIMER_PERIOD         ( pdMS_TO_TICKS( 500UL ) )
#define YELLOW_LED_RELOAD_TIMER_PERIOD  ( pdMS_TO_TICKS( 250UL ) )
#define TaskDEFLTLED_DLY   250

// Debug
#define PRINT_MESS    // // print debugging message


uint8_t seg7NumIndx = 0;    // 7-segment char index

//encoding for character 0-F of common anode SevenSegmentDisplay. 
unsigned char seg7_chr = 0x00;

// Variable to store the combined Dip Switch values
uint8_t sw_input = 0x00;
uint8_t sw_input_sav = 0x00;

// NVIC registers
io_rw_32 *nvic_int_en_ptr = (io_rw_32 *)(PPB_BASE + M0PLUS_NVIC_ISER_OFFSET);

// *************
// FreeRTOS 
// *************
//
// The timer handles are used inside the callback function so 
// these timers are given file scope.
static TimerHandle_t xSeg7ReloadTimer, xOneShotTimer;
static TimerHandle_t xYlwLedReloadTimer;

// Function Prototypes for FreeRTOS Tasks and Callback
// The blinking LED function; Pico default LED
static void vTaskBlinkDefault(); 
// RX UART Deferred Task
static void vUartRxDeferredIntrHandlerTask( void * pvParameters );

// SSD1306 display
static void vTaskSSD1306(); 

// The callback function that is used by both timers.
static void prvTimerCallback( TimerHandle_t xTimer );

// UART RX interrupt handler
static void vUartRxInterruptHandler();

// Declare a variable of type SemaphoreHandle_t.  Used to reference
// the semaphore that is used to synchronize a task with an interrupt.
//SemaphoreHandle_t xUartRxBinarySemaphore;

// Stores the handle of the task to which interrupt processing is deferred.
static TaskHandle_t xUartRXHandlerTask = NULL;

// *****************************************************************
// *****************************************************************
// Main
// *****************************************************************
int main() {
//    stdio_init_all();   // manually initialize UART

    // ****************
    // Pico Setup
    // ****************
    pico_io();          // setup Pico LEDs and user switches
    pico_uart_init();   // setup UART
    print_welcome();    // send welcome message to UART
    
    // setup Pico control pins for HC595
    setup_7seg();       // 7-segment display
    setup_ssd1306();    // setup ssd1306 display
    sleep_ms(1000);     // short delay
    
    // State of each DIP Switch input
    // Read Dip Switch settings
    sw_input = 0x00;
    bool sw0_state = gpio_get(SW0_GPIO);
    bool sw1_state = gpio_get(SW1_GPIO);
    sw_input = (sw1_state << 1) | sw0_state;
    sw_input_sav = sw_input;
    
    // ****************
    // FreeRTOS setup
    // ****************
    // Timer tasks
    BaseType_t xTimer1Started, xTimer7SegStarted;
    // Blinking LED timer
    BaseType_t xTimerYlwLedStarted;
 
    // UART RX interrupt setup
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, vUartRxInterruptHandler);
    irq_set_enabled(UART_IRQ, true);  // Note: Timer#3 interrupt is enabled??
//    printf("ISER Interrupt Set-Enable Register after: 0x%08x\n", *nvic_int_en_ptr);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
//    uart_set_irq_enables(UART_ID, true, true);   // both rx and tx
    
    
    // ****************************************************
    // Big Loop
    // ****************************************************
    while (1) {
    // Read Dip Switch settings
      sw_input = 0x00;
      sw0_state = gpio_get(SW0_GPIO);
      sw1_state = gpio_get(SW1_GPIO);
      sw_input = (sw1_state << 1) | sw0_state;
//      printf("Switch Reading %d\n", sw_input);
      
      // Check Dip Switch state at start of each Big Loop
      // If there is a Dip Switch change then clear all displays
      if (sw_input_sav != sw_input) {  // new switch setting
        sw_input_sav = sw_input;
        // Clear all displays
        // SSD1306
        ClearDisplay();   // clear SSD1306 display
        sleep_ms(200);    // short delay
        // Seven Sement
        clr_7seg_dsply();
        clr_ctrls();           // clear controls
        gpio_put(YLW_LED, 0);  // turn off LED
      }
      
    // ****************************************************
    // ****************
    // Blink Yellow LED
    // ****************
//    if (sw_input == 0x10 ) {
    if (sw_input == 0 ) { 
      gpio_put(YLW_LED, 1);  // turn on LED
      sleep_ms(150);         // Wait for 150 milliseconds
      gpio_put(YLW_LED, 0);  // turn off LED
      // There are other delays, so make this one short
      sleep_ms(150);  // Wait for 50 milliseconds
    }  // Yellow LED End

    // ****************
    // Seven Segment Display
    // ****************
//    else if (sw_input == 0x00 ) {
    else if (sw_input == 1 ) {
      // Send number sequence to Seven Segment        
      wr_num_dsply();
      clr_ctrls();        
    }  // Seven Segment End
    
    // ****************
    // SSD1306 Display
    // ****************
//    else if (sw_input == 0x01 ) {
    else if (sw_input == 2 ) {
        // clear SSD1306 display
        ClearDisplay();	
	// send long message to SSD1306
	ssd1306_mess_long();
    }  // SSD1306 Display End
    
    // ****************************************************
    // FreeRTOS Tasks
    // ****************************************************
    else {
      // One-Shot timmer: Create the one shot timer, storing the handle to the created timer in xOneShotTimer.
      xOneShotTimer = xTimerCreate( "OneShot",               // Text name for the timer - not used by FreeRTOS
                                mainONE_SHOT_TIMER_PERIOD, // The timer's period in ticks
                                pdFALSE,                   // Set uxAutoRealod to pdFALSE to create a one-shot timer
                                0,                         // The timer ID is initialised to 0
                                prvTimerCallback );        // The callback function to be used by the timer being created

      // 7-Segment Display Timer: Create the auto-reload, storing the handle to the created timer in xSeg7ReloadTimer
      xSeg7ReloadTimer = xTimerCreate( "AutoReload 7-Segment Display", SE7_RELOAD_TIMER_PERIOD,
                                   pdTRUE, 0, prvTimerCallback ); 
				     
      // Blink Yellow LED: Create the auto-reload, storing the handle to the created timer in xYlwLedReloadTimer.
      xYlwLedReloadTimer = xTimerCreate( "AutoReload LED", YELLOW_LED_RELOAD_TIMER_PERIOD,
                                   pdTRUE, 0, prvTimerCallback );

      // Check the timers were created.
      if( ( xOneShotTimer != NULL ) && ( xSeg7ReloadTimer != NULL ) && (xYlwLedReloadTimer != NULL)) {
        // create a FreeRTOS Yellow Blinking LED  and SSD1306 tasks
        xTaskCreate(vTaskBlinkDefault, "Blink Task", 128, NULL, 1, NULL);         // Pico defualt LED
	    // TODO: May have to replace sleep_ms() delays with vTaskDelay()
	    xTaskCreate(vTaskSSD1306, "SSD1306 Display Task", 128, NULL, 1, NULL);
	    
        // Create the 'handler' task, which is the task to which interrupt
        // processing is deferred, and so is the task that will be synchronized
        // with the interrupt.  The handler task is created with a high priority to
        // ensure it runs immediately after the interrupt exits.  In this case a
        // priority of 3 is chosen.
//        xTaskCreate( vUartRxDeferredIntrHandlerTask, "UART RX Handler", 1000, NULL, 3, NULL );
        xTaskCreate( vUartRxDeferredIntrHandlerTask, "UART RX Handler", 1000, NULL, 3, &xUartRXHandlerTask ); 
        
        // Start the timers, using a block time of 0 (no block time).  The
	    // scheduler has not been started yet so any block time specified here
	    // would be ignored anyway.
        xTimer1Started = xTimerStart( xOneShotTimer, 0 );
        xTimer7SegStarted = xTimerStart( xSeg7ReloadTimer, 0 );
	    xTimerYlwLedStarted = xTimerStart( xYlwLedReloadTimer, 0 );

        // The implementation of xTimerStart() uses the timer command queue, and
        // xTimerStart() will fail if the timer command queue gets full.  The timer
        // service task does not get created until the scheduler is started, so all
        // commands sent to the command queue will stay in the queue until after
        // the scheduler has been started.  Check calls to xTimerStart() passed.
        if( ( xTimer1Started == pdPASS ) && ( xTimer7SegStarted == pdPASS ) &&  (xTimerYlwLedStarted == pdPASS))
          {
            // Start the scheduler.
            vTaskStartScheduler();
          }
        }
   
      // If the scheduler was started then the following line should never be
      // reached because vTaskStartScheduler() will only return if there was not
      // enough FreeRTOS heap memory available to create the Idle and (if configured)
      // Timer tasks.  Heap management, and techniques for trapping heap exhaustion,
      // are described in the book text.
      for( ; ; ) { }
      }   // End of loop to check switches
    
    }   // End of Big Loop

}

// ********************Functions******************************
//
// ***********************************************************
// FreeRTOS Tasks
// ***********************************************************
// The blinking LED function; Pico default LED
void vTaskBlinkDefault() {
   for (;;) {
      gpio_put(PICO_DEFAULT_LED_PIN, 1);
      vTaskDelay(TaskDEFLTLED_DLY);        // FreeRTOS delay
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
      vTaskDelay(TaskDEFLTLED_DLY);        // FreeRTOS delay
   }
}

// SSD1306 display Task
// TODO: May have to replace sleep_ms() delays with vTaskDelay() in
// functions called within this task. For example: 
// 1) ClearDisplay()
// 2) WriteString()
// 3) WriteBigChar()
// 4) UpdateDisplay()
// But it seems to be working without replacing sleep_ms() in the
// SSD1306 functions. Another concern is the I2C interface. A 
// blocking function is called - i2c_write_blocking(). The function
// waits until the external I2C device responses or a timeout expires.
// Ideally, the funciton should release the processor while it waits
// for the external device.
static void vTaskSSD1306() {
   for (;;) {
	 // send patterns to SSD1306 display
     ClearDisplay();
	 // send short message to SSD1306
	 ssd1306_mess_short(true);   // use FreeRTOS delay function
   }
}

// *********************************
// Software Timer Callback Function
// *********************************
// This function is called everytime the software timer expires. The 
// function determines which of the 3 timers expired and executes code 
// for that software timer.
// Timers:
// 1) one-shot for testing  (one-shot)
// 2) Yellow blinking LED (auto-reload)
// 3) 7-segment display (auto-reload)
static void prvTimerCallback( TimerHandle_t xTimer ) {
    TickType_t xTimeNow;
    uint32_t ulExecutionCount;
    
    bool ylwLedStat = 0;

    /* The count of the number of times this software timer has expired is
     * stored in the timer's ID.  Obtain the ID, increment it, then save it as the
     * new ID value.  The ID is a void pointer, so is cast to a uint32_t. */
    ulExecutionCount = ( uint32_t ) pvTimerGetTimerID( xTimer );
    ulExecutionCount++;
    vTimerSetTimerID( xTimer, ( void * ) ulExecutionCount );

    // Obtain the current tick count.
    xTimeNow = xTaskGetTickCount();

    /* The handle of the one-shot timer was stored in xOneShotTimer when the
     * timer was created.  Compare the handle passed into this function with
     * xOneShotTimer to determine if it was the one-shot or auto-reload timer that
     * expired, then output a string to show the time at which the callback was
     * executed. */
    // *****************************
    // xTimer for one-shot (just for testing)
    if( xTimer == xOneShotTimer ) {
	  // FreeRTOS protected version of printf()
//        ab_vPrintStringAndNumber( "One-shot timer callback executing", xTimeNow );
      uart_puts(UART_ID, "One-shot timer callback executing\n\r");
    }
    // *****************************
    // xTimer for yellow blinking LED timer
    else if( xTimer == xYlwLedReloadTimer ) {
//	ab_vPrintStringAndNumber( "Blink Yellow LED", xTimeNow );
	  ylwLedStat = gpio_get(YLW_LED);  // get LED status
	  gpio_put(YLW_LED, !ylwLedStat);  // toggle LED
    }
    // *****************************
    // xTimer for 7-segment Display
    else {
        // xTimer for 7-segment display.
//        ab_vPrintStringAndNumber( "7-Segment display timer callback executing", xTimeNow );
	
      gpio_put(RCLK_GPIO, 0);
//        printf("seg7num = %x\n", seg7num[ulExecutionCount]);   // debug
	  // Note: frShiftOut() is FreeRTOS safe because FreeRTOS delays 
	  // are used in the fuction.
	  seg7_chr = get_seg7num(seg7NumIndx & 0x0f);  // get 7-segment encoded char; keep lower 4 bits
	  frShiftOut(SER_GPIO, SRCLK_GPIO, MSBFIRST, seg7_chr );
	  seg7NumIndx++;	
      gpio_put(RCLK_GPIO, 1);  // send clock pulse to output register	

      if( ulExecutionCount == 15 )  {
            /* Stop the auto-reload timer after it has executed 5 times.  This
             * callback function executes in the context of the RTOS daemon task so
             * must not call any functions that might place the daemon task into
             * the Blocked state.  Therefore a block time of 0 is used. */
//            xTimerStop( xTimer, 0 );
		
      }
    }
}


// *********************************
// UART RX Callback Function
// *********************************

// RX interrupt handler
// IMPORTANT NOTE: The interrupt is automatically cleared when we read from the data register
//                 or the Fifo is read out to the water-mark. So, no need to clear the interrupt.
static void vUartRxInterruptHandler() {
    // Debug
//    uart_putc(UART_ID, 'G');
//    uart_putc(UART_ID, '\n');
//    uart_putc(UART_ID, '\r');

    BaseType_t xHigherPriorityTaskWoken;
    // The xHigherPriorityTaskWoken parameter must be initialized to
    // pdFALSE as it will get set to pdTRUE inside the interrupt safe
    // API function if a context switch is required.
    xHigherPriorityTaskWoken = pdFALSE;    
    
    // Send a notification to the handler task.  The first will unblock the 
    // task, the following 'gives' are to demonstrate that the receiving 
    // task's notification value is being used to latch events - allowing
    // the task to process the events in turn.
    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
    // The next two 'Give' notifications are for testing only. They work correctly.
//    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
//    vTaskNotifyGiveFromISR( xUartRXHandlerTask, &xHigherPriorityTaskWoken );
    
    while (uart_is_readable(UART_ID)) {     // do we have RX data?
        uint8_t ch = uart_getc(UART_ID);    // read from UART0 data register
        
        // Can we send it back?
        if (uart_is_writable(UART_ID)) {    // is the UART0 data register available
            // Change it slightly first!
            ch++;
            uart_putc(UART_ID, ch);
        }
    }
         
    // ab: delete, not used???????
//   portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// ************************
// RX UART Deferred Interrupt Handler Task
// Use Notification instead of semaphore for RX UART interrupt
static void vUartRxDeferredIntrHandlerTask( void * pvParameters ) {
    
    // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from the
        // interrupt handler.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake()
        // returns having received a notification.
//        if( ulTaskNotifyTake( pdFALSE, xMaxExpectedBlockTime ) != 0 ) {
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {

        // To get here the event must have occurred.  Process the event (in this
        // case just print out a message).
        
        // Debug
        uart_putc(UART_ID, 'D');
        uart_putc(UART_ID, '\n');
        uart_putc(UART_ID, '\r');

#ifdef PRINT_MESS
        uart_puts(UART_ID, "UART RX Deferred Interrupt Handler task - Processing event.\r\n");
//        ab_vPrintString( "UART RX Deferred Interrupt Handler task - Processing event.\r\n" );
#endif
        }
        else {
            uart_puts(UART_ID, "UART RX Deferred Task Time-Out.\r\n");
        }
        
//        alarm_fired = false;
    }
}
