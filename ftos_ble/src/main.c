// September 21, 2024
// Pico W project to create a build with FreeRTOS and BTstack (Bluetooth LE)
// Testbed to start building a FreeRTOS project with BLE.
// Project plan:
// 1) Created FreeRTOS to blink an LED with Pico W
// 2) Added Bluekitchen's BtStack 
// 3) Added characteric function to turn ON and OFF separate LED
// This is a generic FreeRTOS/BLE project that can be used as a basis for more
// complex Pico W development.
//
// An Android App is needed to communicate with Pico's BTstack (Bluetooth).
// The Android App LightBlue by PunchThrough was used in this project.
// Currentlly, the Pico will respond to 2 Client commands from the LightBlue
// App: "fwd" and "hlt". These two commands are for testing the BTstack and
// the Pico will respond by toggling an IO pin connected to an LED.

// Useful commands:
// openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ftos_ble.elf reset exit"
// minicom -b 115200 -o -D /dev/ttyACM0

// Useful links:
// https://www.raspberrypi.com
// https://github.com/raspberrypi
// https://freertos.org/Documentation/RTOS_book.html
// https://bluekitchen-gmbh.com/btstack
// https://punchthrough.com/lightblue

// Releases:
// 10/29/2024  A. Baeza  Initial release: V1.0.0

// Important Note:
// See a description of this BTstack build process in Hunter Adams’ webpage:
// https://vanhunteradams.com/Pico/BLE/GATT_Server.html

// September 23, 2024
// BTstack is working with FreeRTOS. Able to connect to LightBlue app on my phone. 
// Able to 'write' message to Pico and toggle LED on GP15. Pico W locks-up after
// a few minutes of operation.
// September 24, 2024
// Fixed error in serive_implementation.h; commented-out lines#200-206, characteristic_f not used.
// Added ftos_tasks.h with BLE input to move vehicle forward Handler Task; vBLEinput_HandlerTask( ).
// Also added pico_init.c and defs.h.
// September 25, 2024
// Moved Client command comparison to vBLEinput_HandlerTask() in ftos_tasks.h to perform the actual
// command request outside of Bluetooth functions in service_implementation.h.
// September 28, 2024
// Code cleanup.
// October 29, 2024
// Moved Client command input task vBLEinput_HandlerTask() back to main.c


// Pico
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
// BTstack
#include "btstack.h"
// High-level libraries
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include "ftos_tasks.h"
#include "defs.h"
#include "pico_init.h"

// GAP and GATT
#include "GAP_Advertisement/gap_config.h"
#include "GATT_Service/service_implementation.h"
// Period with which we'll enter the BTstack timer callback
#define HEARTBEAT_PERIOD_MS 250

// BTstack objects
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Some data that we will communicate over Bluetooth
static int hb_counter = 0 ;

// We send data as formatted strings (just like a serial console)
static char characteristic_a_tx[100] ;
static char characteristic_b_rx[100] ;      // commands from client (phone) placed here
static char characteristic_c_tx[5] ;

// FreeRTOS Tasks
// BLE input to move vehicle forward Handler Task
static void vBLEinput_HandlerTask( void * pvParameters );
// The blinking LED functions
void vLedDefltBlinkTask();
void vLedEx2BlinkTask();
// GATT Server task
static void bleServerTask(void *pv);

// Main
void main() {
    stdio_init_all();
    
    // Setup basic pico IO; LEDs and user switches
    pico_io();

    print_welcome();     // ab print delays and hello using PuTTY

    // initialize Infineon CYW43439 wireless IC
    if (cyw43_arch_init()) {
        printf("CYW43439 init failed");
        return;
    }
   
    // create a FreeRTOS tasks
    xTaskCreate(vLedDefltBlinkTask, "Blink Pico LED Task", 128, NULL, 1, NULL);   // Pico LED
    // LED_EX1 used for BTstack LED "LED Status and Control" characteristic_e        LED_EX1
    xTaskCreate(vLedEx2BlinkTask, "Blink LED_EX2 Task", 128, NULL, 1, NULL);      // LED_EX2
    
    // Bluetooth Low Energy (BLE)
    //
    // BLE Characteristics tasks
    xTaskCreate( vBLEinput_HandlerTask, "Characteristic_d Handler", 1000, NULL, 3, &xBLEinput_HandlerTask );

    // Task to define BLE and to execute the BTstack run_loop()
    xTaskCreate(
      bleServerTask,            // pointer to the task
      "BLEserver",              // task name for kernel awareness debugging
      1200/sizeof(StackType_t), // task stack size
      (void*)NULL,              // optional task startup argument
      tskIDLE_PRIORITY+2,       // initial priority
      (TaskHandle_t*)NULL       // optional task handle to create
    );
    
    // start the FreeRTOS scheduler
    vTaskStartScheduler();
}

// *********************************************************
// *********************************************************
// BLE GATT Server task
static void bleServerTask(void *pv) {
  l2cap_init(); // Set up L2CAP and register L2CAP with HCI layer
  sm_init(); // setup security manager
  
  // Initialize ATT server, no general read/write callbacks
  // because we'll set one up for each service
  att_server_init(profile_data, NULL, NULL);   

  // Instantiate our custom service handler
  custom_service_server_init( characteristic_a_tx, characteristic_b_rx,
                              characteristic_c_tx) ;

  // inform about BTstack state
  hci_event_callback_registration.callback = &packet_handler; // setup callback for events
  hci_add_event_handler(&hci_event_callback_registration); // register callback handler

  // register for ATT event
  att_server_register_packet_handler(packet_handler); // register packet handler

  hci_power_control(HCI_POWER_ON); // turn BLE on
  
  for(;;) {
    btstack_run_loop_execute(); // does not return
  }
}

// BLE command input Handler Task
// Use Notification instead of semaphore 
static void vBLEinput_HandlerTask( void * pvParameters ) {
    // Implement this task within an infinite loop.
    for( ; ; ) {       
        // Wait to receive a notification sent directly to this task from custom_service_write_callback()
        // function in service_implementation.h.  The xClearCountOnExit parameter is now pdFALSE, so
        // the task's notification will be decremented when ulTaskNotifyTake() returns having received 
        // a notification.
        if( ulTaskNotifyTake( pdFALSE, portMAX_DELAY ) != 0 ) {
          // To get here the event must have occurred.

          printf("Task_Handler: Text command from client: %s.\n", ble_input);   // cmd buffer defined in defs.h
          
          if (!strcmp(ble_input, "fwd")) {
            gpio_put(LED_EX3, 1);                            // turn on exteranl LED#3
          }
          else if (!strcmp(ble_input, "hlt")) {
            gpio_put(LED_EX3, 0);                            // turn off exteranl LED#3
          }
          else {
            // do nothing
          }
        }  // end of notification loop         
    }  // end of for loop 
}  // end handler task

// *************************
// The blinking LED functions
// *************************
// Pico's default LED
void vLedDefltBlinkTask() {
    
   for (;;) {
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      vTaskDelay(HEARTBEAT_PERIOD_MS);    // FreeRTOS delay
      watchdog_update();                  // Watchdog timer reset
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      vTaskDelay(HEARTBEAT_PERIOD_MS);    // FreeRTOS delay
      watchdog_update();                  // Watchdog timer reset
    
      // heartbeat;  Increment the counter
      hb_counter += 1 ;
      // Update characteristic (sends to client if notifications enabled)
      set_characteristic_a_value(hb_counter) ;
   }
}

// Exteranl LED#2
void vLedEx2BlinkTask() {
   for (;;) {
      gpio_put(LED_EX2, 1);
      vTaskDelay(500);                    // FreeRTOS delay
      gpio_put(LED_EX2, 0);
      vTaskDelay(500);                    // FreeRTOS delay
   }
}
