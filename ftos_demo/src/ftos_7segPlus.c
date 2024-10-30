// June 15, 2024
// Support functions for the Combo project that includes:
// 1) 7-segment display
// 2) FreeRTOS supporting ab_vPrint functions

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "defs.h"


//encoding for character 0-F of common anode SevenSegmentDisplay. 
unsigned char seg7num[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};

// ****************************
// 7-Segment Display
// ****************************
// Function to shift out 8 bits of each digit without FreeRTOS
void shiftOut(int dPin, int cPin, int order, int val){   
    // dPin=serial data; cPin=SRCLK; order=msb; val=digit in binary
    int jj;
    bool out_bit;  // shifted output bit
    // shift out 8 bits of the digit
    for(jj = 0; jj < 8; jj++){
        gpio_put(cPin, 0);
        out_bit = ((0x01&(val>>jj)) == 0x01) ? HIGH : LOW;
        sleep_ms(0.01);
        if(order == LSBFIRST){
            out_bit = ((0x01&(val>>jj)) == 0x01) ? HIGH : LOW;  // get LSB
            gpio_put(dPin, out_bit);
            sleep_ms(0.01);
	}
        else {// if(order == MSBFIRST)
            out_bit = ((0x80&(val<<jj)) == 0x80) ? HIGH : LOW;   // get MSB
            gpio_put(dPin, out_bit);
            sleep_ms(0.01);
	}
        gpio_put(cPin, 1);
        sleep_ms(0.01);
    }
}

// Function to shift out 8 bits of each digit WITH FreeRTOS
// Replaced sleep_ms() with vTaskDelay()
void frShiftOut(int dPin, int cPin, int order, int val){   
    // dPin=serial data; cPin=SRCLK; order=msb; val=digit in binary
    int jj;
    bool out_bit;  // shifted output bit
    // shift out 8 bits of the digit
    for(jj = 0; jj < 8; jj++){
        gpio_put(cPin, 0);
        out_bit = ((0x01&(val>>jj)) == 0x01) ? HIGH : LOW;
	vTaskDelay(1);        // FreeRTOS delay
        if(order == LSBFIRST){
            out_bit = ((0x01&(val>>jj)) == 0x01) ? HIGH : LOW;  // get LSB
            gpio_put(dPin, out_bit);
	    vTaskDelay(1);        // FreeRTOS delay
	}
        else {// if(order == MSBFIRST)
            out_bit = ((0x80&(val<<jj)) == 0x80) ? HIGH : LOW;   // get MSB
            gpio_put(dPin, out_bit);
	    vTaskDelay(1);        // FreeRTOS delay
	}
        gpio_put(cPin, 1);
	vTaskDelay(1);        // FreeRTOS delay
    }
}

// Write sequence of numbers to display
void wr_num_dsply(void) {
    // write digits to seven segment display
    int kk;
    for(kk=0; kk<16; kk=kk+1) {
        gpio_put(RCLK_GPIO, 0);
        printf("seg7num = %x\n", seg7num[kk]);
        shiftOut(SER_GPIO, SRCLK_GPIO, MSBFIRST, seg7num[kk]);
        gpio_put(RCLK_GPIO, 1);  // send clock pulse to output register
        sleep_ms(1000);
    }
}

// Clear seven segment
void clr_7seg_dsply(void) {
    // clear seven segment display
    gpio_put(RCLK_GPIO, 0);
    shiftOut(SER_GPIO, SRCLK_GPIO, MSBFIRST, 0xff);
    gpio_put(RCLK_GPIO, 1);  // send clock pulse to output register
    sleep_ms(1000);
}

// Clear control signals
void clr_ctrls(void) {
    gpio_put(SER_GPIO, 0);
    gpio_put(RCLK_GPIO, 0);
    gpio_put(SRCLK_GPIO, 0);
}

// Get 7-Segment encoded character
unsigned char get_seg7num(uint8_t seg7NumIndx) {
    return seg7num[seg7NumIndx];
}

// Setup for 7-Segment Display
void setup_7seg(void) {
    gpio_init(SER_GPIO);
    gpio_set_dir(SER_GPIO, GPIO_OUT);

    gpio_init(RCLK_GPIO);
    gpio_set_dir(RCLK_GPIO, GPIO_OUT);

    gpio_init(SRCLK_GPIO);
    gpio_set_dir(SRCLK_GPIO, GPIO_OUT);
    
    clr_7seg_dsply();
    clr_ctrls();
}

// ****************************
// ab FreeRTOS Supporting Functions
// ****************************
// These functions were copied from supporting_function.c and
// modified to run in this application
// ************
void ab_vPrintString( const char *pcString )
{
BaseType_t xKeyHit = pdFALSE;
    /* Print the string, using a critical section as a crude method of mutual
    exclusion. */
    taskENTER_CRITICAL();
    {
	printf( "%s", pcString );
	fflush( stdout );
    }
    taskEXIT_CRITICAL();
}


// ************
void ab_vPrintStringAndNumber( const char *pcString, uint32_t ulValue )
{
    /* Print the string, using a critical section as a crude method of mutual
    exclusion. */
    taskENTER_CRITICAL();
    {
	printf( "%s %lu\r\n", pcString, ulValue );
	fflush( stdout );
    }
    taskEXIT_CRITICAL();
}
