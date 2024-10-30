// June 17, 2023
// Display Combination Project
// Definitions for Combo

// ***************************
// Pico UART
// ***************************
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// ***************************
// Pico default pins are GP4 SDA0 and GP5 SCL0
// ***************************
// Used to communicate with SSD1306
#define I2C_PORT i2c_default
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// ***************************
// LEDs and Switches
// ***************************
#define YLW_LED  10
#define SW0_GPIO 12
#define SW1_GPIO 13

// ***************************
// 7-Segment Display
// ***************************
// Three control signals needed for 74HC595:
// SER: serial data;             Pin of 74HC595(Pin14); Pico GPIO6
// RCLK: output register clock;  Pin of 74HC595(Pin12): Pico GPIO7
// SRCLK: serial register clock; Pin of 74HC595(Pin11): Pico GPIO8
// March 21/2024 Moved to different Pico pins to avoid confict
// with SSD1306_bigfont project pin assignments.
#define SER_GPIO 6
#define RCLK_GPIO 7
#define SRCLK_GPIO 8
#define BUTTON_GPIO 9
// 
#define HIGH 1
#define LOW  0
#define LSBFIRST 0
#define MSBFIRST 1

// ***************************
// SSD1306 Display
// ***************************
/* Example code to talk to a SSD1306 OLED display, 128 x 64 pixels
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.
   Connections on Raspberry Pi Pico board, other boards may vary.
*/

// By default these devices are on bus address 0x3C or 0x3D. Check your documentation.
static int DEVICE_ADDRESS = 0x3C;

 // This can be overclocked, 2000 seems to work on the device being tested
 // Spec says 400 is the maximum. Try faster clocks until it stops working!
 // KHz.
#define I2C_CLOCK  1000

#define SSD1306_LCDWIDTH            128
#define SSD1306_LCDHEIGHT           64
#define SSD1306_FRAMEBUFFER_SIZE    (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)
