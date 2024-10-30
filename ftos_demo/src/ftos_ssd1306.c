// June 15, 2024
// Support functions for the Combo project that includes:
// 1) blinking LEDs
// 2) 7-segment display
// 3) SSD1306 display
// 4) FreeRTOS supporting functions

// NOTES:
// In the original design, all SSD1306 funcitons were decleared as 
// "static".
// When static is used in the context of a function, like 
// "static void ClearDisplay()", static means that the function has 
// internal linkage.  In other words, static specifies that the function
// is only visible within the current translation unit (i.e., the 
// current source file) and cannot be called from outside the file.
// The function cannot be called from another file, even if the 
// function is declared in a header file.
// In essence, static makes the function local to the file, and 
// it's not visible to the outside world.

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

//
#include "ftos_ssd1306.h"
#include "defs.h"


/* Example code to talk to a SSD1306 OLED display, 128 x 64 pixels
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.
   Connections on Raspberry Pi Pico board, other boards may vary.
*/


// Not currently used.
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10

#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_DEACTIVATE_SCROLL   0x2E
#define SSD1306_ACTIVATE_SCROLL     0x2F

#define SSD1306_SETSTARTLINE        0x40

#define SSD1306_SETCONTRAST         0x81
#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_SEGREMAP0           0xA0
#define SSD1306_SEGREMAP127         0xA1
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF

#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB

#define TaskSSD1306_DLY             500


// ****************************
// SSD1306 Display
// ****************************

// We need a 0x40 in the byte before our framebuffer
uint8_t _Framebuffer[SSD1306_FRAMEBUFFER_SIZE + 1] = {0x40};
uint8_t *Framebuffer = _Framebuffer+1;

static uint8_t reversed[sizeof(font)] = {0};

// ***************************************************
// Public Functions
// ***************************************************
// Non-static functions that can be called from main.c

//static void SSD1306_initialize() {
void SSD1306_initialize() {
uint8_t init_cmds[]=
    {
    0x00,                // send one command
//    0x80,    // send multiple commands; didn't work????
    SSD1306_DISPLAYOFF,
    SSD1306_SETMULTIPLEX, 0x3f,
    SSD1306_SETDISPLAYOFFSET, 0x00,
    SSD1306_SETSTARTLINE,
    SSD1306_SEGREMAP127,
    SSD1306_COMSCANDEC,
    SSD1306_SETCOMPINS, 0x12,
    SSD1306_SETCONTRAST, 0xff,
    SSD1306_DISPLAYALLON_RESUME,
    SSD1306_NORMALDISPLAY,
    SSD1306_SETDISPLAYCLOCKDIV, 0x80,
    SSD1306_CHARGEPUMP, 0x14,
    SSD1306_DISPLAYON,
    SSD1306_MEMORYMODE, 0x00,   // 0 = horizontal, 1 = vertical, 2 = page
    SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1,  // Set the screen wrapping points
    SSD1306_PAGEADDR, 0, 7};

    SendCommandBuffer(init_cmds, sizeof(init_cmds));
}

//static void InvertDisplay(bool yes) {
void InvertDisplay(bool yes) {
    if (yes)
        SendCommand(SSD1306_INVERTDISPLAY);
    else
        SendCommand(SSD1306_NORMALDISPLAY);
}

// This copies the entire framebuffer to the display.
//static void UpdateDisplay() {
void UpdateDisplay() {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, _Framebuffer, sizeof(_Framebuffer), false);
}

//static void ClearDisplay() {
void ClearDisplay() {
    memset(Framebuffer, 0, SSD1306_FRAMEBUFFER_SIZE);
    UpdateDisplay();
}

// Basic Bresenhams.
//static void DrawLine(int x0, int y0, int x1, int y1, bool on) {
void DrawLine(int x0, int y0, int x1, int y1, bool on) {
    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        SetPixel(x0, y0, on);

        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

//static void WriteBigChar(uint x, uint y, uint8_t ch) {
void WriteBigChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        uint16_t w = ExpandByte(reversed[idx * 8 + i]);
        Framebuffer[fb_idx] = w & 0x0ff;
        Framebuffer[fb_idx+1] = w & 0x0ff;
        Framebuffer[fb_idx+128] = w >> 8;
        Framebuffer[fb_idx+129] = w >> 8;
        fb_idx+=2;

    }
}

//static void WriteString(int x, int y, uint8_t *str) {
void WriteString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    while (*str) {
        WriteChar(x,y, *str++);
        x+=8;
    }
}

//static void WriteBigString(int x, int y, uint8_t *str) {
void WriteBigString(int x, int y, uint8_t *str) {
    // Cull out any string off the screen
    if (x > SSD1306_LCDWIDTH - 16 || y > SSD1306_LCDHEIGHT - 16)
        return;

    while (*str) {
        WriteBigChar(x,y, *str++);
        x+=16;
    }
}

//static void WriteChar(uint x, uint y, uint8_t ch) {
void WriteChar(uint x, uint y, uint8_t ch) {
    if (reversed[0] == 0)
        FillReversedCache();

    if (x > SSD1306_LCDWIDTH - 8 || y > SSD1306_LCDHEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) {
        Framebuffer[fb_idx++] = reversed[idx * 8 + i];
    }
}

// setup SSD1306 Display
void setup_ssd1306(void) {
    i2c_init(I2C_PORT, I2C_CLOCK * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);  // (4, 3)
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);  // (5, 3)
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    SSD1306_initialize();         // initialize SSD1306
    ClearDisplay();               // clear SSD1306 display
}

// Short message displayed on SSD1306
// Use FreeRTOS delay function if this function is called after 
// scheduler has been enabled.
void ssd1306_mess_short(bool isFreeRTOS_enabled) {
    //
    DrawLine(0,0,SSD1306_LCDWIDTH-1, 0, true);
    DrawLine(0,0,0,SSD1306_LCDHEIGHT-1, true);
    DrawLine(SSD1306_LCDWIDTH-1, 0,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
    DrawLine(0,SSD1306_LCDHEIGHT-1,SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, true);
    
    if (isFreeRTOS_enabled) {
        WriteString(26,8,"FreeRTOS");  // send FreeRTOS message
        WriteString(16,22,"is Enabled!");
    } else {
        WriteString(13,8,"HELLO SSD1306");
    }

    for (int ch = '9'; ch>='0';ch--) {
        WriteBigChar(56, 36, ch);
        UpdateDisplay();
        if (isFreeRTOS_enabled) {
            vTaskDelay(TaskSSD1306_DLY);  // FreeRTOS delay
        } else {
            sleep_ms(500);                // Pico SDK delay
        }
    }
    
    // short delay at end of count-down
    if (isFreeRTOS_enabled) {
        vTaskDelay(TaskSSD1306_DLY);  // FreeRTOS delay
    } else {
        sleep_ms(500);                // Pico SDK delay
    }
}

// Long message displayed on SSD1306
void ssd1306_mess_long(void) {
    // Call short message then continue with more
    ssd1306_mess_short(false);   // use Pico SDK delay function
    // Continue with longer message
    WriteBigString(0,35,"BLASTOFF");
    UpdateDisplay();

    for (int i = 0;i<10;i++) {
        InvertDisplay(true);
        sleep_ms(100);
        InvertDisplay(false);
        sleep_ms(100);
    }

    for (int i=0;i<SSD1306_LCDWIDTH;i++) {
        if (i >= SSD1306_LCDWIDTH/2) {
            DrawLine(i,0,i, SSD1306_LCDHEIGHT-1, false);
            DrawLine(SSD1306_LCDWIDTH - i - 1,0,SSD1306_LCDWIDTH - i - 1, SSD1306_LCDHEIGHT-1, false);
        }
        else {
            DrawLine(i,0,i, SSD1306_LCDHEIGHT-1, true);
            DrawLine(SSD1306_LCDWIDTH - i - 1,0,SSD1306_LCDWIDTH - i - 1, SSD1306_LCDHEIGHT-1,  true);
        }
        UpdateDisplay();
    }
}

// ***************************************************
// Private Functions  (static functions)
// ***************************************************
static void SendCommand(uint8_t cmd) {
//void SendCommand(uint8_t cmd) {
    uint8_t buf[] = {0x00, cmd};
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, buf, 2, false);
}

static void SendCommandBuffer(uint8_t *inbuf, int len) {
//void SendCommandBuffer(uint8_t *inbuf, int len) {
    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, inbuf, len, false);
}

static void SetPixel(int x,int y, bool on) {
//void SetPixel(int x,int y, bool on) {
    assert(x >= 0 && x < SSD1306_LCDWIDTH && y >=0 && y < SSD1306_LCDHEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = 128; // 128 pixels, 1bpp, but each row is 8 pixel high, so (128 / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow   +   x;
    uint8_t byte = Framebuffer[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    Framebuffer[byte_idx] = byte;
}

static uint8_t reverse(uint8_t b) {
//uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

static inline int GetFontIndex(uint8_t ch) {
//inline int GetFontIndex(uint8_t ch) {
    if (ch >= 'A' && ch <='Z')
        return  ch - 'A' + 1;
    else if (ch >= '0' && ch <='9')
        return  ch - '0' + 27;
    else
        return  0; // Not got that char so space.
}

//static uint8_t reversed[sizeof(font)] = {0};

static void FillReversedCache() {
//void FillReversedCache() {
    // calculate and cache a reversed version of fhe font, because I defined it upside down...doh!
    for (int i=0;i<sizeof(font);i++)
        reversed[i] = reverse(font[i]);
}

//uint16_t ExpandByte(uint8_t b) {
static uint16_t ExpandByte(uint8_t b) {
    uint16_t w = 0;
    for (int i=7;i>=0;i--) {
        uint16_t t = (b & (1 << i));
        w |= (t << i);
        w |= (t << (i + 1));
    }
    return w;
}
