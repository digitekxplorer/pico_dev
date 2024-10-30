// June 15, 2024
// Support functions for the Combo project that includes:
// 1) blinking LEDs
// 2) 7-segment display
// 3) SSD1306 display
// 4) FreeRTOS supporting functions

// ****************************
// SSD1306 Display
// ****************************
// Public Functions
// Send commands over I2C to intialize SSD1306
void SSD1306_initialize();
// Invert the display
void InvertDisplay(bool yes);
// Copy the entire framebuffer to the display.
//void UpdateDisplay();
void UpdateDisplay();
// Clear SSD1306 Display
void ClearDisplay();
// Basic Bresenhams.
void DrawLine(int x0, int y0, int x1, int y1, bool on);
// Write one Big Character at location x,y
void WriteBigChar(uint x, uint y, uint8_t ch);
// Write string starting at location x,y
void WriteString(int x, int y, uint8_t *str);
// Write Big tring starting at location x,y
void WriteBigString(int x, int y, uint8_t *str);
// Write one character at location x,y
void WriteChar(uint x, uint y, uint8_t ch);
// setup SSD1306 Display
void setup_ssd1306(void);
// Short message displayed on SSD1306
void ssd1306_mess_short(bool isFreeRTOS_enabled);
// Long message displayed on SSD1306
void ssd1306_mess_long(void);

// Private Functions
// Send a command over I2C to SSD1306
static void SendCommand(uint8_t cmd);
// Send the command buffer over I2C to SSD1306
static void SendCommandBuffer(uint8_t *inbuf, int len);
// Access one pixel at the x,y location
static void SetPixel(int x,int y, bool on);
//
static uint8_t reverse(uint8_t b) ;
//
static inline int GetFontIndex(uint8_t ch);
//
static void FillReversedCache();
//
static uint16_t ExpandByte(uint8_t b);
