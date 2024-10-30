// June 15, 2024
// Support functions for the Combo project that includes:
// 1) 7-segment display
// 2) FreeRTOS supporting ab_vPrint functions


// ****************************
// 7-Segment Display
// ****************************
// 7-segment shift out byte to 74HC595
void shiftOut(int dPin, int cPin, int order, int val);
// FreeRTOS shift out
void frShiftOut(int dPin, int cPin, int order, int val);
// Clear seven segment
void clr_7seg_dsply(void);
// Write sequence of numbers to display
void wr_num_dsply(void);
// Clear control signals
void clr_ctrls(void);

// ab: Get 7-Segment encoded character
unsigned char get_seg7num(uint8_t seg7NumIndx);

// Setup for 7-Segment Display
void setup_7seg(void);


// ****************************
// ab FreeRTOS Supporting Functions
// ****************************
void ab_vPrintString( const char *pcString );

void ab_vPrintStringAndNumber( const char *pcString, uint32_t ulValue );
