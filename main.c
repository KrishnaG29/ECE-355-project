//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include "diag/Trace.h"
#include <string.h>

#include "cmsis/cmsis_device.h"
#include "stm32f0xx_hal.h"

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=48000000.
//
/// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/*** This is partial code for accessing LED Display via SPI interface. ***/

// Connections: PA1 = POT, PA4 = DAC, PA0 = USER button.
// PB2 = generator, PB3 = NE555.
// OLED: PB8 = CS, PB9 = DC, PB11 = RESET, PB13 = SCK, PB15 = MOSI.

// These are the same calibration values as the working program.
#define POT_TOTAL_OHMS    5000U
#define ADC_REFERENCE_MV 2960U
#define POT_SUPPLY_MV    3310U
#define FREQUENCY_LINES ((1U << 2) | (1U << 3))

unsigned int Freq = 0;
unsigned int Res = 0;
volatile uint8_t selectedSource = 0;  // 0 = generator, 1 = NE555.

// Shared by main and the frequency interrupt handlers.
enum { MEASURE_IDLE, WAIT_FIRST, WAIT_SECOND, MEASURE_DONE,
       MEASURE_TIMEOUT, MEASURE_CANCELLED };
static volatile uint32_t measureState = MEASURE_IDLE;
static volatile uint32_t measuredTicks = 0;
static volatile uint8_t measuredSource = 0;
static uint32_t timerClockHz;

enum { DISPLAY_WAITING, DISPLAY_VALID, DISPLAY_TIMEOUT };
static uint8_t displayStatus = DISPLAY_WAITING;
static uint8_t displaySource = 0;
static uint8_t potAtLimit = 0;
static char previousLine[8][17];  // Remember rows to avoid unnecessary redraws.
SPI_HandleTypeDef SPI_Handle;

void SystemClock48MHz(void);
void myGPIOA_Init(void);
void myADC_Init(void);
uint16_t myADC_Read(void);
void myDAC_Init(void);
void myDAC_Write(uint16_t value);
void myPot_Update(uint16_t adcValue);
void myFrequency_Init(void);
void myFrequency_Start(void);
void myFrequency_Stop(void);
void myLED_Init(void);
void myLED_Update(void);
void myButton_Init(void);
void myButton_Service(void);
void myTIM3_Init(void);
void delay_ms(uint16_t milliseconds);
void myOLED_GPIO_Init(void);
void oled_config(void);
void oled_Write(unsigned char Value);
void oled_Write_Cmd(unsigned char cmd);
void oled_Write_Data(unsigned char data);
void oled_PrintLine(uint8_t page, const char *text);
void refresh_OLED(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void TIM2_IRQHandler(void);



//
// LED Display initialization commands
//
const unsigned char oled_init_cmds[] =
{
    0xAE,
    0x20, 0x00,
    0x40,
    0xA0 | 0x01,
    0xA8, 0x40 - 1,
    0xC0 | 0x08,
    0xD3, 0x00,
    0xDA, 0x32,
    0xD5, 0x80,
    0xD9, 0x22,
    0xDB, 0x30,
    0x81, 0xFF,
    0xA4,
    0xA6,
    0xAD, 0x30,
    0x8D, 0x10,
    0xAE | 0x01,
    0xC0,
    0xA0
};


//
// Character specifications for LED Display (1 row = 8 bytes = 1 ASCII character)
// Example: to display '4', retrieve 8 data bytes stored in Characters[52][X] row
//          (where X = 0, 1, ..., 7) and send them one by one to LED Display. 
// Row number = character ASCII code (e.g., ASCII code of '4' is 0x34 = 52)
//
const unsigned char Characters[][8] = {
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // SPACE
    {0b00000000, 0b00000000, 0b01011111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // !
    {0b00000000, 0b00000111, 0b00000000, 0b00000111, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // "
    {0b00010100, 0b01111111, 0b00010100, 0b01111111, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // #
    {0b00100100, 0b00101010, 0b01111111, 0b00101010, 0b00010010,0b00000000, 0b00000000, 0b00000000},  // $
    {0b00100011, 0b00010011, 0b00001000, 0b01100100, 0b01100010,0b00000000, 0b00000000, 0b00000000},  // %
    {0b00110110, 0b01001001, 0b01010101, 0b00100010, 0b01010000,0b00000000, 0b00000000, 0b00000000},  // &
    {0b00000000, 0b00000101, 0b00000011, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // '
    {0b00000000, 0b00011100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // (
    {0b00000000, 0b01000001, 0b00100010, 0b00011100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // )
    {0b00010100, 0b00001000, 0b00111110, 0b00001000, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // *
    {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // +
    {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ,
    {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // -
    {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // .
    {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // /
    {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // 0
    {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // 1
    {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // 2
    {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // 3
    {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000,0b00000000, 0b00000000, 0b00000000},  // 4
    {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001,0b00000000, 0b00000000, 0b00000000},  // 5
    {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000,0b00000000, 0b00000000, 0b00000000},  // 6
    {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // 7
    {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // 8
    {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110,0b00000000, 0b00000000, 0b00000000},  // 9
    {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // :
    {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // ;
    {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // <
    {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100,0b00000000, 0b00000000, 0b00000000},  // =
    {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // >
    {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // ?
    {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // @
    {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110,0b00000000, 0b00000000, 0b00000000},  // A
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110,0b00000000, 0b00000000, 0b00000000},  // B
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010,0b00000000, 0b00000000, 0b00000000},  // C
    {0b01111111, 0b01000001, 0b01000001, 0b00100010, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // D
    {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // E
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // F
    {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010,0b00000000, 0b00000000, 0b00000000},  // G
    {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // H
    {0b01000000, 0b01000001, 0b01111111, 0b01000001, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // I
    {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // J
    {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001,0b00000000, 0b00000000, 0b00000000},  // K
    {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // L
    {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // M
    {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // N
    {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // O
    {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110,0b00000000, 0b00000000, 0b00000000},  // P
    {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110,0b00000000, 0b00000000, 0b00000000},  // Q
    {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110,0b00000000, 0b00000000, 0b00000000},  // R
    {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001,0b00000000, 0b00000000, 0b00000000},  // S
    {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001,0b00000000, 0b00000000, 0b00000000},  // T
    {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // U
    {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111,0b00000000, 0b00000000, 0b00000000},  // V
    {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111,0b00000000, 0b00000000, 0b00000000},  // W
    {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011,0b00000000, 0b00000000, 0b00000000},  // X
    {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111,0b00000000, 0b00000000, 0b00000000},  // Y
    {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011,0b00000000, 0b00000000, 0b00000000},  // Z
    {0b01111111, 0b01000001, 0b00000000, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // [
    {0b00010101, 0b00010110, 0b01111100, 0b00010110, 0b00010101,0b00000000, 0b00000000, 0b00000000},  // back slash
    {0b00000000, 0b00000000, 0b00000000, 0b01000001, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // ]
    {0b00000100, 0b00000010, 0b00000001, 0b00000010, 0b00000100,0b00000000, 0b00000000, 0b00000000},  // ^
    {0b01000000, 0b01000000, 0b01000000, 0b01000000, 0b01000000,0b00000000, 0b00000000, 0b00000000},  // _
    {0b00000000, 0b00000001, 0b00000010, 0b00000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // `
    {0b00100000, 0b01010100, 0b01010100, 0b01010100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // a
    {0b01111111, 0b01001000, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // b
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // c
    {0b00111000, 0b01000100, 0b01000100, 0b01001000, 0b01111111,0b00000000, 0b00000000, 0b00000000},  // d
    {0b00111000, 0b01010100, 0b01010100, 0b01010100, 0b00011000,0b00000000, 0b00000000, 0b00000000},  // e
    {0b00001000, 0b01111110, 0b00001001, 0b00000001, 0b00000010,0b00000000, 0b00000000, 0b00000000},  // f
    {0b00001100, 0b01010010, 0b01010010, 0b01010010, 0b00111110,0b00000000, 0b00000000, 0b00000000},  // g
    {0b01111111, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // h
    {0b00000000, 0b01000100, 0b01111101, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // i
    {0b00100000, 0b01000000, 0b01000100, 0b00111101, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // j
    {0b01111111, 0b00010000, 0b00101000, 0b01000100, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // k
    {0b00000000, 0b01000001, 0b01111111, 0b01000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // l
    {0b01111100, 0b00000100, 0b00011000, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // m
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b01111000,0b00000000, 0b00000000, 0b00000000},  // n
    {0b00111000, 0b01000100, 0b01000100, 0b01000100, 0b00111000,0b00000000, 0b00000000, 0b00000000},  // o
    {0b01111100, 0b00010100, 0b00010100, 0b00010100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // p
    {0b00001000, 0b00010100, 0b00010100, 0b00011000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // q
    {0b01111100, 0b00001000, 0b00000100, 0b00000100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // r
    {0b01001000, 0b01010100, 0b01010100, 0b01010100, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // s
    {0b00000100, 0b00111111, 0b01000100, 0b01000000, 0b00100000,0b00000000, 0b00000000, 0b00000000},  // t
    {0b00111100, 0b01000000, 0b01000000, 0b00100000, 0b01111100,0b00000000, 0b00000000, 0b00000000},  // u
    {0b00011100, 0b00100000, 0b01000000, 0b00100000, 0b00011100,0b00000000, 0b00000000, 0b00000000},  // v
    {0b00111100, 0b01000000, 0b00111000, 0b01000000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // w
    {0b01000100, 0b00101000, 0b00010000, 0b00101000, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // x
    {0b00001100, 0b01010000, 0b01010000, 0b01010000, 0b00111100,0b00000000, 0b00000000, 0b00000000},  // y
    {0b01000100, 0b01100100, 0b01010100, 0b01001100, 0b01000100,0b00000000, 0b00000000, 0b00000000},  // z
    {0b00000000, 0b00001000, 0b00110110, 0b01000001, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // {
    {0b00000000, 0b00000000, 0b01111111, 0b00000000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // |
    {0b00000000, 0b01000001, 0b00110110, 0b00001000, 0b00000000,0b00000000, 0b00000000, 0b00000000},  // }
    {0b00001000, 0b00001000, 0b00101010, 0b00011100, 0b00001000,0b00000000, 0b00000000, 0b00000000},  // ~
    {0b00001000, 0b00011100, 0b00101010, 0b00001000, 0b00001000,0b00000000, 0b00000000, 0b00000000}   // <-
};



void SystemClock48MHz( void )
{
//
// Disable the PLL
//
    RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
//
// Configure the PLL for a 48MHz system clock
//
    RCC->CFGR = 0x00280000;

//
// Enable the PLL
//
    RCC->CR |= RCC_CR_PLLON;

//
// Wait for the PLL to lock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

//
// Switch the processor to the PLL clock source
//
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

//
// Update the system with the new clock frequency
//
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClockUpdate();

}



int
main(int argc, char* argv[])
{
    SystemClock48MHz();
    myGPIOA_Init();
    myADC_Init();
    myDAC_Init();
    myTIM3_Init();
    myFrequency_Init();
    myOLED_GPIO_Init();
    oled_config();
    myLED_Init();
    myButton_Init();

    while (1)
    {
        uint16_t adcValue = myADC_Read();
        myDAC_Write(adcValue);
        myButton_Service();
        myFrequency_Start();

        // Keep updating the DAC while waiting for two rising edges.
        while (measureState == WAIT_FIRST || measureState == WAIT_SECOND)
        {
            adcValue = myADC_Read();
            myDAC_Write(adcValue);
        }

        // Copy the result together, so a button interrupt cannot mix sources.
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        uint32_t result = measureState;
        uint32_t ticks = measuredTicks;
        displaySource = measuredSource;
        myFrequency_Stop();
        measureState = MEASURE_IDLE;
        __set_PRIMASK(irqState);

        if (result == MEASURE_DONE && ticks != 0)
        {
            Freq = (timerClockHz + ticks / 2U) / ticks;
            displayStatus = DISPLAY_VALID;
        }
        else
        {
            Freq = 0;
            displayStatus = (result == MEASURE_CANCELLED)
                            ? DISPLAY_WAITING : DISPLAY_TIMEOUT;
        }

        adcValue = myADC_Read();
        myDAC_Write(adcValue);
        myPot_Update(adcValue);

        // Print only after timing stops: semihosting can pause the processor.
        if (displayStatus == DISPLAY_VALID)
        {
            if (displaySource == 0)
                trace_printf("Signal Generator: %u Hz\n", Freq);
            else
                trace_printf("NE555 Timer: %u Hz | ADC: %u | R: %u Ohms\n",
                             Freq, (unsigned int)adcValue, Res);
        }
        else
        {
            trace_printf("%s: %s\n",
                         displaySource == 0 ? "Signal Generator" : "NE555 Timer",
                         displayStatus == DISPLAY_TIMEOUT ? "timeout" : "waiting");
        }

        refresh_OLED();
    }
}



// ADC, DAC, frequency measurement, button and timer functions.

void myGPIOA_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    (void)RCC->AHBENR;
    GPIOA->MODER |= GPIO_MODER_MODER1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
    GPIOA->MODER |= GPIO_MODER_MODER4;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
}

void myADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
    ADC1->CFGR2 = ADC_CFGR2_CKMODE_1;
    ADC1->CFGR1 = 0;
    ADC1->SMPR = ADC_SMPR_SMP;
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;
    ADC1->IER = 0;
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0)
    {
    }
    for (unsigned int i = 0; i < 64U; i++)
    {
        __NOP();
    }
    ADC1->ISR = ADC_ISR_ADRDY;
    do
    {
        ADC1->CR |= ADC_CR_ADEN;
    }
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

uint16_t myADC_Read(void)
{
    while ((ADC1->CR & ADC_CR_ADSTART) != 0)
    {
    }
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0)
    {
    }
    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

void myDAC_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    (void)RCC->APB1ENR;
    RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;
    DAC->DHR12R1 = 0;
    DAC->CR = DAC_CR_EN1;
    for (unsigned int i = 0; i < 1000U; i++)
    {
        __NOP();
    }
}

void myDAC_Write(uint16_t value)
{
    DAC->DHR12R1 = value & 0x0FFFU;
}

void myPot_Update(uint16_t adcValue)
{
    uint64_t numerator = (uint64_t)adcValue * ADC_REFERENCE_MV * POT_TOTAL_OHMS;
    uint64_t denominator = 4095ULL * POT_SUPPLY_MV;
    Res = (unsigned int)((numerator + denominator / 2U) / denominator);
    if (Res > POT_TOTAL_OHMS) Res = POT_TOTAL_OHMS;

    // A clipped ADC gives only a lower bound for the resistance.
    potAtLimit = (adcValue >= 4095U && POT_SUPPLY_MV > ADC_REFERENCE_MV);
}

void myFrequency_Stop(void)
{
    EXTI->IMR &= ~FREQUENCY_LINES;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR = 0U;
    EXTI->PR = FREQUENCY_LINES;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
}

void myFrequency_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    GPIOB->MODER &= ~((3U << 4) | (3U << 6));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3U << 4) | (3U << 6)))
                    | (2U << 4) | (2U << 6);
    EXTI->IMR &= ~FREQUENCY_LINES;
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~((0xFU << 8) | (0xFU << 12)))
                         | (1U << 8) | (1U << 12);
    EXTI->RTSR |= FREQUENCY_LINES;
    EXTI->FTSR &= ~FREQUENCY_LINES;
    EXTI->PR = FREQUENCY_LINES;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    // APB timer clock doubles when the APB prescaler is not 1.
    uint32_t ppre = (RCC->CFGR >> 8) & 7U;
    uint32_t divider = ppre < 4U ? 1U : (1U << (ppre - 3U));
    timerClockHz = SystemCoreClock / divider;
    if (divider != 1U) timerClockHz *= 2U;
    TIM2->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
    TIM2->PSC = 0U;
    TIM2->ARR = timerClockHz - 1U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(EXTI2_3_IRQn, 1U);
    NVIC_SetPriority(TIM2_IRQn, 1U);
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void myFrequency_Start(void)
{
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    myFrequency_Stop();
    measuredSource = selectedSource;
    measuredTicks = 0U;
    measureState = WAIT_FIRST;
    TIM2->CNT = 0U;
    TIM2->CR1 |= TIM_CR1_CEN;
    EXTI->IMR |= measuredSource == 0U ? (1U << 2) : (1U << 3);
    __set_PRIMASK(irqState);
}

void EXTI2_3_IRQHandler(void)
{
    uint32_t line = measuredSource == 0U ? (1U << 2) : (1U << 3);
    if ((EXTI->PR & EXTI->IMR & line) == 0U) return;
    uint32_t ticks = TIM2->CNT;
    EXTI->PR = line;
    if ((TIM2->SR & TIM_SR_UIF) != 0U)
    {
        myFrequency_Stop();
        measureState = MEASURE_TIMEOUT;
    }
    else if (measureState == WAIT_FIRST)
    {
        measuredTicks = ticks;
        measureState = WAIT_SECOND;
    }
    else if (measureState == WAIT_SECOND)
    {
        measuredTicks = ticks - measuredTicks;
        myFrequency_Stop();
        measureState = MEASURE_DONE;
    }
}

void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0U)
    {
        myFrequency_Stop();
        if (measureState == WAIT_FIRST || measureState == WAIT_SECOND)
            measureState = MEASURE_TIMEOUT;
    }
}

void myButton_Init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    (void)RCC->APB2ENR;
    GPIOA->MODER &= ~(3U << 0);
    GPIOA->PUPDR &= ~(3U << 0);
    GPIOA->PUPDR |=  (2U << 0);
    EXTI->IMR &= ~(1U << 0);
    SYSCFG->EXTICR[0] &= ~(0xFU << 0);
    EXTI->RTSR |=  (1U << 0);
    EXTI->FTSR &= ~(1U << 0);
    EXTI->PR = (1U << 0);
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 1U);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    EXTI->IMR |= (1U << 0);
}

void EXTI0_1_IRQHandler(void)
{
    if ((EXTI->PR & EXTI->IMR & (1U << 0)) != 0U)
    {
        EXTI->IMR &= ~(1U << 0);
        EXTI->PR = (1U << 0);
        selectedSource ^= 1U;
        myLED_Update();
        myFrequency_Stop();
        measuredTicks = 0U;
        measureState = MEASURE_CANCELLED;
    }
}

void myButton_Service(void)
{
    static uint8_t releasedSamples = 0U;
    if ((EXTI->IMR & (1U << 0)) != 0U)
    {
        releasedSamples = 0U;
        return;
    }
    if ((GPIOA->IDR & (1U << 0)) == 0U)
    {
        releasedSamples++;
        if (releasedSamples >= 2U)
        {
            releasedSamples = 0U;
            EXTI->PR = (1U << 0);
            EXTI->IMR |= (1U << 0);
        }
    }
    else
    {
        releasedSamples = 0U;
    }
}

void myTIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
    TIM3->CR1 = TIM_CR1_OPM;
    TIM3->PSC = 47999U;
    TIM3->DIER = 0;
}

void delay_ms(uint16_t milliseconds)
{
    if (milliseconds == 0U)
    {
        return;
    }
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->ARR = milliseconds - 1U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
     while ((TIM3->SR & TIM_SR_UIF) == 0)
    {
    }
    TIM3->SR = 0;
}

void myOLED_GPIO_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;
    GPIOB->BSRR = (1U << 8) | (1U << 11);
    GPIOB->BRR = (1U << 9);
    GPIOB->MODER &= ~(
        (3U << (8U  * 2U)) |
        (3U << (9U  * 2U)) |
        (3U << (11U * 2U)) |
        (3U << (13U * 2U)) |
        (3U << (15U * 2U))
    );
    // PB8/9/11 = outputs; PB13/15 = SPI2 alternate function AF0.
    GPIOB->MODER |=
        (1U << (8U  * 2U)) |
        (1U << (9U  * 2U)) |
        (1U << (11U * 2U)) |
        (2U << (13U * 2U)) |
        (2U << (15U * 2U));
    GPIOB->OTYPER &= ~(
        (1U << 8)  |
        (1U << 9)  |
        (1U << 11) |
        (1U << 13) |
        (1U << 15)
    );
    GPIOB->PUPDR &= ~(
        (3U << (8U  * 2U)) |
        (3U << (9U  * 2U)) |
        (3U << (11U * 2U)) |
        (3U << (13U * 2U)) |
        (3U << (15U * 2U))
    );
    GPIOB->AFR[1] &= ~(
        (0xFU << ((13U - 8U) * 4U)) |
        (0xFU << ((15U - 8U) * 4U))
    );
}

// Blue PC8 = signal generator; green PC9 = NE555.
void myLED_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    (void)RCC->AHBENR;
    GPIOC->MODER &= ~((3U << 16) | (3U << 18));
    GPIOC->MODER |= (1U << 16) | (1U << 18);
    GPIOC->OTYPER &= ~((1U << 8) | (1U << 9));
    GPIOC->PUPDR &= ~((3U << 16) | (3U << 18));
    myLED_Update();
}

void myLED_Update(void)
{
    if (selectedSource == 0)
        GPIOC->BSRR = (1U << 8) | (1U << 25);
    else
        GPIOC->BSRR = (1U << 9) | (1U << 24);
}

//
// LED Display Functions
//


void refresh_OLED( void )
{
    char Buffer[17];
    uint8_t source = selectedSource;
    uint8_t status = displayStatus;

    // Do not show an old frequency under a newly selected source.
    if (source != displaySource) status = DISPLAY_WAITING;

    if (source == 0)
        Buffer[0] = '\0';  // Clear the resistance row in generator mode.
    else if (potAtLimit)
        snprintf(Buffer, sizeof(Buffer), "R: >=%4u Ohms", Res);
    else
        snprintf(Buffer, sizeof(Buffer), "R: %5u Ohms", Res);
    oled_PrintLine(0, Buffer);

    if (status == DISPLAY_VALID)
        snprintf(Buffer, sizeof(Buffer), "F: %5u Hz", Freq);
    else if (status == DISPLAY_TIMEOUT)
        snprintf(Buffer, sizeof(Buffer), "F: timeout");
    else
        snprintf(Buffer, sizeof(Buffer), "F: waiting");
    oled_PrintLine(2, Buffer);

    oled_PrintLine(4, source == 0 ? "SRC: GEN PB2" : "SRC: 555 PB3");
    oled_PrintLine(6, source == 1 && potAtLimit ? "ADC AT LIMIT" : "");
    delay_ms(100);
}


void oled_Write_Cmd( unsigned char cmd )
{
    GPIOB->BSRR = (1U << 8);  // CS = 1
    GPIOB->BRR  = (1U << 9);  // DC = 0: command
    GPIOB->BRR  = (1U << 8);  // CS = 0
    oled_Write(cmd);
    GPIOB->BSRR = (1U << 8);  // CS = 1
}

void oled_Write_Data( unsigned char data )
{
    GPIOB->BSRR = (1U << 8);  // CS = 1
    GPIOB->BSRR = (1U << 9);  // DC = 1: data
    GPIOB->BRR  = (1U << 8);  // CS = 0
    oled_Write(data);
    GPIOB->BSRR = (1U << 8);  // CS = 1
}


void oled_Write( unsigned char Value )
{
    while ((SPI2->SR & SPI_SR_TXE) == 0) {}
    HAL_SPI_Transmit(&SPI_Handle, &Value, 1, HAL_MAX_DELAY);
    while ((SPI2->SR & SPI_SR_TXE) == 0) {}
    while ((SPI2->SR & SPI_SR_BSY) != 0) {}  // Finish the last bit before CS rises.
}


void oled_config( void )
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;
    RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    SPI_Handle.Instance = SPI2;
    SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
    SPI_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Handle.Init.CRCPolynomial = 7;
    SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    HAL_SPI_Init(&SPI_Handle);
    __HAL_SPI_ENABLE(&SPI_Handle);

    delay_ms(100);
    GPIOB->BRR = (1U << 11);
    delay_ms(10);
    GPIOB->BSRR = (1U << 11);
    delay_ms(10);

    for (unsigned int i = 0; i < sizeof(oled_init_cmds); i++)
        oled_Write_Cmd(oled_init_cmds[i]);

    // Keep the page addressing that works on this display.
    oled_Write_Cmd(0x20);
    oled_Write_Cmd(0x02);
    for (unsigned int page = 0; page < 8; page++)
    {
        oled_Write_Cmd(0xB0 | page);
        oled_Write_Cmd(0x00);
        oled_Write_Cmd(0x10);
        for (unsigned int column = 0; column < 128; column++)
            oled_Write_Data(0x00);
    }
    memset(previousLine, 0, sizeof(previousLine));
}



void oled_PrintLine(uint8_t page, const char *text)
{
    char line[17];
    if (page >= 8 || text == NULL) return;

    snprintf(line, sizeof(line), "%-16.16s", text);
    if (strcmp(previousLine[page], line) == 0) return;

    oled_Write_Cmd(0xB0 | page);
    oled_Write_Cmd(0x00);
    oled_Write_Cmd(0x10);

    for (unsigned int i = 0; i < 16; i++)
    {
        unsigned char c = (unsigned char)line[i];
        if (c < 32 || c > 127) c = ' ';
        for (unsigned int column = 0; column < 8; column++)
            oled_Write_Data(Characters[c][column]);
    }
    strcpy(previousLine[page], line);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
