#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <inttypes.h>
#include "logging.h"

extern LOGGING::Logging Log;

#define LOG_LEVEL_DEBUG 4
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_NOTICE 2
#define LOG_LEVEL_WARN 1

//*************** USER MODIFYABLE AREA BEGIN **************************

/*
* Tunable parameters
*/


#define HANGUP_WAIT_TIME 1500 // Used to distinguish a hangup from hookflash and dial pulses. Time in milliseconds

/*
* GPIO pin definitions
*/

#define RM1 PA0
#define SH1 PA1
#define PDN1 PA2
#define MUTE1 PA3
#define RM2 PA4
#define SH2 PA5
#define PDN2 PA6
#define MUTE2 PA7
#define BAT_FR1 PA8
#define BAT_FR2 PA11
#define LEDN_TEST PC13
#define LED_OFH1N PB0
#define LED_OFH2N PB1
#define TP406 PB3
#define TP407 PB4
#define TP408 PB5
#define UART_TX PB6
#define UART_RX PB7
#define SCL PB8
#define SDA PB9
#define ATTEN PB10
#define SW1N PB11
#define SW2N PB12
#define SW3N PB13
#define CFG0N PA15
#define CFG1N PB14
#define CFG2N PB15


/*
* Default logging level
*/

#define LOG_LEVEL LOG_LEVEL_DEBUG

//*************** USER MODIFYABLE AREA END **************************


#define LOG_ERROR(tag, format, ...) Log.log(tag, LOGGING::LOGGING_ERROR, millis(), format __VA_OPT__(,) __VA_ARGS__)
#if LOG_LEVEL_WARN <= LOG_LEVEL
#define LOG_WARN(tag, format, ...) Log.log(tag, LOGGING::LOGGING_WARN, millis(), format __VA_OPT__(,) __VA_ARGS__)
#else
#define LOG_WARN(tag, format, ...)
#endif
#if LOG_LEVEL_NOTICE <= LOG_LEVEL
#define LOG_NOTICE(tag, format, ...) Log.log(tag, LOGGING::LOGGING_NOTICE, millis(), format __VA_OPT__(,) __VA_ARGS__)
#else
#define LOG_NOTICE(tag, format, ...)
#endif
#if LOG_LEVEL_INFO <= LOG_LEVEL
#define LOG_INFO(tag, format, ...) Log.log(tag, LOGGING::LOGGING_INFO, millis(), format __VA_OPT__(,) __VA_ARGS__)
#else
#define LOG_INFO(tag, format, ...)
#endif
#if LOG_LEVEL_DEBUG <= LOG_LEVEL
#define LOG_DEBUG(tag, format, ...) Log.log(tag, LOGGING::LOGGING_DEBUG, millis(), format __VA_OPT__(,) __VA_ARGS__)
#else
#define LOG_DEBUG(tag, format, ...)
#endif


