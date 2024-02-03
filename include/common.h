#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <inttypes.h>
#include "logging.h"

extern LOGGING::Logging Log;

#define LOG_LEVEL LOG_LEVEL_DEBUG // Default logging level

#define LOG_ERROR(tag, format, args...) Log.log(tag, LOGGING::LOGGING_ERROR, millis(), format, args);
#define LOG_WARN(tag, format, args...) Log.log(tag, LOGGING::LOGGING_WARN, millis(), format, args);
#define LOG_NOTICE(tag, format, args...) Log.log(tag, LOGGING::LOGGING_NOTICE, millis(), format, args);
#define LOG_INFO(tag, format, args...) Log.log(tag, LOGGING::LOGGING_INFO, millis(), format, args);
#define LOG_DEBUG(tag, format, args...) Log.log(tag, LOGGING::LOGGING_DEBUG, millis(), format, args);

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


