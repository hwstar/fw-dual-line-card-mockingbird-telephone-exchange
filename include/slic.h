#pragma once
#include "common.h"

namespace SLIC {

const uint8_t LINE_COUNT = 2;
const uint32_t TICK_TIME = 25;
const uint8_t EVENT_BUFFER_DEPTH = 4;
const uint8_t EVENT_BUFFER_SIZE = 2;

// Software timers
enum {TIMER_ON_HOOK=0, MAX_TIMERS};
// LED states
enum {LED_ONHOOK=0, LED_OFFHOOK, LED_RING};
// Ringing states
enum {RINGING_OFF=0, RINGING_START_R1, RINGING_ACTIVE_R1, RINGING_PAUSE_R1, RINGING_PICKUP, RINGING_STOP};
// Main handler states
enum {MHS_IDLE=0, MHS_REQUEST_OR, MHS_REQUEST_OR_WAIT, MHS_OR_CONNECTED, MHS_RINGING, MHS_IN_CALL, MHS_ON_HOOK};
// Events sent to main CPU
enum {EV_READY=0, EV_REQUEST_OR, EV_DIALED_DIGIT, EV_HOOKFLASH, EV_BUSY, EV_RINGING, EV_ANSWERED, EV_HUNGUP, MHS_TIME_ON_HOOK_SUB};
// Commands
enum {CMD_POWER, CMD_OR_CONNECTED, CMD_POLARITY, CMD_RING};
// Test modes
enum {TM_NONE=0, TM_STANDALONE};


typedef struct eventBuffer {
    uint8_t head;
    uint8_t tail;
    bool overflow_error;
    uint8_t ring_buffer [EVENT_BUFFER_DEPTH][EVENT_BUFFER_SIZE];
} eventBuffer;


class SLIC {
public:
    void setup();
    void service();
    void loop();
    void i2c_request();
    void i2c_receive(int howMany);
protected:
    void _hook_switch_handler();
    void _led_handler();
    void _ringing_handler();
    void _main_handler();

    bool _event_buffer_full() { return (((this->_event_buffer.head + 1) & (EVENT_BUFFER_DEPTH - 1)) == this->_event_buffer.tail); }
    bool _event_buffer_empty() { return (this->_event_buffer.tail == this->_event_buffer.head); }
    uint8_t _event_buffer_next(uint8_t buffer) { return (buffer + 1) & (EVENT_BUFFER_DEPTH - 1); }

    void _set_line_state_led(uint8_t line, uint8_t mode);
    bool _get_off_hook_state(uint8_t line);
    bool _off_hook_transition(uint8_t line);
    bool _on_hook_transition(uint8_t line);
    bool _request_service(uint8_t line, uint8_t event);


  

    uint8_t _i2c_address;
    uint8_t _ring_ticks;
    uint8_t _hook_state_current;
    uint8_t _hook_state_last;
    uint8_t _line_state_leds[LINE_COUNT];
    uint8_t _ring_state[LINE_COUNT];
    uint8_t _next_ring_state[LINE_COUNT];
    uint8_t _mh_state[LINE_COUNT];
    uint8_t _mh_state_return[LINE_COUNT];
    uint8_t _test_mode;
    eventBuffer _event_buffer;
    uint16_t _mh_timer[LINE_COUNT][MAX_TIMERS];
    uint16_t _ring_timer[LINE_COUNT];

};

} // End namespace SLIC
