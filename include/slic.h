#pragma once
#include "common.h"
#include "ringbuf.h"


namespace SLIC {

const uint8_t LINE_COUNT = 2;
const uint32_t RING_TICK_TIME = 25;

enum {MH_TIMER_1=0, MH_TIMER_2, MAX_MH_TIMERS};
enum {LED_ONHOOK=0, LED_OFFHOOK, LED_RING};
enum {RINGING_OFF=0, RINGING_START_R1, RINGING_ACTIVE_R1, RINGING_PAUSE_R1, RINGING_PICKUP, RINGING_STOP};
enum {MHS_IDLE=0,MHS_DIALING};
enum {EV_NONE=0, EV_OFFHOOK, EV_ONHOOK};


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

    void _set_line_state_led(uint8_t line, uint8_t mode);
    bool _get_off_hook_state(uint8_t line);
    bool _off_hook_transition(uint8_t line);
    bool _on_hook_transition(uint8_t line);

  

    uint8_t _i2c_address;
    uint8_t _ring_ticks;
    uint8_t _hook_state_current;
    uint8_t _hook_state_last;
    uint8_t _line_state_leds[LINE_COUNT];
    uint8_t _ring_state[LINE_COUNT];
    uint8_t _next_ring_state[LINE_COUNT];
    uint8_t _mh_state[LINE_COUNT];
    uint16_t _mh_timer[LINE_COUNT][MAX_MH_TIMERS];
    uint16_t _ring_timer[LINE_COUNT];

    RingBuf<uint8_t, 20> _event_buffer; // Holds events to send to the main CPU
};

} // End namespace SLIC
