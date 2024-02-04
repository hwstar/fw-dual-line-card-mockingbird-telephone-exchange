#include "common.h"
#include "slic.h"

#define TAG "slic"

namespace SLIC {

// Pin mapping tables
static const uint16_t pin_map_leds[LINE_COUNT] = {LED_OFH1N, LED_OFH2N};
static const uint16_t pin_map_batfr[LINE_COUNT] = {BAT_FR1, BAT_FR2};
static const uint16_t pin_map_rm[LINE_COUNT] = {RM1, RM2};
static const uint16_t pin_map_sh[LINE_COUNT] = {SH1, SH2};
static const uint16_t pin_map_mute[LINE_COUNT] = {MUTE1, MUTE2};
static const uint16_t pin_map_pd[LINE_COUNT] = {PDN1, PDN2};


/*
* Handle the line state LED's
*/

void SLIC::_led_handler() {
  for(int i = 0; i < LINE_COUNT; i++) {
    switch(this->_line_state_leds[i]) {
      case LED_OFFHOOK:
        digitalWrite(pin_map_leds[i], LOW);
        break;

      case LED_RING:
        // Toggle LED to indicate ringing
        digitalWrite(pin_map_leds[i], !digitalRead(pin_map_leds[i]));
        break;

      case LED_ONHOOK:
      default:
        digitalWrite(pin_map_leds[i], HIGH);
        break;
    }
  }
 }


/*
* Set a line status led to a specific mode
*/

void SLIC::_set_line_state_led(uint8_t line, uint8_t mode) {
  if((line >= 0) && (line < LINE_COUNT)) {
    this->_line_state_leds[line] = mode;
  }
}

/*
* Get current hook state
*/

bool SLIC::_get_off_hook_state(uint8_t line)
{
  return (this->_hook_state_current & (1 << line) > 0);
}


/*
* Return TRUE if there was a on-hook to off hook transition
*/

bool SLIC::_off_hook_transition(uint8_t line)
{
  uint8_t c = this->_hook_state_current & (1 << line);
  uint8_t l = this->_hook_state_last & (1 << line);

  if((c > 0 ) && (!l)) {
    return true;
  }
  return false;
}

/*
* Return TRUE if there was a off-hook to on hook transition
*/

bool SLIC::_on_hook_transition(uint8_t line)
{
  uint8_t c = this->_hook_state_current & (1 << line);
  uint8_t l = this->_hook_state_last & (1 << line);

  if((!c) && (l > 0)) {
    return true;
  }
  return false;

}

/*
* Request service from main processor
*/

bool SLIC::_request_service(uint8_t line, uint8_t event){
  bool res = true;
  LOG_DEBUG(TAG, "Sending event: %d for line %d to main CPU", event, line)
  if(!test_mode) {



  // TODO add ring buffer
  // TODO Add assert ATTEN
  }
  return res;
}

/*
* This handles the hookswitch
*/

void SLIC::_hook_switch_handler() {

 
  uint8_t hook_state_now = 0;
  // Get the current switch hook state and save the prior state
  for(int i = 0; i < LINE_COUNT; i++) {
    hook_state_now |= (digitalRead(pin_map_sh[i]) << i);
  }

  this->_hook_state_last = this->_hook_state_current;
  this->_hook_state_current = hook_state_now;
  
  // Update the hook state leds
  for(int i = 0; i < LINE_COUNT; i++) {
    bool line_hook_state_current = ((this->_hook_state_current & (1 << i)) > 0);
    bool line_hook_state_last = ((this->_hook_state_last & (1 << i)) > 0);

    if(line_hook_state_current != line_hook_state_last) {
      this->_set_line_state_led(i, line_hook_state_current);
    }
  }
}

/*
* Control ringing, ring trip, and ring cadence 
*/

void SLIC::_ringing_handler() {

  for(int i = 1; i < LINE_COUNT; i++) {
    uint8_t line_hook_state = ((this->_hook_state_current & (1 << i)) > 0);
    
    switch(this->_ring_state[i]) {
      case RINGING_START_R1: // 2 Sec on, 4 Sec off Cadence
        digitalWrite(pin_map_rm[i], 1); // Set high voltage ring mode
        this->_set_line_state_led( i, LED_RING);
        this->_ring_timer[i] = 2000/RING_TICK_TIME;
        this->_ring_state[i] = RINGING_ACTIVE_R1;
        break;

      case RINGING_PAUSE_R1: 
        // Test for ring trip
        if(this->_hook_state_current & (1 << i)) {
          this->_set_line_state_led( i, LED_OFFHOOK);
          digitalWrite(pin_map_rm[i], 0); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], 0); // Normal battery polarity
          this->_ring_state[i] = RINGING_PICKUP;
          break;
        }
        
        // Test to see if ring timer expired
        if(!this->_ring_timer[i]){
          this->_set_line_state_led( i, LED_RING);
          this->_ring_timer[i] = 2000/RING_TICK_TIME;
          this->_ring_state[i] = RINGING_ACTIVE_R1;
        }
        else {
          this->_ring_timer[i]--;
        }
        break;
  

      case RINGING_ACTIVE_R1: 
        // Test for ring trip
        if(this->_hook_state_current & (1 << i)) {
          this->_set_line_state_led( i, LED_OFFHOOK);
          digitalWrite(pin_map_rm[i], 0); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], 0); // Normal battery polarity
          this->_ring_state[i] = RINGING_PICKUP;
          break;
        }

        // Test to see if ring timer expired
        if(!this->_ring_timer[i]) {
          digitalWrite(pin_map_batfr[i], 0);
          this->_set_line_state_led( i, LED_ONHOOK);
          this->_ring_timer[i] = 4000/RING_TICK_TIME;
          this->_ring_state[i] = RINGING_PAUSE_R1;
        }
        else {
          digitalWrite(pin_map_batfr[i], !digitalRead(pin_map_batfr[i]));
          this->_ring_timer[i]--;
        }
        break;
  
      case RINGING_PICKUP: // Called party picked up
        break; // Wait here so main handler can observe this state

      case RINGING_STOP: // Asked to stop
          digitalWrite(pin_map_rm[i], 0); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], 0); // Normal battery polarity
          this->_set_line_state_led( i, LED_ONHOOK);
          this->_ring_state[i] = RINGING_OFF;
          break;


      case RINGING_OFF:
      default:
        break;
    }
  }
}

/*
* Main Handler - Acts on information from lower level handlers
*/

void SLIC::_main_handler() {
  for( int i = 0; i < LINE_COUNT; i++) {
    
    switch(this->_mh_state[i]) {


    default:
    case MHS_IDLE:
      // Check for off hook condition
      //LOG_INFO(TAG, "Line %d", i + 1);
      if(this->_off_hook_transition(i)) {
        LOG_INFO(TAG, "Line %d off hook", i + 1);
        this->_mh_state[i] = MHS_REQUEST_OR;
      }
      break;

    case MHS_REQUEST_OR:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_state[i] = MHS_ON_HOOK;
      }
      else {
        this->_request_service(i, EV_REQUEST_OR);
        // Wait for the OR to get connected
        this->_mh_state[i] = MHS_REQUEST_OR_WAIT;
      }
      break;

    case MHS_REQUEST_OR_WAIT:
     // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_state[i] = MHS_ON_HOOK;
      }
      else {
        if(test_mode == TM_STANDALONE) {
          // Standalone test mode

        }
        else {
          // Wait for OR to be connected
        }

      
      }
      break;

    case MHS_OR_CONNECTED:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_state[i] = MHS_ON_HOOK;
      }
      else {
      // Wait for connection to be set up
      }
      break;

    case MHS_RINGING:
      break;


    case MHS_IN_CALL:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_state[i] = MHS_ON_HOOK;
      }





    case MHS_ON_HOOK:
      // Print log entry
      LOG_INFO(TAG, "Line %d on hook", i + 1);
      // Notify main processor to abort the call
      this->_request_service(i, EV_HUNGUP);
      // Go back to IDLE state
      this->_mh_state[i] = MHS_IDLE;
       break;
    }
  }
}

/*
* I2C Master is requesting something
*/
void SLIC::i2c_request() {

}

/*
* We received some data from the I2C Master
*/

void SLIC::i2c_receive(int howMany) {

}

/*
* Hardware specific setup for the SLIC class
*/

void SLIC::setup() {
    // Dual Slic Configuration-specific initialization
    pinMode(RM1, OUTPUT);
    digitalWrite(RM1, LOW); // Not ringing
    pinMode(SH1, INPUT);
    pinMode(PDN1, OUTPUT);
    digitalWrite(PDN1, HIGH); // Powered off
    pinMode(MUTE1, OUTPUT);
    digitalWrite(MUTE1, HIGH); // Muted
    pinMode(RM2, OUTPUT);
    digitalWrite(RM2, LOW); // Not ringing
    pinMode(SH2, INPUT);
    pinMode(PDN2, OUTPUT);
    digitalWrite(PDN2, HIGH); // Powered off
    pinMode(MUTE2, OUTPUT);
    digitalWrite(MUTE2, HIGH); // Muted
    pinMode(BAT_FR1, OUTPUT);
    digitalWrite(BAT_FR1, LOW); // Normal line polarity
    pinMode(BAT_FR2, OUTPUT);
    digitalWrite(BAT_FR2, LOW); // Normal line polarity
    pinMode(LED_OFH1N, OUTPUT);
    digitalWrite(LED_OFH1N, HIGH); // Line 1 LED off
    pinMode(LED_OFH2N, OUTPUT);
    digitalWrite(LED_OFH2N, HIGH); // Line 2 LED off

    this->test_mode = TM_STANDALONE;
 
    if(this->test_mode == TM_STANDALONE) {
      digitalWrite(LEDN_TEST, 0);
      digitalWrite(PDN1, LOW); // Power on
      digitalWrite(PDN2, LOW); // 
      digitalWrite(MUTE1, LOW); // Unmuted
      digitalWrite(MUTE2, LOW);
    }



    
   
}


/*
* Called every 5ms from timer interrupt
*/

void SLIC::service() {
  // Called every 5mS.

  // Set TP406
  digitalWrite(TP406, 1);
 
  // Every 25 mS call the handlers in a staggered manner
  if((this->_ring_ticks & 7) == 0) { 
    this->_hook_switch_handler();
  }
  else if((this->_ring_ticks & 7) == 1) {
    this->_led_handler();
  }
  else if((this->_ring_ticks & 7) == 2) {
    this->_ringing_handler();
  }
  else if((this->_ring_ticks & 7 ) == 4) { // Always last
    this->_main_handler();
  }

  this->_ring_ticks++;
  if(_ring_ticks > 4) {
    _ring_ticks = 0;
  }
  // Clear TP406
  digitalWrite(TP406, 0);
}  


/*
* Foreground loop
*/


void SLIC::loop() {
  

}



} // End Namespace SLIC