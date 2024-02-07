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
  return ((this->_hook_state_current & (1 << line)) > 0);
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
* Power up or power down a SLIC
*/

void SLIC::_set_power_ctrl(uint8_t line, bool power_ctrl) {
  if(line < LINE_COUNT) {
    digitalWrite(pin_map_pd[line], !power_ctrl);
  }

}


/*
* I2C Master is requesting something to be sent to it
*/
void SLIC::i2c_request() {
  uint8_t tx_str[EVENT_BUFFER_SIZE];
  // Process master TX requests here
  switch(this->_i2c_register_address) {
    case REG_GET_EVENT:
      if(!this->_event_buffer_empty()) {
        memcpy(tx_str, this->_event_buffer.ring_buffer[this->_event_buffer.tail], EVENT_BUFFER_SIZE); // Make local copy of event
        this->_event_buffer.tail = this->_event_buffer_next(this->_event_buffer.tail); // Next tail position
        // If event buffer is empty, drop ATTEN
        if(this->_event_buffer_empty()) {
          digitalWrite(ATTEN, LOW);
        }
        // Send the event
        Wire.write(tx_str[0]);
        Wire.write(tx_str[1]);
      }
      else { // Event queue was empty
        digitalWrite(ATTEN, LOW); // Drop ATTEN to be sure
        Wire.write(EV_NONE);
        Wire.write(0xFF);
      }
      break;

    case REG_GET_BUSY_STATUS:
      Wire.write(this->_mh_state[0] != MHS_IDLE);
      Wire.write(this->_mh_state[1] != MHS_IDLE);
      break;

    default:
      Wire.write(0xFF);
      Wire.write(0xFF);
      break;
  }
}

/*
* We received some data from the I2C Master
*/

void SLIC::i2c_receive(int howMany) {
  if(howMany > 0) {
    // First byte is the I2C register address 
    this->_i2c_register_address = Wire.read();
    howMany--;
    this->_i2c_read_data_length = 0;
    // If there is data following the I3C address process it here.
    while((howMany) && (this->_i2c_read_data_length < MAX_I2C_DATA_LENGTH)) {
        this->_i2c_read_data[this->_i2c_read_data_length++] = Wire.read();
        howMany--;
    }
    // Signal to the foreground that read data is ready to be processed.
    if(this->_i2c_read_data_length) {
      this->_i2c_read_data_ready = true;
    }
  }
}

/*
* Request service from main processor
*/

bool SLIC::_request_service(uint8_t line, uint8_t event){
  bool res = !this->_event_buffer_full();
  LOG_DEBUG(TAG, "Queuing event %d on line %d", event, line + 1);
  //if(this->_test_mode) { // TODO: Uncomment
  //  return true;
  //}

  if(res) {
    uint8_t *ev = this->_event_buffer.ring_buffer[this->_event_buffer.head];
    ev[0] = event;
    ev[1] = line;
    this->_event_buffer.head = this->_event_buffer_next(this->_event_buffer.head);
    digitalWrite(ATTEN, HIGH); // Set attention
  }
  else {
    this->_event_buffer.overflow_error = true;
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

  for(int i = 0; i < LINE_COUNT; i++) {
    uint8_t line_hook_state = ((this->_hook_state_current & (1 << i)) > 0);
    
    switch(this->_ring_state[i]) {
      case RINGING_START_R1: // 2 Sec on, 4 Sec off Cadence
        digitalWrite(pin_map_mute[i], HIGH);
        digitalWrite(pin_map_rm[i], HIGH); // Set high voltage ring mode
        this->_set_line_state_led( i, LED_RING);
        this->_ring_timer[i] = 2000/TICK_TIME; // 2 seconds on
        this->_ring_state[i] = RINGING_ACTIVE_R1;
        break;

      case RINGING_PAUSE_R1: 
        // Test for ring trip
        if(this->_hook_state_current & (1 << i)) {
          this->_set_line_state_led( i, LED_OFFHOOK);
          digitalWrite(pin_map_rm[i], LOW); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], LOW); // Normal battery polarity
          this->_ring_state[i] = RINGING_PICKUP;
          break;
        }
        
        // Test to see if ring timer expired
        if(!this->_ring_timer[i]){
          this->_set_line_state_led( i, LED_RING);
          this->_ring_timer[i] = 2000/TICK_TIME; // 2 seconds on
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
          digitalWrite(pin_map_rm[i], LOW); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], LOW); // Normal battery polarity
          digitalWrite(pin_map_mute[i], LOW);
          this->_ring_state[i] = RINGING_PICKUP;
          break;
        }

        // Test to see if ring timer expired
        if(!this->_ring_timer[i]) {
          digitalWrite(pin_map_batfr[i], LOW);
          this->_set_line_state_led( i, LED_ONHOOK);
          this->_ring_timer[i] = 4000/TICK_TIME; // 4 seconds off
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
          digitalWrite(pin_map_rm[i], LOW); // High voltage ring mode off
          digitalWrite(pin_map_batfr[i], LOW); // Normal battery polarity
          digitalWrite(pin_map_mute[i], LOW);
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
      if(this->_mh_state_advance[i]) { // Check for ringing request
        LOG_DEBUG(TAG,"Line %d ringing", i + 1);
        this->_mh_state_advance[i] = false;
        this->_ring_state[i] = RINGING_START_R1;
        this->_mh_state[i] = MHS_RINGING;
      }

      break;

    case MHS_REQUEST_OR:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        // Since we didn't get too far into the call,
        // just go back to the idle state here.
        // No need to send an event the main CPU.
        this->_mh_state[i] = MHS_IDLE;
      }
      else {
        LOG_DEBUG(TAG,"Line %d requesting OR", i + 1);
        // Tell the main CPU we need an OR.
        this->_request_service(i, EV_REQUEST_OR);
        // Wait for the OR to get connected
        this->_mh_state_advance[i] = false;
        this->_mh_state[i] = MHS_REQUEST_OR_WAIT;
      }
      break;

    case MHS_REQUEST_OR_WAIT:
     // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_timer[i][TIMER_ON_HOOK] = HANGUP_WAIT_TIME/TICK_TIME; 
        this->_mh_state_return[i] = MHS_REQUEST_OR_WAIT;
        this->_mh_state[i] = MHS_TIME_ON_HOOK_SUB;
      }
      else {
          // Wait for OR to be connected
          if(this->_mh_state_advance[i]) {
            LOG_DEBUG(TAG,"Line %d OR attached", i + 1);
            this->_mh_state_advance[i] = false;
            this->_mh_state[i] = MHS_OR_CONNECTED;
          }
        }
      break;

    case MHS_OR_CONNECTED:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_timer[i][TIMER_ON_HOOK] = HANGUP_WAIT_TIME/TICK_TIME; 
        this->_mh_state_return[i] = MHS_OR_CONNECTED;
        this->_mh_state[i] = MHS_TIME_ON_HOOK_SUB;
      }
      else {
        // Wait for connection to be set up
        if(this->_mh_state_advance[i]) {
          LOG_DEBUG(TAG,"Line %d call connected", i + 1);
          this->_mh_state_advance[i] = false;
          this->_mh_state[i] = MHS_IN_CALL;
        }
      }
      break;

    case MHS_RINGING:
      // Test for called party pickup
      if(this->_ring_state[i] == RINGING_PICKUP) {
        this->_request_service(i, EV_ANSWERED);
        this->_ring_state[i] = RINGING_OFF;
        this->_mh_state[i] = MHS_IN_CALL;
      }
      // Test for calling party hangup
      else if (this->_mh_state_advance[i]){
        LOG_DEBUG(TAG,"Line %d calling party hungup", i + 1);
        this->_ring_state[i] = RINGING_STOP;
        this->_mh_state_advance[i] = false;
        this->_mh_state[i] = MHS_CALL_DISCONNECTED;
      }

      break;


    case MHS_IN_CALL:
      // Test for on-hook
      if(this->_on_hook_transition(i)) {
        this->_mh_timer[i][TIMER_ON_HOOK] = HANGUP_WAIT_TIME/TICK_TIME; 
        this->_mh_state_return[i] = MHS_IN_CALL;
        this->_mh_state[i] = MHS_TIME_ON_HOOK_SUB;
      }
      // Did we get a disconnect request from the main CPU?
      else if (this->_mh_state_advance[i]) {
        LOG_DEBUG(TAG,"Line %d disconnected by the man CPU", i + 1);
        this->_mh_state_advance[i] = false;
        // Wait for our line to hang up
        this->_mh_state[i] = MHS_CALL_DISCONNECTED;
      }
      break;


    case MHS_ON_HOOK: // We end up here if the line being monitored goes on hook.
      // Print log entry
      LOG_INFO(TAG, "Line %d on hook", i + 1);
      // Notify main processor to abort the call
      this->_request_service(i, EV_HUNGUP);
      // Set the default return state
      this->_mh_state_return[i] = MHS_IDLE;
      // Go back to IDLE state
      this->_mh_state_advance[i] = false;
      this->_mh_state[i] = MHS_IDLE;
       break;

    
    case MHS_CALL_DISCONNECTED: // We end up here if disconnected by the main CPU
        // Wait for the line to go back on hook
        // This is so the line will look busy until hungup.
        if(!this->_get_off_hook_state(i)) {
          LOG_INFO(TAG, "Line %d on hook after disconnect", i + 1);
          this->_mh_state_advance[i] = false;
          this->_mh_state[i] = MHS_IDLE;
      }
      break;




    case MHS_TIME_ON_HOOK_SUB: // Subroutine. Time the on hook time before registering a on hook condition
      if(!this->_mh_timer[i][TIMER_ON_HOOK]) {
        LOG_DEBUG(TAG, "Line %d has hung up", i + 1);
        this->_mh_state[i] = MHS_ON_HOOK;
      }
      else {
        // If user goes back off hook, then return to the original state set.
        
        if(this->_get_off_hook_state(i)) {
          LOG_DEBUG(TAG, "Line %d went back off hook before hook timer expired", i + 1);
          this->_mh_state[i] = this->_mh_state_return[i];
        }
        this->_mh_timer[i][TIMER_ON_HOOK]--;
      }
      break;

    }
  }
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
    digitalWrite(MUTE1, LOW); // Not Muted
    pinMode(RM2, OUTPUT);
    digitalWrite(RM2, LOW); // Not ringing
    pinMode(SH2, INPUT);
    pinMode(PDN2, OUTPUT);
    digitalWrite(PDN2, HIGH); // Powered off
    pinMode(MUTE2, OUTPUT);
    digitalWrite(MUTE2, LOW); // Not Muted
    pinMode(BAT_FR1, OUTPUT);
    digitalWrite(BAT_FR1, LOW); // Normal line polarity
    pinMode(BAT_FR2, OUTPUT);
    digitalWrite(BAT_FR2, LOW); // Normal line polarity
    pinMode(LED_OFH1N, OUTPUT);
    digitalWrite(LED_OFH1N, HIGH); // Line 1 LED off
    pinMode(LED_OFH2N, OUTPUT);
    digitalWrite(LED_OFH2N, HIGH); // Line 2 LED off
    pinMode(ATTEN, OUTPUT); 
    digitalWrite(ATTEN, LOW); // Atten OFF

    this->_test_mode = TM_NONE;

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

  if(this->_event_buffer.overflow_error) {
    LOG_ERROR(TAG, "Event buffer overflow");
    this->_event_buffer.overflow_error = false;
  }

  // Check for and process write register requests from the main processor
  if(this->_i2c_read_data_ready) {
    switch(this->_i2c_register_address) {
      case REG_SET_OR_ATTACHED:  // Notification of an attached OR
        if(this->_i2c_read_data_length == 1) {
          uint8_t line = this->_i2c_read_data[0];
          noInterrupts();
          if(this->_mh_state[line] == MHS_REQUEST_OR_WAIT) {
            this->_mh_state_advance[line] = true;
          }
          interrupts();
        }
        break;

      case REG_SET_IN_CALL: // Dialing complete. Set in call state
        if(this->_i2c_read_data_length == 1) {
          uint8_t line = this->_i2c_read_data[0];
          noInterrupts();
          if(this->_mh_state[line] == MHS_OR_CONNECTED) {
            this->_mh_state_advance[line] = true;
          }
          interrupts();
        }
        break;

      case REG_REQUEST_RINGING:  // Request to ring a line
        if(this->_i2c_read_data_length == 2) {
          uint8_t line = this->_i2c_read_data[0];
          uint8_t ring_mode = this->_i2c_read_data[1]; // TODO RFU
          noInterrupts();
          if(this->_mh_state[line] == MHS_IDLE) {
            this->_mh_state_advance[line] = true;
            this->_request_service(line, EV_RINGING);
          }
          else {
            this->_request_service(line, EV_BUSY);
          }
          interrupts();
        }
        break;

      case REG_END_CALL:  // Cancel ringing, or end a call if one is in process
        if(this->_i2c_read_data_length == 1) {
          uint8_t line = this->_i2c_read_data[0];
          noInterrupts();
          if((this->_mh_state[line] == MHS_RINGING) || (this->_mh_state[line] == MHS_IN_CALL)) {
            this->_mh_state_advance[line] = true;
          }
          interrupts();
        }
        break;

      case REG_POWER_CTRL: // Power up or power down a SLIC
        if(this->_i2c_read_data_length == 2) {
          uint8_t line = this->_i2c_read_data[0];
          uint8_t power_ctrl = this->_i2c_read_data[1];
          this->_set_power_ctrl(line, (power_ctrl > 0));
        }
    }
    this->_i2c_read_data_ready = false; // Request processed
  }
}

} // End Namespace SLIC