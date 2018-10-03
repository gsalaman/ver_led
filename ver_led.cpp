/*====================================================
 * VER_LED.CPP
 * 
 * This library uses the built-in LED on the main board to blink 
 * a "version number".
 * 
 * Example:  if setup with version=3, the board will do three short blinks, 
 *   followed by a pause, then repeat.
 *   
 * Notes:
 *   Functionality implemented by a simple state machine.  state_machine_driver is the main
 *   processing function; it calls the appropriate sub-state functions.  Those
 *   functions will check the current time compared to the time we entered that state to 
 *   determine whether a state transition is needed.
 *  
 *   The state machine driver is now triggered via a 10ms periodic timer and
 *   runs in interrupt space.  
 *   
 * Side Effects and Dependencies:
 *   This function will take over the built in LED.
 *   We use the arduino's TIMER1 in order to process the state machine.

 */

#include <ver_led.h>

//#define DEBUG_SERIAL

// States for our state machine
typedef enum
{
  VER_LED_INIT,         // Uninitialized, waiting for configuration
  VER_LED_ON,           // LED has been turned on, and we're waiting to turn it off.
  VER_LED_SHORT_OFF,    // We just turned the LED off, and we've got more "blinks" before a long off.
  VER_LED_LONG_OFF      // We just did our last blink, and we're pausing before doing the next sequence.
} ver_led_state_type;

#ifdef DEBUG_SERIAL
char *led_state_strings[] =
{
  "VER_LED_INIT",
  "VER_LED_ON",
  "VER_LED_SHORT_OFF",
  "VER_LED_LONG_OFF"
};
#endif

static ver_led_state_type led_state;  // This is the current state of our state machine

static int version=0;    // Version number corresponds to how many "quick blinks" in a row we'll do.

// Blink time parameters.  Using variables instead of #defines here in case we want to change these
//   on the fly in future implementations.
static int led_on_time=500;           // Time in ms to keep the LED on for each blink
static int led_short_off_time=500;    // Time in ms to between each quick blink
static int led_long_off_time = 3000;  // Time in ms for the pause between blink sequences

static int which_blink=0;   // keeps track of the number of blinks in a row we've done.  Used to determine
                            // whether we're going to a short or long pause after a given blink.

static unsigned long led_state_entry_time=0;  // keeps track of the timestamp when we entered our current state.

/*====================================================================
 * INTERNAL function init_tick_timer
 *
 * Description:  
 *   This function is responsible for setting up the tick timer we use
 *   to push the state machine forward.  Note the tick is currently 
 *   hard-coded to 10ms.
 */
static void init_tick_timer( void )
{
  // current iteration:  tick every 10 ms.
  //    Math:  10ms period = 100 Hz.
  //           use prescaler of 256.
  //           compare match register:  16 MHz/256/100 = 625.
  int compare_match_reg=625;

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = compare_match_reg;// compare match register 
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (B00000100);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts  

}  // init_tick_timer


/*====================================================================
 * API Function:  ver_led_setup
 * 
 * Description:  used to set up operations...specifically setting the number of 
 *   "blinks", and configuring the built-in LED for operation.
 *   
 * Parameters: 
 *   version:  the number of consecutive blinks
 *   
 * Return value:
 *   VER_LED_OKAY if our input version is in range.
 *   VER_LED_ERROR if the input version is out of range.  Note that in this case, subsequent calls
 *     to ver_led_run will do nothing.
 */
int ver_led_setup( int ver )
{

#ifdef DEBUG_SERIAL
  Serial.begin(9600);
#endif
  
  if ((ver < VER_LED_MIN_VERSION) || (ver > VER_LED_MAX_VERSION))
  {
    return(VER_LED_ERROR);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  version = ver;
  
  led_state = VER_LED_INIT;
  
  // initialize our tick timing for the state machine.
  init_tick_timer();

} // ver_led_setup

/*==================================================================
 * State function:  init_state
 * 
 * Description:
 *   In the init state, we turn on the LED and mark the "entry time"...which
 *     currently represents when we turned ON the led.
 *     
 * Return Value:
 *   We'll always go to the "ON" state after initing.
 */
ver_led_state_type init_state( void )
{
  // catch the case where we haven't called ver_led_setup without a valid parameter
  if (version == 0)
  {
    // stay in init, hopeful that someone, someday will call ver_led_setup with the 
    // correct parameters.
    return(VER_LED_INIT);
  }

  // Turn on the LED, and mark the time.
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  led_state_entry_time = millis();

  // this is our first "blink"
  which_blink = 1;
  
  return(VER_LED_ON);
    
}  // init_state

/*===================================================================
 * State function:  on_state
 * 
 * Description:
 *   In the on state, we check to see if enough time has passed to turn off the LED.
 *   
 * Return Value:
 *   If we still need to leave the LED on, stay in VER_LED_ON
 *   If enough time has passed and we have more blinks, go to VER_LED_SHORT_OFF.
 *   If enough time has passed and we are done with our sequence, go to VER_LED_LONG_OFF.
 */
ver_led_state_type on_state( void )
{
  unsigned long cur_time;
  
  //what time is it now?
  cur_time = millis();

  // has the LED been on long enough?
  if (cur_time > led_state_entry_time + led_on_time)
  {
    // Turn off the LED and mark the time we turned off the LED.
    digitalWrite(LED_BUILTIN, LOW);
    led_state_entry_time = cur_time;
    
    // do we need a short off or a long off?
    if (which_blink < version)
    {
      // Need more blinks.  Go to a short off.
      which_blink++;  
      return(VER_LED_SHORT_OFF);
    }  // going to short off
    else
    {
      // no more blinks.  Go to a long off.
      which_blink = 0;
      return(VER_LED_LONG_OFF);
    }  // going to a long off

  }  // have we been here long enough?
  else
  {
    // Still have more time to wait.  Stay in this state.
    return(VER_LED_ON);
  }
}  // on_state

/*==========================================================
 * State function:  short_off_state
 * 
 * Description:
 *   This function implements a "quick pause" between blinks.
 *  
 * Return Value:
 *   If we still have more time to wait in our "quick pause", stay in VER_LED_SHORT_OFF.
 *   If enough time has passed, we need to turn on the LED and go to VER_LED_ON
 */
ver_led_state_type short_off_state( void )
{
  unsigned long cur_time;
  
  //what time is it now?
  cur_time = millis();

  // has the LED been off long enough?
  if (cur_time > led_state_entry_time + led_short_off_time)
  {
    // Turn the LED back on, and mark the time we turned it on.
    digitalWrite(LED_BUILTIN, HIGH);
    led_state_entry_time = cur_time;

    return (VER_LED_ON);
  }
  else
  {
    return (VER_LED_SHORT_OFF);
  }
}  // short_off_state

/*========================================================================
 * State Function: long_off_state
 * 
 * Description:  
 *   This function implements the "long pause" between sequences.
 *   
 * Return Value:
 *   If we have more time to wait before starting our next sequence, stay in VER_LED_LONG_OFF.
 *   Otherwise, turn on the LED and go to VER_LED_ON
 */
ver_led_state_type long_off_state( void )
{
  unsigned long cur_time;
  
  //what time is it now?
  cur_time = millis();

  // has the LED been off long enough?
  if (cur_time > led_state_entry_time + led_long_off_time)
  {
    // Turn on the LED, mark the time, and reset the sequence counter.
    digitalWrite(LED_BUILTIN, HIGH);
    which_blink = 1;
    led_state_entry_time = cur_time;
    return(VER_LED_ON);
  }
  else
  {
    return(VER_LED_LONG_OFF);
  }
  
}  // long_off_state

/*===========================================================
 * DRIVER Function:  state_machine_driver
 * 
 * Description:  
 *   This is the driver function for the state machine.  
 *   It's currenly triggered by Timer1's compare ISR, and thus all of this
 *     code currenly runs in interrupt space.
 *   
 *  High level flow:
 *    VER_LED_INIT sets up the configuration.  Turn on the LED, mark the entry time, 
 *      and go to the VER_LED_ON state.
 *      
 *    In the VER_LED_ON state, we check to see how much time has passed.  If we've
 *      left the LED on long enough, we see how many consecutive blinks have happened, to 
 *      determine whether to go to a "short off" (if there are more blinks left) or a "long off"
 *      (if we've done a full sequence and need a long pause before the next sequence).
 *    
 *    VER_LED_SHORT_OFF checks time to see if our "quick pause" time is done.  If so, we'll turn the LED 
 *      back on and go to the VER_LED_ON state.  If not, we'll stay here.
 *      
 *    VER_LED_LONG_OFF checks time to see if our "long pause" time is done.  If so, we'll reset our 
 *      count for how many "blinks" we've done, mark the entry time, and turn the LED back on (going to
 *      VER_LED_ON).  If we still need to pause, we'll stay in this state.
 *
 */
static void state_machine_driver( void )
{

  ver_led_state_type        next_state; 
  
  switch (led_state)
  {
    case VER_LED_INIT:
      next_state = init_state();
    break;

    case VER_LED_ON:
      next_state = on_state();
    break;

    case VER_LED_SHORT_OFF:
      next_state = short_off_state();
    break;

    case VER_LED_LONG_OFF:
      next_state = long_off_state();
    break;

    // default:
      // no error handling.  Probably need something here...
    
  }  // end of switch on led state

#ifdef DEBUG_SERIAL
  Serial.print("Current State: ");
  Serial.print(led_state_strings[led_state]);
  Serial.print(" Next State: ");
  Serial.print(led_state_strings[next_state]);
  Serial.println();
#endif

  led_state = next_state;
}  // state_machine_driver

/*====================================================================
 * Time tick ISR
 *
 * This is the interrupt service routine that the arduino calls whenever our
 * 10ms timer expires.  Currenly we just run the state machine driver.
 */
ISR(TIMER1_COMPA_vect)
{
  // every 10ms, tick over the state machine.
  state_machine_driver();
}
