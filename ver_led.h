#ifndef VER_LED_H
#define VER_LED_H

/*====================================================
 * VER_LED.H
 * 
 * This library uses the built-in LED on the main board to blink 
 * a "version number".
 * 
 * Example:  if setup with version=3, the board will do three short blinks, 
 *   followed by a pause, then repeat.
 * 
 * To use:  
 *   - Call ver_led_setup with the version number (or number of "blinks")
 *   
 * Side Effects and Dependencies:
 *   This function will take over the built in LED.
 *   Current implementation uses the TIMER1 ISR to tick it's internal state machine
 */
#include <Arduino.h>


//  Current range of "blinks" supported
#define VER_LED_MIN_VERSION 1
#define VER_LED_MAX_VERSION 5

//  Function return codes
#define VER_LED_OKAY   0
#define VER_LED_ERROR -1


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
extern int ver_led_setup( int version );

#endif  // VER_LED_H
