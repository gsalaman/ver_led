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
 *   - Periodically call ver_led_run in order to "tick" the processing forward.
 *   
 * Side Effects and Dependencies:
 *   This function will take over the built in LED.
 *   It's current implementation is polled, so any blocking calls outside of 
 *     this implementation can delay the timing of the blinks.
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

/*===========================================================
 * API Function:  ver_led_run
 * 
 * Description:  
 *   This is the driver function for the state machine.  
 *   It's currently impelmented as a polled time mechanism, so it 
 *   needs to be called periodically. 
 */
extern void ver_led_run( void );

#endif  // VER_LED_H
