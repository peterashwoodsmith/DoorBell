//
// This ESP32 ARDUINO program is a Zibgee end device that will interact with a solenoid chime door bell. It allows the door bell
// button presses to become Zibgee binary sensors and allows playing of the bells via relays using inputs from zigbee.
// It uses the 12v-24v AC power of the normal door bell.
//
// HARDWARE:
//
// On an ESP32-C6 we have a factory reset button, inputs for two door bell buttons, and outputs for two relays which drive the
// door bell chimes. So you need a 12-24AC to 5v DC converter to drive it. The output of the AC to DC converter powers the 5V
// input and ground of the Esp board. The raw AC is however is fed into to one side of the bell solenoids while the other 
// side is switched by the normally open side of the switches. The switches are driven by the 3.3v output pins from the ESP32.
// As a result the ESP32 can detect when door bells buttons are pressed and can also trigger the bells in any patter in wants by
// simply setting the proper output pins that drive the switches. The exact timeing and sequence of course depends on the physical
// characterastics of the solenoids and experimentation is required to get the proper duraction of the 'true' output to get a good
// hard strike without undue buzzing.
//
// BUILD NOTES:
//          I built this on a Mac and had problems with the USB driver. Waveshare has a nice page describing how to put a new
//          driver on your Mac which worked perfectly. Without it one of my boards refused to load the code via Arduino but 
//          surprise a bunch of other boards worked just fine. Anyway if you get CRC errors downlaoding, try lower speeds and if that
//          fails pop over to Waveshare and lookup the USB driver problem.
//
//          ARDUINO IDE TOOLS SETTINGS:
//          You need to set a number of settings in the Arduino IDE/Tools menu for this to work properly.
//              1 - Tools/USB CDC on boot - enabled (allows serial IO for debugging).
//              2 - Tools/Core debug level (set as desired useful for debugging zibbee attach etc.) start verbose.
//              3 - Tools/Erase all flash before upload - this means each download its a brand new Zibgee end point.
//                  Id erase for first few downloads and always delete/reattach but after its working don't erease the
//                  flash each time. Once its working you can erase the flash and start scratch with a long press on the
//                  reset button anyway.
//              3 - Tools/partition scheme: 4MB with spiffs - seems to be what the Zibgee library wants.
//              4 - Tools/zibgee mode ED (end device) - you can also use the end mode with debug enabled for more tracing.
//
// SOFTWARE:
//
// The software has three interrupt handlers. The first handles the factory reset button which erases all the zibbee data so
// that rebinding is required. The second and third handlers will fire when one of the door bell buttons is pressed. 
// These handlers are a bit special because if the zigee connection is not up we don't want to ignore a door bell press so we
// simply pass the state of the button through to the switch. This in insures that if zibgee goes down or does not connect the
// door bells function with a single strike per button press each. 
//
// The setup() function of course configures zigbee clusters and sets all the attributes correctly then attaches to the 
// zigbee network. Once the network is up we enter the main loop().
//
// The main loop listens either for signs of button presses by the interrupt handlers, or from the HA binary switches. In 
// Either case it looks up the proper tones and repetitions and ask the relays to play that pattern. After they are finished
// it resets all the zibgee attributes. As a result zibgee will show when an external button is pressed so that can be used as
// a trigger for other things, and it also allows playing from zibgee. In addition two the two manual buttons I also provide a
// Z button which only can be triggered by Zibgee. This allows automations/notifications etc. to set a tone, repetition and 
// then request it be played. 
//
// There is a watch dog timers that is fed in the main loop and a simple blue flashing led when trying to bind to zibeee and
// a green flashing led when its fully bound.
//
// Since the solenoids/mechanical buttons can be quite noisy we need do quite a bit of debouncing. A few tricks are employed 
// here. First we look for enough '1's so we read a bit of the signal to ensure its steady enough to warrant an event.
// Next we completely ignore the interrupts if the solenoids are busy. Some external hardware would be useful here with some
// capacitors/diods and perhaps Schmidt triggers but this is simpler albeit a bit ugly.
//
// For debugging purposes we store a number of attributes in non volatile store (such as reboot reasons etc) and display them
// as clusters for debugging.
//
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
#include "Zigbee.h"
#include "esp_log.h"

//
// Hardware Pin configurations.
//
const int isr_resetButtonPin = 18;                      // Causes a factory reset by erasing all NVS
const int isr_door1ButtonPin = 2;                       // detects door bell button 1 when grounded
const int isr_door2ButtonPin = 3;                       // detects door bell button 2 when grounded
const int solenoid1Pin       = 10;                      // setting to high activates solenoid 1 to strike bar 1
const int solenoid2Pin       = 11;                      // setting to high activates solenoid 2 to strike bar 2

//
// Output unitless count app type missing so define it.
//
#define ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT  ESP_ZB_ZCL_AO_SET_APP_TYPE_WITH_ID( ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS, 0x0000)
//
// Debugging stuff, simple macro to log debug for us.
//
static const char *TAG = "zDoor"; 
#define DPRINTF(format, ...)  ESP_LOGD(TAG, format, ##__VA_ARGS__) 

//
// Set 1 and you'll get lots of useful info as it runs. For debugging the lower layer Zibgee see the tools settings
// in the Arduino menu for use with the debug enabled library and debug levels in that core. We can also compile in/out 
// the watch dog timers. 
//
const bool debug_g = false;
const bool wdt_g   = true;

// 
// Non volatile storage for debugging. When we restart etc. we will write the reasons 
// and track last uptime etc. for display via a Zigbee debug cluster sensor.
//
const char       *ha_nvs_name = "_MRVZDOOR_";              // Unique name for our partition
const char       *ha_nvs_vname= "_vars_";                  // name for our packeed variables
nvs_handle_t      ha_nvs_handle = 0;                       // Once open this is read/write to NVS
uint32_t          ha_nvs_last_uptime = 0;                  // minutes we were up last time before reboot
uint32_t          ha_nvs_last_reboot_reason = 0;           // why we rebooted last time. (0 factory reset)
uint32_t          ha_nvs_last_reboot_count = 0;            // increase each reboot except factory reset
volatile uint32_t isr_door1ButtonStatus = 0;               // goes true when door bell one is pressed
volatile uint32_t isr_door2ButtonStatus = 0;               // goes true when door bell two is pressed
volatile bool     isr_no_zigbee = true;                    // true when no zigbee available (default bell behavior)            
volatile bool     isr_ignore = true;                       // we set this to true while activing solenoids to minimize spurious

//
// We are looking for persistant values of the last reboot reason and last uptime. We store these two packed
// into a single Uint32 which we depack after reading from the NVS.
//
void ha_nvs_read()
{    
     ha_nvs_last_reboot_reason = 0;
     ha_nvs_last_uptime        = 0;
     ha_nvs_last_reboot_count  = 0;
     //
     esp_err_t err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        nvs_flash_erase();
        nvs_flash_init();
        if (debug_g) DPRINTF("ha_nvs_read - nvs_flash_init\n", esp_err_to_name(err));
     }
     err = nvs_open(ha_nvs_name, NVS_READWRITE, &ha_nvs_handle);
     if (err != ESP_OK) {
        if (debug_g) DPRINTF("ha_nvs_read - Error (%s) opening NVS name %s!\n", esp_err_to_name(err), ha_nvs_name);
        return;
     }  
     //
     uint32_t vars;
     err = nvs_get_u32(ha_nvs_handle, ha_nvs_vname, &vars);
     if (err != ESP_OK) {
          if (debug_g) DPRINTF("ha_nvs_read - cant get variable name %s\n", ha_nvs_name);
          return;
     }
     ha_nvs_last_reboot_reason  = vars         & 0xff;
     ha_nvs_last_reboot_count   = (vars >> 8)  & 0xff;
     ha_nvs_last_uptime         = (vars >> 16) & 0xffff;
     ha_nvs_last_reboot_reason += esp_reset_reason() * 1000;  // See below for why * 1000
     /* 
      * For convenient reference. We multiply these by 1000 to we can see the 
      * ESPs idea why it rebooted together with our own reboot reason as a single
      * number displayed as a Zigbee cluster. This is taken from the enum so they 
      * start at 0,1,2... 
      *
      * ESP_RST_UNKNOWN,    //!< Reset reason can not be determined
      * ESP_RST_POWERON,    //!< Reset due to power-on event
      * ESP_RST_EXT,        //!< Reset by external pin (not applicable for ESP32)
      * ESP_RST_SW,         //!< Software reset via esp_restart
      * ESP_RST_PANIC,      //!< Software reset due to exception/panic
      * ESP_RST_INT_WDT,    //!< Reset (software or hardware) due to interrupt watchdog
      * ESP_RST_TASK_WDT,   //!< Reset due to task watchdog
      * ESP_RST_WDT,        //!< Reset due to other watchdogs
      * ESP_RST_DEEPSLEEP,  //!< Reset after exiting deep sleep mode
      * ESP_RST_BROWNOUT,   //!< Brownout reset (software or hardware)
      * ESP_RST_SDIO,       //!< Reset over SDIO
      * ESP_RST_USB,        //!< Reset by USB peripheral
      * ESP_RST_JTAG,       //!< Reset by JTAG
      * ESP_RST_EFUSE,      //!< Reset due to efuse error
      * ESP_RST_PWR_GLITCH, //!< Reset due to power glitch detected
      * ESP_RST_CPU_LOCKUP, //!< Reset due to CPU lock up (double exception)
      */
     if (debug_g) {
          DPRINTF("ha_nvs_read got vars=%x, reason %d, count %d, uptime=%d\n", vars,
               ha_nvs_last_reboot_reason, ha_nvs_last_reboot_count, ha_nvs_last_uptime);
     }
}

//
// And here is the write to NVS of the attributes after they have been changed and sent to the Heat Pump
//
void ha_nvs_write(uint32_t reason = 0, uint32_t uptime = 0)
{
     ha_nvs_last_reboot_count = (ha_nvs_last_reboot_count + 1) & 0xff;
     reason &= 0xff;
     uptime &= 0x0000fffff;
     uint32_t vars  = reason | (ha_nvs_last_reboot_count << 8) | (uptime << 16);
     if (debug_g) {
          DPRINTF("ha_nvs_write got vars=%x, reason %d, count %d, uptime=%d\n", vars, reason, ha_nvs_last_reboot_count, uptime);
     }
     esp_err_t err = nvs_set_u32(ha_nvs_handle, ha_nvs_vname, vars);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write  %s can't write, because %s\n", ha_nvs_vname, esp_err_to_name(err));
         return;
     }
     err = nvs_commit(ha_nvs_handle);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write %s can't commit, because %s\n", ha_nvs_vname, esp_err_to_name(err));
     }  
}
// 
// Function complete shutdown and restart. Forward declared also a flash sequence for factory reset.
//
extern void ha_restart(uint32_t reason, uint32_t uptime); 
extern void rgb_led_set_factory_reset();

//
// Interrupt handler for Reset button. If its pressed we do full factory reset. Normal reset is just done with a power off/on.
// We just look to see if we are getting a bunch of lows on the reset pin and if so we reset otherwise just ignore it as we can
// get spurious interrupts when the solenoids activate.
//
void isr_resetButtonPress()      
{    if (isr_ignore) return;                            // We ignore interrupts while playing solenoids
     int n = 1;
     for(int i = 0; i < 50; i++) {
         n += (digitalRead(isr_resetButtonPin) == 0) ? 1 : 0;
     }
     if (n < 40) return;                                // if its too much like noise ignore it. 
     //
     rgb_led_set_factory_reset();                       // Go white so its obvious
     Zigbee.factoryReset(false);                        // This should do the same but not sure it does anyway ...
     ha_restart(0, 0);                                  // And stop all the Zigbee stuff and just restart the ESP
}

//
// Door bell button 1 or 2 pressed or released (changed) so if no zigbee connection we just reflect this directly to the solenoid
// pins however if zibgee is up it is processed in the main loop. Note that we want to see a bit of a solid edge before declaring
// a press so we look ahead a bit in time to make sure its staying low.
//
void isr_door1ButtonPress()
{    if (isr_ignore) return;
     unsigned int pressed = (digitalRead(isr_door1ButtonPin) == 0) ? 1 : 0;
     if (isr_no_zigbee) {
         digitalWrite(solenoid1Pin, pressed);
         return;
     } 
     if (pressed) { 
         int n = 1;
         for(int i = 0; i < 50; i++) {
             n += (digitalRead(isr_door1ButtonPin) == 0) ? 1 : 0;
         }
         if (n < 40) return;
         isr_door1ButtonStatus += 1;   
     }    
}
//
void isr_door2ButtonPress()
{    if (isr_ignore) return;
     unsigned int pressed = (digitalRead(isr_door2ButtonPin) == 0) ? 1 : 0;
     if (isr_no_zigbee) {
         digitalWrite(solenoid2Pin, pressed);    
         return;
     } 
     if (pressed) { 
         int n = 1;
         for(int i = 0; i < 50; i++) {
             n += (digitalRead(isr_door2ButtonPin) == 0) ? 1 : 0;
         }
         if (n < 40) return;
         isr_door2ButtonStatus += 1;   
     }            
}

//
// Setup the Input and Output Interrupt service routine for the reset button. Just call the isr_resetButtonPress routing when the pin goes LOW.
// This triggers a factory reset.
//
void hw_setup()
{   
     pinMode(isr_resetButtonPin, INPUT_PULLUP); 
     attachInterrupt(digitalPinToInterrupt(isr_resetButtonPin), isr_resetButtonPress,   FALLING);  
     //
     pinMode(isr_door1ButtonPin, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(isr_door1ButtonPin), isr_door1ButtonPress,   CHANGE);   
     //
     pinMode(isr_door2ButtonPin, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(isr_door2ButtonPin), isr_door2ButtonPress,   CHANGE);    
     //
     pinMode(solenoid1Pin, OUTPUT);
     pinMode(solenoid2Pin, OUTPUT);
     digitalWrite(solenoid1Pin, LOW);
     digitalWrite(solenoid2Pin, LOW);
     isr_door1ButtonStatus = 0;
     isr_door2ButtonStatus = 0;
     isr_no_zigbee = true;
     isr_ignore = true;
}

//
// If the task watch dog times out, rather than use the system handler it will come here and we do a nice
// controlled reboot and keep track of the reason and how long we were up for better debugging.
//
void esp_task_wdt_isr_user_handler(void)
{
     ha_restart(1, millis()/1000); 
}

//
// Debugg Clusters
//
ZigbeeAnalog      zbRebootReason  = ZigbeeAnalog(10);      // reason for last reboot
ZigbeeAnalog      zbLastUptime    = ZigbeeAnalog(11);      // How long it was up last time before reboot
ZigbeeAnalog      zbRebootCount   = ZigbeeAnalog(12);      // How many reboots since factory reset
ZigbeeAnalog      zbUptime        = ZigbeeAnalog(13);      // Seconds since last reboot.
//
ZigbeeBinary      zbDoor1Button   = ZigbeeBinary(14);      // Door button 1
ZigbeeBinary      zbDoor2Button   = ZigbeeBinary(15);      // Door button 2
ZigbeeBinary      zbDoorZButton   = ZigbeeBinary(18);      // button HA can press to cause a tone (yes its 18)
ZigbeeAnalog      zbDoor1Play     = ZigbeeAnalog(16);      // Tone to play when door 1 pressed.
ZigbeeAnalog      zbDoor2Play     = ZigbeeAnalog(17);      // Tone to play when door 2 plressed.
ZigbeeAnalog      zbDoorZPlay     = ZigbeeAnalog(19);      // tone to play when HA presses the Z button
ZigbeeAnalog      zbDoor1PlayReps = ZigbeeAnalog(20);      // how many times to repeat the tones for each button
ZigbeeAnalog      zbDoor2PlayReps = ZigbeeAnalog(21);
ZigbeeAnalog      zbDoorZPlayReps = ZigbeeAnalog(22);

//
// These are the variables that maintain the state of what HA has asked to be set
// set.
//
volatile unsigned int ha_door1ButtonStatus   = 0;    // if HA thinks door button 1 is pressed or not (can be set by another task)
volatile unsigned int ha_door2ButtonStatus   = 0;    // if HA thinks door button 2 is pressed or not (can be set by another task)
//
unsigned int ha_doorZButtonStatus   = 0;    // if HA presses this button we play
unsigned int ha_door1PlayStatus     = 3;    // tone to play when button 1 is pressed Ding Dong Ding Dong Ding Dong
unsigned int ha_door2PlayStatus     = 2;    // tone to play when button 2 is pressed Dong Dong Dong Dong Dong Dong
unsigned int ha_doorZPlayStatus     = 4;    // tone to play when HA triggers doorZ button
unsigned int ha_door1PlayReps       = 3;    // how many times to repeat tones for the given trigger
unsigned int ha_door2PlayReps       = 3;
unsigned int ha_doorZPlayReps       = 3;
//
// Keep HA up to date with any changes that happen on the heat pump from the serial updates.
//
void ha_sync_status()
{
     if (debug_g) DPRINTF("HA sync %d %d\n", ha_door1ButtonStatus, ha_door2ButtonStatus);
     zbDoor1Play.setAnalogOutput(ha_door1PlayStatus);
     zbDoor2Play.setAnalogOutput(ha_door2PlayStatus);
     zbDoorZPlay.setAnalogOutput(ha_doorZPlayStatus);

     zbDoor1PlayReps.setAnalogOutput(ha_door1PlayReps);
     zbDoor2PlayReps.setAnalogOutput(ha_door2PlayReps);
     zbDoorZPlayReps.setAnalogOutput(ha_doorZPlayReps);

     zbDoor1Button.setBinaryInput(ha_door1ButtonStatus);
     zbDoor1Button.setBinaryOutput(ha_door1ButtonStatus);
     zbDoor2Button.setBinaryInput(ha_door2ButtonStatus);
     zbDoor2Button.setBinaryOutput(ha_door2ButtonStatus);

     zbDoorZButton.setBinaryOutput(ha_doorZButtonStatus);    // HA can press this button to cause sound but
     zbDoorZButton.setBinaryInput(ha_doorZButtonStatus);     // we reset it after we've finished playing.

     zbRebootReason.setAnalogInput(ha_nvs_last_reboot_reason);
     zbLastUptime.setAnalogInput(ha_nvs_last_uptime);
     zbRebootCount.setAnalogInput(ha_nvs_last_reboot_count);
     zbUptime.setAnalogInput(millis()/1000);
     //
     zbDoor1Play.reportAnalogOutput();
     zbDoor2Play.reportAnalogOutput();
     zbDoorZPlay.reportAnalogOutput();

     zbDoor1PlayReps.reportAnalogOutput();
     zbDoor2PlayReps.reportAnalogOutput();
     zbDoorZPlayReps.reportAnalogOutput();
     
     zbDoor1Button.reportBinaryInput();
     zbDoor1Button.reportBinaryOutput();
     zbDoor2Button.reportBinaryInput();
     zbDoor2Button.reportBinaryOutput();
     zbDoorZButton.reportBinaryInput();
     zbDoorZButton.reportBinaryOutput();

     zbRebootReason.reportAnalogInput();
     zbLastUptime.reportAnalogInput();
     zbRebootCount.reportAnalogInput();
     zbUptime.reportAnalogInput();
}

//
// These are just useful debugging functions to display the attributes that HA has given us.
// One for each attributes.
// 
void ha_displayDoor1ButtonStatus()
{    DPRINTF("Door1ButtonStatus   = %d\n", ha_door1ButtonStatus);
}
// 
void ha_displayDoor2ButtonStatus()
{    DPRINTF("Door2ButtonStatus   = %d\n", ha_door2ButtonStatus);
}
// 
void ha_displayDoorZButtonStatus()
{    DPRINTF("DoorZButtonStatus   = %d\n", ha_doorZButtonStatus);
}

void ha_displayDoor1PlayStatus()
{
     DPRINTF("Door1PlayStatus     = %d\n", ha_door1PlayStatus); 
}
//
void ha_displayDoor2PlayStatus()
{
     DPRINTF("Door2PlayStatus     = %d\n", ha_door2PlayStatus); 
}
//
void ha_displayDoorZPlayStatus()
{
     DPRINTF("DoorZPlayStatus     = %d\n", ha_doorZPlayStatus); 
}
//
//-------
//
void ha_displayDoor1PlayReps()
{
     DPRINTF("Door1PlayReps     = %d\n", ha_door1PlayReps); 
}
//
void ha_displayDoor2PlayReps()
{
     DPRINTF("Door2PlayReps     = %d\n", ha_door2PlayReps); 
}
//
void ha_displayDoorZPlayReps()
{
     DPRINTF("DoorZPlayReps     = %d\n", ha_doorZPlayReps); 
}
//
// ---------
//
void ha_setDoor1PlayStatus(float value)
{
     ha_door1PlayStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor1PlayStatus(); }
}
//
void ha_setDoor2PlayStatus(float value)
{
     ha_door2PlayStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor2PlayStatus(); }
}
//
void ha_setDoorZPlayStatus(float value)
{
     ha_doorZPlayStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoorZPlayStatus(); }
}
//
// -----
//
void ha_setDoor1PlayReps(float value)
{
     ha_door1PlayReps = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor1PlayReps(); }
}
//
void ha_setDoor2PlayReps(float value)
{
     ha_door2PlayReps = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor2PlayReps(); }
}
//
void ha_setDoorZPlayReps(float value)
{
     ha_doorZPlayReps = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoorZPlayReps(); }
}
//
void ha_setDoor1ButtonStatus(bool value)
{
     ha_door1ButtonStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor1ButtonStatus(); }  
}
//
void ha_setDoor2ButtonStatus(bool value)
{
     ha_door2ButtonStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoor2ButtonStatus(); }  
}
//
void ha_setDoorZButtonStatus(bool value)
{
     ha_doorZButtonStatus = value;
     if (debug_g) { DPRINTF("HA=> "); ha_displayDoorZButtonStatus(); }  
}

//
// We use the color RGB LED to indicate state.
//
const uint8_t RGB_LED_OFF    = 0;        // Enums are causing compiler problems when passed as first argument.
const uint8_t RGB_LED_WHITE  = 1;        // so back to old school.
const uint8_t RGB_LED_RED    = 2;
const uint8_t RGB_LED_GREEN  = 3;
const uint8_t RGB_LED_BLUE   = 4;
const uint8_t RGB_LED_ORANGE = 5;
const uint8_t RGB_MAX        = RGB_BRIGHTNESS/8;      // Fairly dim or they keep people awake
const uint8_t RGB_MIN        = 0;
#define       RGB_ORDER        LED_COLOR_ORDER_RGB    // Compiler problems passing enums, have to be explicit
//
void rgb_led_set(int color) {
     switch(color) {                                  //RED     GREEN     BLUE
         case RGB_LED_GREEN : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MAX,  RGB_MIN); break;
         case RGB_LED_WHITE : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MAX,  RGB_MAX); break;
         case RGB_LED_RED   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MIN,  RGB_MIN); break;          
         case RGB_LED_BLUE  : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  RGB_MAX); break;
         case RGB_LED_ORANGE: rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MAX/2,RGB_MIN); break;
         case RGB_LED_OFF   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  RGB_MIN); break;
     }
}
//
//   Simple LED flash routine, should really use a background task to do this.. tbd. Flash the chosen color and
//   then return to the restore color after.
//
void rgb_led_flash(int color, int restore_color)
{
     for(int i = 0; i < 5; i++) {
        rgb_led_set(color);
        delay(50);
        rgb_led_set(RGB_LED_OFF);
        delay(50);
     }
     rgb_led_set(restore_color);
}

//
// We are in an interrupt handler for the reset button and will indicate a factory reset.
// Just use white for now.
//
void rgb_led_set_factory_reset()
{
     rgb_led_set(RGB_LED_WHITE);
}

//
// Called when device is asked to identify itself. We will just flash alternating white and green for 1/2 second or so.
// We get called 5 or 6 times with x = 5, 4, ... down to 0. We only triggers the flashing on the 0 call.
//
void ha_identify(uint16_t x)
{
     if (debug_g) DPRINTF("******** HA => IDENTIFY(%d) ******\n", (int) x);
     //
     if (x == 0) {
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500);
        rgb_led_flash(RGB_LED_GREEN, RGB_LED_GREEN);
        delay(500);
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500); 
        rgb_led_set(RGB_LED_GREEN);
     }
}

//
// Complete restart for some reason and we've been up for some amount of time. Write this to the 
// NVS for display after reboot.
//
void ha_restart(uint32_t reason, uint32_t uptime)
{  
     ha_nvs_write(reason, uptime);        // remember why we are restarting so it can be shown in HA next time
     rgb_led_set(RGB_LED_OFF);            // Sometimes gets stuck on, don't know why perhaps timing.      
     delay(100);
     rgb_led_set(RGB_LED_OFF);            // So do it twice .
     delay(100);
     if (debug_g) DPRINTF("Restarting...\n"); 
     Zigbee.closeNetwork();
     Zigbee.stop();
     delay(100);
     ESP.restart();
}

//
// Make the DING sound, just strike solenoid 1 briefly and return it. The duration to hold
// the solenoid closed needs to be tuned to minimum. Otherwise the buzz unnecessarily.
//
void DING() {
     digitalWrite(solenoid1Pin, true);
     delay(100);                            // how long to hold electromagnet
     digitalWrite(solenoid1Pin, false);
     delay(250);
}

//
// Make the DONG sound, just strike solenoid 2 briefly and return it.
//
void DONG() {
     digitalWrite(solenoid2Pin, true);
     delay(100);
     digitalWrite(solenoid2Pin, false);
     delay(250);
}

void ALARMDING()
{    for(int i = 0; i < 20; i++) {
        digitalWrite(solenoid1Pin, true);
        delay(100);
        digitalWrite(solenoid1Pin, false);
        delay(75);
     }
}

//
// Strike the solenoids according to the mode pattern
//
void solenoidsStrike(unsigned mode)
{    switch(mode) {
          case 1: DING(); DING();              break;
          case 2: DONG(); DONG();              break;
          case 3: DING(); DONG();              break;
          case 4: DONG(); DING();              break;
          case 5: DING(); DING(); DONG();      break;
          case 6: DONG(); DONG(); DING();      break;
          case 7: DONG(); DING(); DONG();      break;
          case 8: DING(); DONG(); DING();      break;
          case 9: DONG(); DONG(); DONG();      break;
          case 10:ALARMDING();                 break;
          default:                             break;
     }
}

//
// When one of the buttons is pressed this function is called with the mode corresponding to the desired behavior 
// for the given button. The sequence is played three times with a short pause between. While playing the solenoids 
// we ignore any incomming interrupts as we sometimes get spurious noise. Easier to do this than to filter it out
// at the hardware level for now.
//
void solenoidsPlay(unsigned mode, int reps)
{    if (debug_g) 
         DPRINTF("solenoidsPlay %d\n", mode);
     isr_ignore = true;
     for(int i = 0; i < reps; i++) {
         solenoidsStrike(mode);
         delay(100);
     }
     isr_ignore = false;
}

//
// Force All solenoids off. We ignore any interrupts while doing this in case one of them switches off and makes some noise.
// They should be off anyway.
//
void solenoids_reset()
{    isr_ignore = true;
     digitalWrite(solenoid1Pin, false); 
     digitalWrite(solenoid2Pin, false);
     isr_ignore = false;
}

// 
// We woke up, configure zibgee and wait for connection, then process any pending requests
// and go back to sleep. 
//
void setup() {
     //
     // Until we have zibgee connection the button interrupts do normal door bell operation.
     //
     isr_no_zigbee = true;
     isr_ignore = true;

     //
     // Debug stuff
     //
     if (debug_g) {
         esp_log_level_set(TAG, ESP_LOG_DEBUG);
         DPRINTF("RiverView S/W Zibgee 3.0 Solenoid Door Bells");
     }
     //
     // We get debug information from last reboot (uptime and reboot reason etc.)
     ha_nvs_read();
     //
     // Get everything back to square one, we don't always power reset and I'm not convinced these get reset
     // as globals when a panic restart happens.
     //
     ha_door1ButtonStatus  = 0;    // if HA thinks door button 1 is pressed or not
     ha_door2ButtonStatus  = 0;    // if HA thinks door button 2 is pressed or not
     ha_doorZButtonStatus  = 0;
     ha_door1PlayStatus    = 3;    // tone to play when button 1 is pressed
     ha_door2PlayStatus    = 2;    // tone to play when button 2 is pressed
     ha_doorZPlayStatus    = 4;    // tone to play when zigbee button pressed by HA
     ha_door1PlayReps      = 3;    // How many times to repeat tones
     ha_door2PlayReps      = 3;
     ha_doorZPlayReps      = 3;

     //
     // Watch dog timer on this task to panic if we don't get to main loop regulary.
     //
     if (wdt_g) {
         static const esp_task_wdt_config_t wdt_config = {                  // MUST BE CONST!!
              .timeout_ms = 10 * 60 * 1000,                                 // 10 minutes max to get back to main loop()
              .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Bitmask of all cores
              .trigger_panic = true };                                      // no panic, just restart
         esp_task_wdt_reconfigure(&wdt_config);
         esp_task_wdt_add(NULL);
         esp_task_wdt_status(NULL);
     }
     //
     rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
     //
     // enable interrupts for reset buttons door bell buttons and configure solenoids outputs
     //
     hw_setup();
     //
     // Add the zibgee clusters (buttons/sliders etc.)
     //
     const char *MFGR = "zRiverView";    // Because my home office looks out over the ottwawa river ;)
     const char *MODL = "zzDoor";        // Door bell interface
     //
     if (debug_g) DPRINTF("Door 1 play\n");
     zbDoor1Play.setManufacturerAndModel(MFGR,MODL);
     zbDoor1Play.addAnalogOutput();
     zbDoor1Play.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoor1Play.setAnalogOutputDescription("Tones1 (0=off)");
     zbDoor1Play.setAnalogOutputResolution(1);
     zbDoor1Play.setAnalogOutputMinMax(0, 10);  
     zbDoor1Play.onAnalogOutputChange(ha_setDoor1PlayStatus);
     //
     if (debug_g) DPRINTF("Door 2 play\n");
     zbDoor2Play.setManufacturerAndModel(MFGR,MODL);
     zbDoor2Play.addAnalogOutput();
     zbDoor2Play.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoor2Play.setAnalogOutputDescription("Tones2 (0=off)");
     zbDoor2Play.setAnalogOutputResolution(1);
     zbDoor2Play.setAnalogOutputMinMax(0, 10);  
     zbDoor2Play.onAnalogOutputChange(ha_setDoor2PlayStatus);
     //
     if (debug_g) DPRINTF("Door Z play\n");
     zbDoorZPlay.setManufacturerAndModel(MFGR,MODL);
     zbDoorZPlay.addAnalogOutput();
     zbDoorZPlay.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoorZPlay.setAnalogOutputDescription("TonesZ (0=off)");
     zbDoorZPlay.setAnalogOutputResolution(1);
     zbDoorZPlay.setAnalogOutputMinMax(0, 10);  
     zbDoorZPlay.onAnalogOutputChange(ha_setDoorZPlayStatus);
     //
     if (debug_g) DPRINTF("Door 1 reps\n");
     zbDoor1PlayReps.setManufacturerAndModel(MFGR,MODL);
     zbDoor1PlayReps.addAnalogOutput();
     zbDoor1PlayReps.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoor1PlayReps.setAnalogOutputDescription("Repetitions1");
     zbDoor1PlayReps.setAnalogOutputResolution(1);
     zbDoor1PlayReps.setAnalogOutputMinMax(0, 10);  
     zbDoor1PlayReps.onAnalogOutputChange(ha_setDoor1PlayReps);
     //
     if (debug_g) DPRINTF("Door 2 reps\n");
     zbDoor2PlayReps.setManufacturerAndModel(MFGR,MODL);
     zbDoor2PlayReps.addAnalogOutput();
     zbDoor2PlayReps.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoor2PlayReps.setAnalogOutputDescription("Repetitions2");
     zbDoor2PlayReps.setAnalogOutputResolution(1);
     zbDoor2PlayReps.setAnalogOutputMinMax(0, 10);  
     zbDoor2PlayReps.onAnalogOutputChange(ha_setDoor2PlayReps);
     //
     if (debug_g) DPRINTF("Door Z reps\n");
     zbDoorZPlayReps.setManufacturerAndModel(MFGR,MODL);
     zbDoorZPlayReps.addAnalogOutput();
     zbDoorZPlayReps.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbDoorZPlayReps.setAnalogOutputDescription("RepetitionsZ");
     zbDoorZPlayReps.setAnalogOutputResolution(1);
     zbDoorZPlayReps.setAnalogOutputMinMax(0, 10);  
     zbDoorZPlayReps.onAnalogOutputChange(ha_setDoorZPlayReps);
     //
     if (debug_g) DPRINTF("Door Button 1\n");
     zbDoor1Button.setManufacturerAndModel(MFGR,MODL);
     zbDoor1Button.addBinaryInput();
     zbDoor1Button.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoor1Button.setBinaryInputDescription("Button1");
     zbDoor1Button.addBinaryOutput();
     zbDoor1Button.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoor1Button.setBinaryOutputDescription("Play1");
     zbDoor1Button.onBinaryOutputChange(ha_setDoor1ButtonStatus);
     //
     if (debug_g) DPRINTF("Door Button 2\n");
     zbDoor2Button.setManufacturerAndModel(MFGR,MODL);
     zbDoor2Button.addBinaryInput();
     zbDoor2Button.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoor2Button.setBinaryInputDescription("Button2");
     zbDoor2Button.addBinaryOutput();
     zbDoor2Button.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoor2Button.setBinaryOutputDescription("Play2");
     zbDoor2Button.onBinaryOutputChange(ha_setDoor2ButtonStatus);
     //
     if (debug_g) DPRINTF("Door Button Z\n");
     zbDoorZButton.setManufacturerAndModel(MFGR,MODL);
     zbDoorZButton.addBinaryInput();
     zbDoorZButton.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoorZButton.setBinaryInputDescription("ButtonZ");
     zbDoorZButton.addBinaryOutput();
     zbDoorZButton.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbDoorZButton.setBinaryOutputDescription("PlayZ");
     zbDoorZButton.onBinaryOutputChange(ha_setDoorZButtonStatus);
     //
     if (debug_g) DPRINTF("RebootReason cluster\n");
     zbRebootReason.setManufacturerAndModel(MFGR,MODL);
     zbRebootReason.addAnalogInput();
     zbRebootReason.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootReason.setAnalogInputDescription("Last Reboot Reason");
     zbRebootReason.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("LastUptime cluster\n");
     zbLastUptime.setManufacturerAndModel(MFGR,MODL);
     zbLastUptime.addAnalogInput();
     zbLastUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbLastUptime.setAnalogInputDescription("Last Uptime s");
     zbLastUptime.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("RebootCount cluster\n");
     zbRebootCount.setManufacturerAndModel(MFGR,MODL);
     zbRebootCount.addAnalogInput();
     zbRebootCount.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootCount.setAnalogInputDescription("Reboot Count");
     zbRebootCount.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("This Uptime\n");
     zbUptime.setManufacturerAndModel(MFGR,MODL);
     zbUptime.addAnalogInput();
     zbUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbUptime.setAnalogInputDescription("This Uptime s");
     zbUptime.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("Set mains power\n");
     zbDoor1Button.setPowerSource(ZB_POWER_SOURCE_MAINS); 
     zbDoor1Button.onIdentify(ha_identify);
     //
     Zigbee.addEndpoint(&zbDoor1Button);
     Zigbee.addEndpoint(&zbDoor2Button);
     Zigbee.addEndpoint(&zbDoorZButton);
     Zigbee.addEndpoint(&zbDoor1Play);
     Zigbee.addEndpoint(&zbDoor2Play);
     Zigbee.addEndpoint(&zbDoorZPlay);
     Zigbee.addEndpoint(&zbDoor1PlayReps);
     Zigbee.addEndpoint(&zbDoor2PlayReps);
     Zigbee.addEndpoint(&zbDoorZPlayReps);
     Zigbee.addEndpoint(&zbRebootReason);
     Zigbee.addEndpoint(&zbLastUptime);
     Zigbee.addEndpoint(&zbRebootCount);
     Zigbee.addEndpoint(&zbUptime);
     //
     // Create a custom Zigbee configuration for End Device with longer timeouts/keepalive
     //
     esp_zb_cfg_t zigbeeConfig =                             \
       {  .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,              \
          .install_code_policy = false,                      \
          .nwk_cfg = {                                       \
            .zed_cfg =  {                                    \                                                          
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN, \
                .keep_alive = 3000,                          \
              },                                             \
          },                                                 \
       };
     //
     if (debug_g) DPRINTF("Starting Zigbee\n");
     rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);
     //
     // When all EPs are registered, start Zigbee in End Device mode
     //
     isr_ignore = false; // allow factory reset from this point on.
     //
     if (!Zigbee.begin(&zigbeeConfig, false)) { 
        if (debug_g) {
            DPRINTF("Zigbee failed to start!\n");
            DPRINTF("Rebooting ESP32!\n");
        }
        for(int i = 0; i < 10; i++) {
           rgb_led_flash(RGB_LED_RED, RGB_LED_WHITE);
           rgb_led_flash(RGB_LED_WHITE, RGB_LED_RED);
        }
        ha_restart(3, millis()/1000);             // restart and remember why
     }
     //
     // Now connect to network.
     //
     if (debug_g) DPRINTF("Connecting to network\n");   
     int tries = 0;      
     while (!Zigbee.connected()) {
        rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);         // the led sets have delays built in
        delay(5000);
        if (debug_g) DPRINTF("connecting..\n");
        if (tries ++ > 360) {                              // Maximum 30 minutes trying    
           if (debug_g) {
               DPRINTF("Zigbee failed to connect!\n");
               DPRINTF("Rebooting ESP32!\n");
           }
           rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);  // We tried for 30 minutes, restart.
           rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
           ha_restart(4, millis()/1000);   
        }
     }
     rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);   
     if (debug_g) DPRINTF("Successfully connected to Zigbee network\n");
     //
     // Update the debug related information to HA.
     //
     ha_sync_status();
     //
     // During Zigbee attach the button press will cause normal door bell operation,
     // once we have zigbee its handled in the main loop.
     //
     isr_no_zigbee = false;
}

//
// In the loop we just process any pending requests, the requests came via the callbacks.
// We use a watch dog timer to make sure we get into this loop at least every 10 minutes or so.
// If we don't get here in time we will get a hardware reset.
//
void loop()
{    static int ix = 0;            // Loop counter 0..4 for LED on/of flash choice.
     //
     solenoids_reset();            // Make sure solenoids are off
     //
     if (!Zigbee.connected()) {
         isr_no_zigbee = true;
         if (debug_g) DPRINTF("zigbee disconnected while in loop()- restarting\n");
         ha_restart(5, millis()/1000);   
     }
     //
     int status_color = RGB_LED_GREEN;
     //
     // And feed the watch dog because all is well, zigbee ok and serial comms ok.
     // 
     if (wdt_g) esp_task_wdt_reset();  
     //
     // Check to see if either door bell is pressed. We can check the status later for changes.
     // Must or with existing status because it may have been changed by HA.
     //       
     ha_door1ButtonStatus |= isr_door1ButtonStatus > 0;
     ha_door2ButtonStatus |= isr_door2ButtonStatus > 0;
     //
     if (debug_g && (isr_door1ButtonStatus + isr_door2ButtonStatus) > 0) {
         DPRINTF("loop() Door1=%d Door2=%d\n", isr_door1ButtonStatus, isr_door2ButtonStatus);
     }
     //
     // Now do any actual work based on the button status. We set the sensor output to true to indicate pressed status.
     // Then play the solenoids, then reset the triggers that caused it (interrupts or HA setting the binary status). 
     // Then update HA input/output to reflect we are all done. This toggles the activiation switch on HA back off.
     // Its basically the same for all triggers except the Z button which has no interrupt source locally.
     // 
     if (ha_door1ButtonStatus == true) {
         zbDoor1Button.setBinaryInput(true);      // report sensor back to HA that we are playing it now.
         zbDoor1Button.reportBinaryInput();
         solenoidsPlay(ha_door1PlayStatus, ha_door1PlayReps);
         isr_door1ButtonStatus = 0;               // Ignore any buttons pressed while we were playing tones.
         ha_door1ButtonStatus = false;
         zbDoor1Button.setBinaryInput(false);     // report sensor back to HA that we are playing it now.
         zbDoor1Button.reportBinaryInput();
         zbDoor1Button.setBinaryOutput(false);
         zbDoor1Button.reportBinaryOutput();
     }
     //
     if (ha_door2ButtonStatus == true) {
         zbDoor2Button.setBinaryInput(true);      
         zbDoor2Button.reportBinaryInput();
         solenoidsPlay(ha_door2PlayStatus, ha_door2PlayReps);
         isr_door2ButtonStatus = 0;                
         ha_door2ButtonStatus = false;
         zbDoor2Button.setBinaryInput(false);      
         zbDoor2Button.reportBinaryInput();
         zbDoor2Button.setBinaryOutput(false);
         zbDoor2Button.reportBinaryOutput();
     }
     //
     if (ha_doorZButtonStatus == true) {
         zbDoorZButton.setBinaryInput(true);       
         zbDoorZButton.reportBinaryInput();
         solenoidsPlay(ha_doorZPlayStatus, ha_doorZPlayReps);
         ha_doorZButtonStatus = false;
         zbDoorZButton.setBinaryInput(false);      
         zbDoorZButton.reportBinaryInput();
         zbDoorZButton.setBinaryOutput(false);
         zbDoorZButton.reportBinaryOutput();
     }
     //
     // Every so often (5 mins) we update the HA with all all attributes.
     //
     {  const unsigned long  MAX_TIME              = 60*5;
        static unsigned long last_update_time      = 0;
        unsigned long        now_time              = millis() / 1000;
        //
        if ((last_update_time + MAX_TIME) <= now_time) {
            last_update_time = now_time;
            ha_sync_status();                 
        }
     }
     //
     // Led is off for 4 seconds, then 1 second on Green .. etc. to indicate zibgee is up and ok.
     //
     rgb_led_set(ix > 1 ? RGB_LED_OFF : status_color);
     ix = (ix + 1) % 10;
     //
     // No need to buzz this loop, little pause is fine.
     //
     delay(500);
}
