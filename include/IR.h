//***************************************************************************************************
//*  IR.h -- Driver for Infrared receiver                                                           *
//***************************************************************************************************
//                                                                                                  *
//***************************************************************************************************
//

#include "Arduino.h"

#define IR_PIN   0          // Set IR pin to GPIO 0

char*      dbgprint ( const char* format, ... ) ;  // Print a formatted debug line

uint16_t   ir_preset1 = 0xA25D ;
uint16_t   ir_preset2 = 0x629D ;
uint16_t   ir_preset3 = 0xE21D ;
uint16_t   ir_preset4 = 0x22DD ;
uint16_t   ir_preset5 = 0x02FD ;
uint16_t   ir_preset6 = 0xC23D ;
uint16_t   ir_preset7 = 0xE01F ;
uint16_t   ir_preset8 = 0xA857 ;
uint16_t   ir_preset9 = 0x906F ;
uint16_t   ir_preset0 = 0x9867 ;
uint16_t   ir_stop    = 0xB04F ; // #
uint16_t   ir_play    = 0x6897 ; // *
uint16_t   ir_volup   = 0x18E7 ;
uint16_t   ir_voldown = 0x4AB5 ;
uint16_t   ir_mute    = 0x38C7 ;
uint16_t   ir_next    = 0x5AA5 ;
uint16_t   ir_prev    = 0x10EF ;

int        ir_intcount = 0 ;      // For test IR interrupts
uint16_t   ir_value = 0 ;         // IR code
uint32_t   ir_0 = 550 ;           // Average duration of an IR short pulse
uint32_t   ir_1 = 1650 ;          // Average duration of an IR long pulse



//**************************************************************************************************
//                                          I S R _ I R                                            *
//**************************************************************************************************
// Interrupts received from VS1838B on every change of the signal.                                 *
// Intervals are 640 or 1640 microseconds for data.  syncpulses are 3400 micros or longer.         *
// Input is complete after 65 level changes.                                                       *
// Only the last 32 level changes are significant and will be handed over to common data.          *
//**************************************************************************************************
void IRAM_ATTR isr_IR()
{
  static volatile uint32_t      t0 = 0 ;             // To get the interval
  static volatile uint32_t      ir_locvalue = 0 ;    // IR code
  static volatile int           ir_loccount = 0 ;    // Length of code

  uint32_t         t1, intval ;                      // Current time and interval since last change
  uint32_t         mask_in = 2 ;                     // Mask input for conversion
  uint16_t         mask_out = 1 ;                    // Mask output for conversion

  ir_intcount++ ;                                    // Test IR input.
  t1 = micros() ;                                    // Get current time
  intval = t1 - t0 ;                                 // Compute interval
  t0 = t1 ;                                          // Save for next compare
  if ( ( intval > 300 ) && ( intval < 800 ) )        // Short pulse?
  {
    ir_locvalue = ir_locvalue << 1 ;                 // Shift in a "zero" bit
    ir_loccount++ ;                                  // Count number of received bits
    ir_0 = ( ir_0 * 3 + intval ) / 4 ;               // Compute average durartion of a short pulse
  }
  else if ( ( intval > 1400 ) && ( intval < 1900 ) ) // Long pulse?
  {
    ir_locvalue = ( ir_locvalue << 1 ) + 1 ;         // Shift in a "one" bit
    ir_loccount++ ;                                  // Count number of received bits
    ir_1 = ( ir_1 * 3 + intval ) / 4 ;               // Compute average durartion of a short pulse
  }
  else if ( ir_loccount == 65 )                      // Value is correct after 65 level changes
  {
    while ( mask_in )                                // Convert 32 bits to 16 bits
    {
      if ( ir_locvalue & mask_in )                   // Bit set in pattern?
      {
        ir_value |= mask_out ;                       // Set set bit in result
      }
      mask_in <<= 2 ;                                // Shift input mask 2 positions
      mask_out <<= 1 ;                               // Shift output mask 1 position
    }
    ir_loccount = 0 ;                                // Ready for next input
  }
  else
  {
    ir_locvalue = 0 ;                                // Reset decoding
    ir_loccount = 0 ;
  }
}


//**************************************************************************************************
//                                   S E T U P I R                                                 *
//**************************************************************************************************
// Setup IR input.                                                                                 *
//**************************************************************************************************
void setupIR()
{
  dbgprint ( "Enable pin %d for IR", IR_PIN ) ;
  pinMode ( IR_PIN, INPUT ) ;                        // Pin for IR receiver VS1838B
  attachInterrupt ( IR_PIN, isr_IR, CHANGE ) ;       // Interrupts will be handled by isr_IR
}