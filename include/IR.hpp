//***************************************************************************************************
//   IR.h - Driver for Infrared receiver                                                            *
//***************************************************************************************************
//                                                                                                  *
//***************************************************************************************************
//

#include "Arduino.h"


extern       char* dbgprint ( const char* format, ... ) ;
extern const char* analyzeCmd ( const char* str ) ;

extern uint16_t ir_preset1 ;
extern uint16_t ir_preset2 ;
extern uint16_t ir_preset3 ;
extern uint16_t ir_preset4 ;
extern uint16_t ir_preset5 ;
extern uint16_t ir_preset6 ;
extern uint16_t ir_preset7 ;
extern uint16_t ir_preset8 ;
extern uint16_t ir_preset9 ;
extern uint16_t ir_preset0 ;
extern uint16_t ir_stop ;
extern uint16_t ir_play ;
extern uint16_t ir_volup ;
extern uint16_t ir_voldown ;
extern uint16_t ir_mute ;
extern uint16_t ir_next ;
extern uint16_t ir_prev ;

       int      ir_intcount = 0 ;              // For test IR interrupts
       uint16_t ir_value = 0 ;                 // IR code
       uint32_t ir_0 = 550 ;                   // Average duration of an IR short pulse
       uint32_t ir_1 = 1650 ;                  // Average duration of an IR long pulse


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
//                                     S C A N I R                                                 *
//**************************************************************************************************
// See if IR input is available.  Execute the programmed command.                                  *
//**************************************************************************************************
void scanIR()
{
  const char* reply ;                                       // Result of analyzeCmd

  if ( ir_value )                                           // Any input?
  {
    if ( ir_value == ir_preset1 )
    {
      dbgprint ( "IR code %04X - ir_preset1", ir_value ) ;
      reply = analyzeCmd ("preset=1") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset2 )
    {
      dbgprint ( "IR code %04X - ir_preset2", ir_value ) ;
      reply = analyzeCmd ("preset=2") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset3 )
    {
      dbgprint ( "IR code %04X - ir_preset3", ir_value ) ;
      reply = analyzeCmd ("preset=3") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset4 )
    {
      dbgprint ( "IR code %04X - ir_preset4", ir_value ) ;
      reply = analyzeCmd ("preset=4") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset5 )
    {
      dbgprint ( "IR code %04X - ir_preset5", ir_value ) ;
      reply = analyzeCmd ("preset=5") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset6 )
    {
      dbgprint ( "IR code %04X - ir_preset3", ir_value ) ;
      reply = analyzeCmd ("preset=6") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset7 )
    {
      dbgprint ( "IR code %04X - ir_preset7", ir_value ) ;
      reply = analyzeCmd ("preset=7") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset8 )
    {
      dbgprint ( "IR code %04X - ir_preset8", ir_value ) ;
      reply = analyzeCmd ("preset=8") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset9 )
    {
      dbgprint ( "IR code %04X - ir_preset9", ir_value ) ;
      reply = analyzeCmd ("preset=9") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_preset0 )
    {
      dbgprint ( "IR code %04X - ir_preset0", ir_value ) ;
      reply = analyzeCmd ("preset=0") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_stop )
    {
      dbgprint ( "IR code %04X - ir_stop", ir_value ) ;
      reply = analyzeCmd ("stop") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_play )
    {
      dbgprint ( "IR code %04X - ir_play", ir_value ) ;
      reply = analyzeCmd ("resume") ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_volup )
    {
      dbgprint ( "IR code %04X - ir_volup", ir_value ) ;
      reply = analyzeCmd ( "upvolume=5" ) ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_voldown )
    {
      dbgprint ( "IR code %04X - ir_voldown", ir_value ) ;
      reply = analyzeCmd ( "downvolume=5" ) ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_mute )
    {
      dbgprint ( "IR code %04X - ir_mute", ir_value ) ;
      reply = analyzeCmd ( "mute" ) ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_next )
    {
      dbgprint ( "IR code %04X - ir_next", ir_value ) ;
      reply = analyzeCmd ( "uppreset=1" ) ;
      dbgprint ( reply ) ;
    }
    else if ( ir_value == ir_prev )
    {
      dbgprint ( "IR code %04X - ir_prev", ir_value ) ;
      reply = analyzeCmd ( "downpreset=1" ) ;               // Analyze command and handle it
      dbgprint ( reply ) ;                                  // Result for debugging
    }
    else
    {
      dbgprint ( "IR code %04X received, but not found in the configuration! Timing %d/%d",
                 ir_value, ir_0, ir_1 ) ;
    }
    ir_value = 0 ;                                          // Reset IR code received
  }
}


//**************************************************************************************************
//                                   S E T U P I R                                                 *
//**************************************************************************************************
// Setup IR input.                                                                                 *
//**************************************************************************************************
void setupIR ( uint8_t ir_pin )
{
  dbgprint ( "Enable GPIO %d for IR", ir_pin ) ;
  pinMode ( ir_pin, INPUT ) ;                        // Pin for IR receiver TSOP4838
  attachInterrupt ( ir_pin, isr_IR, CHANGE ) ;       // Interrupts will be handled by isr_IR
}
