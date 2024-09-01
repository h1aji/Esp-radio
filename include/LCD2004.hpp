//***************************************************************************************************
//   LCD2004.h - Driver for LCD 2004 display with I2C backpack.                                     *
//***************************************************************************************************
// The backpack communicates with the I2C bus and converts the serial data to parallel for the      *
// 2004 board.                                                                                      *
// Do not forget the PULL-UP resistors (4.7k on both SDA and CLK).                                  *
// In the serial data, the 8 bits are assigned as follows:                                          *
// Bit   Destination  Description                                                                   *
// ---   -----------  ------------------------------------                                          *
//  0    RS           H=data, L=command                                                             *
//  1    RW           H=read, L=write.  Only write is used.                                         *
//  2    E            Enable                                                                        *
//  3    BL           Backlight, H=on, L=off.  Always on.                                           *
//  4    D4           Data bit 4                                                                    *
//  5    D5           Data bit 5                                                                    *
//  6    D6           Data bit 6                                                                    *
//  7    D7           Data bit 7                                                                    *
//***************************************************************************************************
//
// Note that the display functions are limited due to the minimal available space.

#include <Wire.h>

#define ACKENA true                                         // Enable ACK for I2C communication

#define DELAY_ENABLE_PULSE_SETTLE           50              // Command requires > 37us to settle
#define FLAG_BACKLIGHT_ON                   0b00001000      // Bit 3, backlight enabled (disabled if clear)
#define FLAG_ENABLE                         0b00000100      // Bit 2, Enable
#define FLAG_RS_DATA                        0b00000001      // Bit 0, RS=data (command if clear)
#define FLAG_RS_COMMAND                     0b00000000      // Command

#define COMMAND_CLEAR_DISPLAY               0x01
#define COMMAND_RETURN_HOME                 0x02
#define COMMAND_ENTRY_MODE_SET              0x04
#define COMMAND_DISPLAY_CONTROL             0x08
#define COMMAND_FUNCTION_SET                0x20
#define COMMAND_SET_DDRAM_ADDR              0x80

#define FLAG_DISPLAY_CONTROL_DISPLAY_ON     0x04
#define FLAG_DISPLAY_CONTROL_CURSOR_ON      0x02

#define FLAG_FUNCTION_SET_MODE_4BIT         0x00
#define FLAG_FUNCTION_SET_LINES_2           0x08
#define FLAG_FUNCTION_SET_DOTS_5X8          0x00

#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT 0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON  0x01

#define dsp_print(a)                                        // Print a string
#define dsp_setCursor(a,b)                                  // Position the cursor
#define dsp_getwidth()                      20              // Get width of screen
#define dsp_getheight()                     4               // Get height of screen

void displayvolume  ( uint8_t vol ) ;
void displaytime    ( const char* str ) ;
void displayinfo    ( const char *str, int pos ) ;
bool dsp_begin      ();
void dsp_update     ();

char*       dbgprint ( const char* format, ... ) ;          // Print a formatted debug line
extern      struct tm timeinfo ;                            // Will be filled by NTP server
void        utf8ascii_ip ( char* s ) ;

class LCD2004
{
  public:
                     LCD2004 ( int8_t sda, int8_t scl ) ;   // Constructor
    void             print ( char c ) ;                     // Send 1 char
    void             reset() ;                              // Perform reset
    void             sclear() ;                             // Clear the screen
    void             shome() ;                              // Go to home position
    void             scursor ( uint8_t col, uint8_t row ) ; // Position the cursor
    void             scroll ( bool son ) ;                  // Set scroll on/off
  private:
    void             scommand ( uint8_t cmd ) ;
    void             strobe ( uint8_t cmd ) ;
    void             swrite ( uint8_t val, uint8_t rs ) ;
    void             write_cmd ( uint8_t val ) ;
    void             write_data ( uint8_t val ) ;
    uint8_t          bl  = FLAG_BACKLIGHT_ON ;              // Backlight in every command
    uint8_t          xchar = 0 ;                            // Current cursor position (text)
    uint8_t          ychar = 0 ;                            // Current cursor position (text)
    byte             LCD_I2C_ADDRESS;
} ;


LCD2004* lcd = NULL ;


bool dsp_begin()
{
  dbgprint ( "Init I2C LCD2004: SDA GPIO %d, SCL GPIO %d",
                                PIN_WIRE_SDA, PIN_WIRE_SCL ) ;
  if ( ( PIN_WIRE_SDA == 4 ) && ( PIN_WIRE_SCL == 5 ) )       // Make sure correct pins are used
  {
    lcd = new LCD2004 ( PIN_WIRE_SDA, PIN_WIRE_SCL ) ;        // Create an instance for LCD
  }
  else
  {
    dbgprint ( "Init I2C LCD2004 failed!" ) ;
  }
  return ( lcd != NULL ) ;
}


//***********************************************************************************************
//                                L C D 2 0 0 4  write functions                                *
//***********************************************************************************************
// Write functins for command, data and general.                                                *
//***********************************************************************************************
void LCD2004::swrite ( uint8_t val, uint8_t rs )            // General write, 8 bits data
{
  strobe ( ( val & 0xf0 ) | rs ) ;                          // Send 4 LSB bits
  strobe ( ( val << 4 ) | rs ) ;                            // Send 4 MSB bits
}


void LCD2004::write_data ( uint8_t val )
{
  swrite ( val, FLAG_RS_DATA ) ;                            // Send data (RS = HIGH)
}


void LCD2004::write_cmd ( uint8_t val )
{
  swrite ( val, FLAG_RS_COMMAND ) ;                         // Send command (RS = LOW)
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S T R O B E                                  *
//***********************************************************************************************
// Send data followed by strobe to clock data to LCD.                                           *
//***********************************************************************************************
void LCD2004::strobe ( uint8_t cmd )
{
  scommand ( cmd | FLAG_ENABLE ) ;                          // Send command with E high
  scommand ( cmd ) ;                                        // Same command with E low
  delayMicroseconds ( DELAY_ENABLE_PULSE_SETTLE ) ;         // Wait a short time
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S C O M M A N D                              *
//***********************************************************************************************
// Send a command to the LCD.                                                                   *
// Actual I/O.  Open a channel to the I2C interface and write one byte.                         *
//***********************************************************************************************
void LCD2004::scommand ( uint8_t cmd )
{
  Wire.beginTransmission ( LCD_I2C_ADDRESS ) ;
  Wire.write ( cmd | FLAG_BACKLIGHT_ON ) ;
  Wire.endTransmission() ;
}

//***********************************************************************************************
//                                L C D 2 0 0 4 :: P R I N T                                    *
//***********************************************************************************************
// Put a character in the buffer.                                                               *
//***********************************************************************************************
void LCD2004::print ( char c )
{
  write_data ( c ) ;
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S C U R S O R                                *
//***********************************************************************************************
// Place the cursor at the requested position.                                                  *
//***********************************************************************************************
void LCD2004::scursor ( uint8_t col, uint8_t row )
{
  const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 } ;
  
  write_cmd ( COMMAND_SET_DDRAM_ADDR |
              ( col + row_offsets[row] ) ) ; 
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S C L E A R                                  *
//***********************************************************************************************
// Clear the LCD.                                                                               *
//***********************************************************************************************
void LCD2004::sclear()
{
  write_cmd ( COMMAND_CLEAR_DISPLAY ) ;
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S C R O L L                                  *
//***********************************************************************************************
// Set scrolling on/off.                                                                        *
//***********************************************************************************************
void LCD2004::scroll ( bool son )
{
  uint8_t ecmd = COMMAND_ENTRY_MODE_SET |                   // Assume no scroll
                 FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT ;

  if ( son )                                                // Scroll on?
  {
    ecmd |= FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON ;            // Yes, change function
  }
  write_cmd ( ecmd ) ;                                      // Perform command
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: S H O M E                                    *
//***********************************************************************************************
// Go to home position.                                                                         *
//***********************************************************************************************
void LCD2004::shome()
{
  write_cmd ( COMMAND_RETURN_HOME ) ;
}


//***********************************************************************************************
//                                L C D 2 0 0 4 :: R E S E T                                    *
//***********************************************************************************************
// Reset the LCD.                                                                               *
//***********************************************************************************************
void LCD2004::reset()
{
  scommand ( 0 ) ;                                          // Put expander to known state
  delayMicroseconds ( 1000 ) ;
  for ( int i = 0 ; i < 3 ; i++ )                           // Repeat 3 times
  {
    strobe ( 0x03 << 4 ) ;                                  // Select 4-bit mode
    delayMicroseconds ( 4500 ) ;
  }
  strobe ( 0x02 << 4 ) ;                                    // 4-bit
  delayMicroseconds ( 4500 ) ;
  write_cmd ( COMMAND_FUNCTION_SET |
              FLAG_FUNCTION_SET_MODE_4BIT |
              FLAG_FUNCTION_SET_LINES_2 |
              FLAG_FUNCTION_SET_DOTS_5X8 ) ;
  write_cmd ( COMMAND_DISPLAY_CONTROL |
              FLAG_DISPLAY_CONTROL_DISPLAY_ON ) ;
  sclear() ;
  write_cmd ( COMMAND_ENTRY_MODE_SET |
              FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT ) ;
  shome() ;
}


//**************************************************************************************************
//                                          I 2 C S C A N                                          *
//**************************************************************************************************
// Utility to scan the I2C bus.                                                                    *
//**************************************************************************************************
byte i2cscan()
{
  byte error, address ;

  dbgprint ( "Scanning I2C bus..." ) ;

  for ( address = 1 ; address < 127 ; address++ )
  {
    Wire.beginTransmission ( address ) ;
    error = Wire.endTransmission() ;
    if ( error == 0 )
    {
      dbgprint ( "I2C device 0x%02X found", address ) ;
      return address;                                   // Return the address of the found device
    }
    else if ( error == 4 )
    {
      dbgprint ( "Error 4 at address 0x%02X", address ) ;
    }
  }
  return 0;                                             // Return 0 if no device is found
}


//***********************************************************************************************
//                                L C D 2 0 0 4                                                 *
//***********************************************************************************************
// Constructor for the display.                                                                 *
//***********************************************************************************************
LCD2004::LCD2004 ( int8_t sda, int8_t scl )
{
  Wire.begin ( sda, scl ) ;
  delay ( 50 ) ;

  LCD_I2C_ADDRESS = i2cscan();                          // Assign the found address to LCD_I2C_ADDRESS

  if ( LCD_I2C_ADDRESS == 0 )
  {
    dbgprint ( "No I2C devices found." ) ;
    return;                                             // Exit if no device is found
  }

  Wire.beginTransmission ( LCD_I2C_ADDRESS ) ;
  if ( Wire.endTransmission() != 0 )
  {
    dbgprint ( "Display not found at I2C 0x%02X address",
                  LCD_I2C_ADDRESS ) ;                   // Safety check, make sure the PCF8574 is connected
  }
  reset() ;
}


struct dsp_str
{
  String          str ;
  uint16_t        len ;                                 // Length of string to show
  uint16_t        pos ;                                 // Start on this position of string
  uint8_t         row ;                                 // Row on display  
} ;

dsp_str dline[4] = { { "", 0, 0, 0 },
                     { "", 0, 0, 0 },
                     { "", 0, 0, 0 },
                     { "", 0, 0, 0 }
                   } ;


//***********************************************************************************************
//                                D S P _U P D A T E _ L I N E                                  *
//***********************************************************************************************
// Show a selected line                                                                         *
//***********************************************************************************************
void dsp_update_line ( uint8_t lnr )
{
  uint8_t         i ;                                   // Index in string
  const char*     p ;

  p = dline[lnr].str.c_str() ;
  dline[lnr].len = strlen ( p ) ;
  //dbgprint ( "Strlen is %d, str is %s", len, p ) ;
  if ( dline[lnr].len > dsp_getwidth() )
  {
    if ( dline[lnr].pos >= dline[lnr].len )
    {
      dline[lnr].pos = 0 ;
    }
    else
    {
      p += dline[lnr].pos ;
    }
    dline[lnr].len -= dline[lnr].pos ;
    if ( dline[lnr].len > dsp_getwidth() )
    {
      dline[lnr].len = dsp_getwidth() ;
    }
  }
  else
  {
    dline[lnr].pos = 0 ;                             // String fits on screen
  }
  dline[lnr].pos++ ;
  lcd->scursor ( 0, lnr ) ;
  for ( i = 0 ; i < dline[lnr].len ; i++ )
  {
    if ( ( *p >= ' ' ) && ( *p <= '~' ) )            // Printable?
    {
      lcd->print ( *p ) ;                            // Yes
    }
    else
    {
      lcd->print ( ' ' ) ;                           // Yes, print space
    }
    p++ ;
  }
  for ( i = 0 ; i < ( dsp_getwidth() - dline[lnr].len ) ; i++ )  // Fill remainder
  {
    lcd->print ( ' ' ) ;
  }
  if ( *p == '\0' )                                  // At end of line?
  {
    dline[lnr].pos = 0 ;                             // Yes, start allover
  }
}


//***********************************************************************************************
//                                D S P _U P D A T E                                            *
//***********************************************************************************************
// Show a selection of the 4 sections                                                           *
//***********************************************************************************************
void dsp_update()
{
  static uint16_t cnt = 0 ;                             // Reduce updates

  if ( cnt++ != 8 )                                     // Action every 8 calls
  {
    return ;
  }
  cnt = 0 ;
  dline[2].str.trim() ;                                 // Remove non printing
  dline[1].str.trim() ;                                 // Remove non printing
  if ( dline[2].str.length() > dsp_getwidth() )
  {
    dline[2].str += String ( "  " ) ;
  }
  if ( dline[1].str.length() > dsp_getwidth() )
  {
    dline[1].str += String ( "  " ) ;
  }
  dsp_update_line ( 1 ) ;
  dsp_update_line ( 2 ) ;
}


//**************************************************************************************************
//                                      D I S P L A Y V O L U M E                                  *
//**************************************************************************************************
// Display volume for this type of display.                                                        *
// line 3 will be used.                                                                            *
//**************************************************************************************************
void displayvolume ( uint8_t vol )
{
  static uint8_t   oldvol = 0 ;                       // Previous volume
  uint16_t         pos ;                              // Positon of volume indicator

  dline[3].str = "";

  if ( vol != oldvol )                                // Volume changed?
  {
    // dbgprint ( "Update volume to %d", vol ) ;
    oldvol = vol ;                                    // Remember for next compare
    pos = map ( vol, 0, 100, 0, dsp_getwidth() ) ;    // Compute end position on TFT
    for ( int i = 0 ; i < dsp_getwidth() ; i++ )      // Set oldstr to dots
    {
      if ( i <= pos )
      {
        dline[3].str += "#" ;                         // Add hash character
      }
      else
      {
        dline[3].str += " " ;                         // Or blank sign
      }
    }
    dsp_update_line(3) ;
  }
}


//**************************************************************************************************
//                                      D I S P L A Y T I M E                                      *
//**************************************************************************************************
// Display date and time to LCD line 0.                                                            *
//**************************************************************************************************
void displaytime ( const char* str )
{
  const char* WDAYS [] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" } ;
  char        datetxt[28] ;
  char        timetxt[6] ;                               // Temporary string for hours and minutes
  static char oldstr = '\0' ;                            // To check time difference

  if ( ( str == NULL ) || ( str[0] == '\0' ) )           // Check time string
  {
    return ;                                             // Not okay, return
  }
  else
  {
    if ( str[4] == oldstr )                              // Difference?
    {
      return ;                                           // No, quick return
    }
    strncpy(timetxt, str, 5);                            // Copy first 5 characters (HH:MM)
    timetxt[5] = '\0';                                   // Null-terminate the string
    sprintf ( datetxt, "%s  %02d.%02d.%02d  %s",         // Format new time to a string
                       WDAYS[timeinfo.tm_wday],
                       timeinfo.tm_mday,
                       timeinfo.tm_mon + 1,
                       (timeinfo.tm_year + 1900) % 100,  // last 2 digits of the year
                       timetxt ) ;
  }
  dline[0].str = String ( datetxt ) ;                    // Copy datestring or empty string to LCD line 0
  oldstr = str[4] ;                                      // For next compare, last digit of time
  dsp_update_line ( 0 ) ;
}


//******************************************************************************************
//                              D I S P L A Y I N F O                                      *
//******************************************************************************************
// Show a string on the LCD at a specified y-position in a specified color                 *
//******************************************************************************************
void displayinfo ( const char *str, int pos )
{
  char buf [ strlen ( str ) + 1 ] ;             // Need some buffer space
  strcpy ( buf, str ) ;                         // Make a local copy of the string
  utf8ascii_ip ( buf ) ;                        // Convert possible UTF8
  dline[pos].str = buf ;                        // Write to buffer
  dsp_update_line ( pos ) ;                     // Show on display
}
