//
#define VERSION "Sun, 1 Sep 2024 22:10:00 GMT"
//
// Access point name if connection to WiFi network fails.  Also the hostname for WiFi and OTA.
// Not that the password of an AP must be at least as long as 8 characters.
// Also used for other naming.
#define NAME "Esp-radio"
//
// Debug buffer size
#define DEBUG_BUFFER_SIZE   100
//
// Name of the ini file
#define INIFILENAME "/radio.ini"
//
// Add a fixed SSID to the list (WiFi only) e.g LAPTOP-99/wifiwifi
//#define FIXEDWIFI ""
//
// Ringbuffer for smooth playing. 20000 bytes is 160 Kbits, about 1.5 seconds at 128kb bitrate.
#define RINGBFSIZ         16000
//
// Maximum number of MQTT reconnects before give-up
#define MAXMQTTCONNECTS     20
//
// Use 23LC1024 SPI RAM as ringbuffer
#define SRAM
#if defined ( SRAM )
  #define SRAM_CS       15            // SPIRAM CS pin is connected to GPIO 15
  #define SRAM_FREQ     20000000      // 23LC1024 supports theorically up to 20MHz
  #define SRAM_SIZE     131072        // Total size SPI RAM in bytes
  #define CHUNKSIZE     32            // Chunk size
  #define SRAM_CH_SIZE  4096          // Total size SPI RAM in chunks
  #define SPIRAMDELAY   100000        // Delay (in bytes) before reading from SPIRAM
  #include "SPIRAM.hpp"
#endif
//
// Define LCD if you are using LCD 2004
#define LCD
#if defined ( LCD )
  #include "LCD2004.hpp"
#else
// Empty declaration
  #define displayinfo(a,b)
  #define displaytime(a)
#endif
//
// Enable support for Infra-red receiver by uncommenting the next line
#define IR
#if defined ( IR )
  #include "IR.hpp"
// Set IR pin to GPIO02
  #define IR_PIN      2
// IR codes
  uint16_t ir_preset1 = 0xA25D ;
  uint16_t ir_preset2 = 0x629D ;
  uint16_t ir_preset3 = 0xE21D ;
  uint16_t ir_preset4 = 0x22DD ;
  uint16_t ir_preset5 = 0x02FD ;
  uint16_t ir_preset6 = 0xC23D ;
  uint16_t ir_preset7 = 0xE01F ;
  uint16_t ir_preset8 = 0xA857 ;
  uint16_t ir_preset9 = 0x906F ;
  uint16_t ir_preset0 = 0x9867 ;
  uint16_t ir_stop    = 0xB04F ; // #
  uint16_t ir_play    = 0x6897 ; // *
  uint16_t ir_volup   = 0x18E7 ;
  uint16_t ir_voldown = 0x4AB5 ;
  uint16_t ir_mute    = 0x38C7 ;
  uint16_t ir_next    = 0x5AA5 ;
  uint16_t ir_prev    = 0x10EF ;
#endif
//
// Digital I/O pins used for VS1053 module
#define VS1053_CS     0
#define VS1053_DCS    16
#define VS1053_DREQ   9
#define VS1053_RST    10
#include "VS1053.hpp"
//
// Definitions for 3 control switches on analog input
// You can test the analog input values by holding down the switch and select /?analog=1
// in the web interface. See schematics in the documentation.
// Switches are programmed as "Goto station 1", "Next station" and "Previous station" respectively.
// Set these values to 2000 if not used or tie analog input to ground.
#define NUMANA  3
#define asw1    2000
#define asw2    2000
#define asw3    2000
