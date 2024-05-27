//
#define VERSION "Wed, 30 Apr 2023 21:30:00 GMT"
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
// Maximum number of MQTT reconnects before give-up
#define MAXMQTTCONNECTS     20
//
// Use 23LC1024 SPI RAM as ringbuffer
#define SRAM
//
// Ringbuffer for smooth playing. 20000 bytes is 160 Kbits, about 1.5 seconds at 128kb bitrate.
#define RINGBFSIZ         20000
//
// Define LCD if you are using LCD 2004
#define LCD
//
// Enable support for Infra-red receiver by uncommenting the next line
#define IR
//
// Set IR pin to GPIO02
#define IR_PIN        2
//
// Digital I/O used
// Pins for VS1053 module
#define VS1053_CS     0
#define VS1053_DCS    16
#define VS1053_DREQ   10
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
