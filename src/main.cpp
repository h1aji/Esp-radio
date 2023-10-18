//************************************************************************************************
//* Esp-radio - Webradio receiver for ESP8266, LCD2004 monochrome display and VS1053 MP3 module, *
//*              by Ed Smallenburg (ed@smallenburg.nl)                                           *
//************************************************************************************************
//
// Define the version number, also used for webserver as Last-Modified header:
//
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <stdio.h>
#include <string.h>
#include <Ticker.h>
#include <time.h>
//
#include <AsyncMqttClient.h>
#include <ESPAsyncWebServer.h>
#include <TinyXML.h>
//
extern "C"
{
  #include <user_interface.h>
}
//
// Specify configuration
#include "config.hpp" 
//
#include "VS1053.hpp"
//
#if defined ( SRAM )
  #include "SPIRAM.hpp"
#endif
//
#if defined ( LCD )
  #include "LCD2004.hpp"
#else
// Empty declaration
  #define displayinfo(a,b)
  #define displaytime(a)
#endif
//
#if defined ( IR )
  #include "IR.hpp"
#endif
//
//******************************************************************************************
// Forward declaration of various functions                                                *
//******************************************************************************************
void   showstreamtitle ( const char* ml, bool full = false ) ;
void   handlebyte ( uint8_t b, bool force = false ) ;
void   handlebyte_ch ( uint8_t b, bool force = false ) ;
void   handleFS ( AsyncWebServerRequest* request ) ;
void   handleFSf ( AsyncWebServerRequest* request, const String& filename ) ;
void   handleCmd ( AsyncWebServerRequest* request )  ;
void   handleFileUpload ( AsyncWebServerRequest* request, String filename,
                          size_t index, uint8_t* data, size_t len, bool final ) ;
char*  dbgprint( const char* format, ... ) ;
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
String chomp ( String str ) ;
void   publishIP() ;
String xmlparse ( String mount ) ;
void   XML_callback ( uint8_t statusflags, char* tagName, uint16_t tagNameLen,
                    char* data,  uint16_t dataLen ) ;
bool   connecttohost() ;
void   gettime() ;
char   utf8ascii ( char ascii ) ;                          // Convert UTF to Ascii
void   utf8ascii_ip ( char* s ) ;                          // Convert UTF to Ascii in place
String utf8ascii ( const char* s ) ;                       // Convert UTF to Ascii as String
void   scan_content_length ( const char* metalinebf ) ;

//
//******************************************************************************************
// Global data section.                                                                    *
//******************************************************************************************
// There is a block ini-data that contains some configuration.  Configuration data is      *
// saved in the LittleFS file radio.ini by the webinterface.  On restart the new data will *
// be read from this file.                                                                 *
// Items in ini_block can be changed by commands from webserver/MQTT/Serial.               *
//******************************************************************************************
struct ini_struct
{
  String         mqttbroker ;                              // The name of the MQTT broker server
  uint16_t       mqttport ;                                // Port, default 1883
  String         mqttuser ;                                // User for MQTT authentication
  String         mqttpasswd ;                              // Password for MQTT authentication
  String         mqtttopic ;                               // Topic to suscribe to
  String         mqttpubtopic ;                            // Topic to pubtop (IP will be published)
  uint8_t        reqvol ;                                  // Requested volume
  uint8_t        rtone[4] ;                                // Requested bass/treble settings
  int8_t         newpreset ;                               // Requested preset
  String         clk_server ;                              // Server to be used for time of day clock
  int8_t         clk_offset ;                              // Offset in hours with respect to UTC
  int8_t         clk_dst ;                                 // Number of hours shift during DST
  String         ssid ;                                    // SSID of WiFi network to connect to
  String         passwd ;                                  // Password for WiFi network
} ;

enum datamode_t { INIT = 1, HEADER = 2, DATA = 4,
                  METADATA = 8, PLAYLISTINIT = 16,
                  PLAYLISTHEADER = 32, PLAYLISTDATA = 64,
                  STOPREQD = 128, STOPPED = 256
                } ;        // State for datastream

// Global variables
int              DEBUG = 1 ;
ini_struct       ini_block ;                               // Holds configurable data
WiFiClient       *mp3client = NULL ;                       // An instance of the mp3 client
AsyncWebServer   cmdserver ( 80 ) ;                        // Instance of embedded webserver on port 80
AsyncMqttClient  mqttclient ;                              // Client for MQTT subscriber
IPAddress        mqtt_server_IP ;                          // IP address of MQTT broker
char             cmd[130] ;                                // Command from MQTT or Serial
Ticker           tckr ;                                    // For timing 100 msec
TinyXML          xml ;                                     // For XML parser.
uint32_t         totalcount = 0 ;                          // Counter mp3 data
datamode_t       datamode ;                                // State of datastream
int              metacount ;                               // Number of bytes in metadata
int              datacount ;                               // Counter databytes before metadata
String           metaline ;                                // Readable line in metadata
String           icystreamtitle ;                          // Streamtitle from metadata
String           icyname ;                                 // Icecast station name
int              bitrate ;                                 // Bitrate in kb/sec
int              mbitrate ;                                // Measured bitrate
int              metaint = 0 ;                             // Number of databytes between metadata
int8_t           currentpreset = -1 ;                      // Preset station playing
String           host ;                                    // The URL to connect to or file to play
String           playlist ;                                // The URL of the specified playlist
bool             xmlreq = false ;                          // Request for XML parse.
bool             hostreq = false ;                         // Request for new host
bool             reqtone = false ;                         // New tone setting requested
bool             muteflag = false ;                        // Mute output
uint16_t         analogsw[NUMANA] = { asw1, asw2, asw3 } ; // 3 levels of analog input
uint16_t         analogrest ;                              // Rest value of analog input
bool             resetreq = false ;                        // Request to reset the ESP8266
bool             NetworkFound ;                            // True if WiFi network connected
String           networks ;                                // Found networks
String           anetworks ;                               // Aceptable networks (present in .ini file)
String           presetlist ;                              // List for webserver
uint8_t          num_an ;                                  // Number of acceptable networks in .ini file
String           testfilename = "" ;                       // File to test (LittleFS speed)
uint16_t         mqttcount = 0 ;                           // Counter MAXMQTTCONNECTS
int8_t           playlist_num = 0 ;                        // Nonzero for selection from playlist
File             mp3file ;                                 // File containing mp3 on LittleFS
bool             localfile = false ;                       // Play from local mp3-file or not
bool             chunked = false ;                         // Station provides chunked transfer
int              chunkcount = 0 ;                          // Counter for chunked transfer
uint16_t         rcount = 0 ;                              // Number of bytes/chunks in ringbuffer/SPIRAM
#if not defined ( SRAM )
  uint8_t       *ringbuf ;                                 // Ringbuffer for VS1053
  uint16_t       rbwindex = 0 ;                            // Fill pointer in ringbuffer
  uint16_t       rbrindex = RINGBFSIZ - 1 ;                // Emptypointer in ringbuffer
#endif
bool             scrollflag = false ;                      // Request to scroll LCD
struct tm        timeinfo ;                                // Will be filled by NTP server
char             timetxt[9] ;                              // Converted timeinfo
bool             time_req = false ;                        // Set time requested
uint32_t         clength ;                                 // Content length found in http header
const char*      fixedwifi = "" ;                          // Used for FIXEDWIFI option


// XML parse globals.
const char* xmlhost = "playerservices.streamtheworld.com" ;// XML data source
const char* xmlget =  "GET /api/livestream"                // XML get parameters
                      "?version=1.5"                       // API Version of IHeartRadio
                      "&mount=%sAAC"                       // MountPoint with Station Callsign
                      "&lang=en" ;                         // Language
int         xmlport = 80 ;                                 // XML Port
uint8_t     xmlbuffer[150] ;                               // For XML decoding
String      xmlOpen ;                                      // Opening XML tag
String      xmlTag ;                                       // Current XML tag
String      xmlData ;                                      // Data inside tag
String      stationServer( "" ) ;                          // Radio stream server
String      stationPort( "" ) ;                            // Radio stream port
String      stationMount( "" ) ;                           // Radio stream Callsign

//******************************************************************************************
// End of global data section.                                                             *
//******************************************************************************************


// The object for the MP3 player
VS1053 vs1053player ( VS1053_CS, VS1053_DCS, VS1053_DREQ, VS1053_RST ) ;


//******************************************************************************************
// Pages and CSS for the webinterface.                                                     *
//******************************************************************************************
#include "html/about_html.h"
#include "html/config_html.h"
#include "html/index_html.h"
#include "html/radio_css.h"
#include "html/favicon_ico.h"


//**************************************************************************************************
//                                      U T F 8 A S C I I                                          *
//**************************************************************************************************
// UTF8-Decoder: convert UTF8-string to extended ASCII.                                            *
// Convert a single Character from UTF8 to Extended ASCII.                                         *
// Return "0" if a byte has to be ignored.                                                         *
//**************************************************************************************************
char utf8ascii ( char ascii )
{
  static const char lut_C3[] = { "AAAAAAACEEEEIIIIDNOOOOO#0UUUU###"
                                 "aaaaaaaceeeeiiiidnooooo##uuuuyyy" } ; 
  static const char lut_C4[] = { "AaAaAaCcCcCcCcDdDdEeEeEeEeEeGgGg"
                                 "GgGgHhHhIiIiIiIiIiJjJjKkkLlLlLlL" } ;
  static const char lut_C5[] = { "lLlNnNnNnnnnOoOoOoOoRrRrRrSsSsSs"
                                 "SsTtTtTtUuUuUuUuUuUuWwYyYZzZzZzs" } ;

  static char       c1 ;              // Last character buffer
  char              res = '\0' ;      // Result, default 0

  if ( ascii <= 0x7F )                // Standard ASCII-set 0..0x7F handling
  {
    c1 = 0 ;
    res = ascii ;                     // Return unmodified
  }
  else
  {
    switch ( c1 )                     // Conversion depending on first UTF8-character
    {
      case 0xC2: res = '~' ;
        break ;
      case 0xC3: res = lut_C3[ascii - 128] ;
        break ;
      case 0xC4: res = lut_C4[ascii - 128] ;
        break ;
      case 0xC5: res = lut_C5[ascii - 128] ;
        break ;
      case 0x82: if ( ascii == 0xAC )
        {
          res = 'E' ;                 // Special case Euro-symbol
        }
    }
    c1 = ascii ;                      // Remember actual character
  }
  return res ;                        // Otherwise: return zero, if character has to be ignored
}


//**************************************************************************************************
//                                U T F 8 A S C I I _ I P                                          *
//**************************************************************************************************
// In Place conversion UTF8-string to Extended ASCII (ASCII is shorter!).                          *
//**************************************************************************************************
void utf8ascii_ip ( char* s )
{
  int  i, k = 0 ;                     // Indexes for in en out string
  char c ;

  for ( i = 0 ; s[i] ; i++ )          // For every input character
  {
    c = utf8ascii ( s[i] ) ;          // Translate if necessary
    if ( c )                          // Good translation?
    {
      s[k++] = c ;                    // Yes, put in output string
    }
  }
  s[k] = 0 ;                          // Take care of delimeter
}


//**************************************************************************************************
//                                      U T F 8 A S C I I                                          *
//**************************************************************************************************
// Conversion UTF8-String to Extended ASCII String.                                                *
//**************************************************************************************************
String utf8ascii ( const char* s )
{
  int  i ;                            // Index for input string
  char c ;
  String res = "" ;                   // Result string

  for ( i = 0 ; s[i] ; i++ )          // For every input character
  {
    c = utf8ascii ( s[i] ) ;          // Translate if necessary
    if ( c )                          // Good translation?
    {
      res += String ( c ) ;           // Yes, put in output string
    }
  }
  return res ;
}


//**************************************************************************************************
//                                         G E T T I M E                                           *
//**************************************************************************************************
// Retrieve the local time from NTP server and convert to string.                                  *
// Will be called every second.                                                                    *
//**************************************************************************************************
void gettime()
{
  static int16_t delaycount = 0 ;                           // To reduce number of NTP requests
  static int16_t retrycount = 100 ;
  {
    if ( timeinfo.tm_year )                                 // Legal time found?
    {
      sprintf ( timetxt, "%02d:%02d:%02d",                  // Yes, format to a string
                timeinfo.tm_hour,
                timeinfo.tm_min,
                timeinfo.tm_sec ) ;
    }
    if ( --delaycount <= 0 )                                // Sync every few hours
    {
      delaycount = 7200 ;                                   // Reset counter
      if ( timeinfo.tm_year )                               // Legal time found?
      {
        dbgprint ( "Sync TOD, old value is %s", timetxt ) ;
      }
      else
      {
        dbgprint ( "Sync TOD" ) ;
      }
      if ( !getLocalTime ( &timeinfo ) )                    // Read from NTP server
      {
        dbgprint ( "Failed to obtain time!" ) ;             // Error
        timeinfo.tm_year = 0 ;                              // Set current time to illegal
        if ( retrycount )                                   // Give up syncing?
        {
          retrycount-- ;                                    // No try again
          delaycount = 5 ;                                  // Retry after 5 seconds
        }
      }
      else
      {
        sprintf ( timetxt, "%02d:%02d:%02d",                // Format new time to a string
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec ) ;
        dbgprint ( "Sync TOD, new value is %s", timetxt ) ;
      }
    }
  }
}


//******************************************************************************************
// Ringbuffer (fifo) routines.                                                             *
//******************************************************************************************
//******************************************************************************************
//                              R I N G S P A C E                                          *
//******************************************************************************************
inline bool ringspace()
{
#if defined ( SRAM )
  return spiram.spaceAvailable() ;         // True if at least one chunk is available
#else
  return ( rcount < RINGBFSIZ ) ;          // True if at least one byte of free space is available
#endif
}


//******************************************************************************************
//                              R I N G A V A I L                                          *
//******************************************************************************************
inline uint16_t ringavail()
{
#if defined ( SRAM )
  return spiram.dataAvailable() ;          // Return number of chunks filled
#else
  return rcount ;                          // Return number of bytes available
#endif
}


//******************************************************************************************
//                                P U T R I N G                                            *
//******************************************************************************************
// No check on available space.  See ringspace()                                           *
//******************************************************************************************
void putring ( uint8_t b )                   // Put one byte in the ringbuffer
{
#if defined ( SRAM )
  static uint8_t pwchunk[32] ;               // Buffer for one chunk
  pwchunk[spiram.prcwinx++] = b ;            // Store in local chunk
  if ( spiram.prcwinx == sizeof(pwchunk) )   // Chunk full?
  {
    spiram.bufferWrite ( pwchunk ) ;         // Yes, store in SPI RAM
    spiram.prcwinx = 0 ;                     // Index to begin of chunk
  }
#else
  *(ringbuf + rbwindex) = b ;                // Put byte in ringbuffer
  if ( ++rbwindex == RINGBFSIZ )             // Increment pointer and
  {
    rbwindex = 0 ;                           // wrap at the end
  }
  rcount++ ;                                 // Count number of bytes in the
#endif
}


//******************************************************************************************
//                                G E T R I N G                                            *
//******************************************************************************************
// Assume there is always something in the bufferpace.  See ringavail().                   *
//******************************************************************************************
uint8_t getring()
{
#if defined ( SRAM )
  static uint8_t prchunk[32] ;                 // Buffer for one chunk
  if ( spiram.prcrinx >= sizeof(prchunk) )     // End of buffer reached?
  {
    spiram.prcrinx = 0 ;                       // Yes, reset index to begin of buffer
    spiram.bufferRead ( prchunk ) ;            // And read new buffer
  }
  return ( prchunk[spiram.prcrinx++] ) ;
#else
  if ( ++rbrindex == RINGBFSIZ )               // Increment pointer and
  {
    rbrindex = 0 ;                             // wrap at the end
  }
  rcount-- ;                                   // Count is now one less
  return *(ringbuf + rbrindex) ;               // Return the oldest byte
#endif
}


//******************************************************************************************
//                               E M P T Y R I N G                                         *
//******************************************************************************************
void emptyring()
{
#if defined ( SRAM )
  spiram.prcwinx = 0 ;
  spiram.prcrinx = 32 ;                        // Set buffer to empty
#else
  rbwindex = 0 ;                               // Reset ringbuffer administration
  rbrindex = RINGBFSIZ - 1 ;
  rcount = 0 ;
#endif
}


//******************************************************************************************
//                                  D B G P R I N T                                        *
//******************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the BEDUg flag.*
// Print only if DEBUG flag is true.  Always returns the the formatted string.             *
//******************************************************************************************
char* dbgprint ( const char* format, ... )
{
  static char sbuf[DEBUG_BUFFER_SIZE] ;                // For debug lines
  va_list varArgs ;                                    // For variable number of params

  va_start ( varArgs, format ) ;                       // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;  // Format the message
  va_end ( varArgs ) ;                                 // End of using parameters
  if ( DEBUG )                                         // DEBUG on?
  {
    Serial.print ( "D: " ) ;                           // Yes, print prefix
    Serial.println ( sbuf ) ;                          // and the info
  }
  return sbuf ;                                        // Return stored string
}


//******************************************************************************************
//                             G E T E N C R Y P T I O N T Y P E                           *
//******************************************************************************************
// Read the encryption type of the network and return as a 4 byte name                     *
//*********************4********************************************************************
const char* getEncryptionType ( int thisType )
{
  switch (thisType)
  {
    case ENC_TYPE_WEP:
      return "WEP " ;
    case ENC_TYPE_TKIP:
      return "WPA " ;
    case ENC_TYPE_CCMP:
      return "WPA2" ;
    case ENC_TYPE_NONE:
      return "None" ;
    case ENC_TYPE_AUTO:
      return "Auto" ;
  }
  return "????" ;
}


//******************************************************************************************
//                                L I S T N E T W O R K S                                  *
//******************************************************************************************
// List the available networks and select the strongest.                                   *
// Acceptable networks are those who have an entry in "anetworks".                         *
// SSIDs of available networks will be saved for use in webinterface.                      *
//******************************************************************************************
void listNetworks()
{
  int         maxsig = -1000 ;   // Used for searching strongest WiFi signal
  int         newstrength ;
  byte        encryption ;       // TKIP(WPA)=2, WEP=5, CCMP(WPA)=4, NONE=7, AUTO=8
  const char* acceptable ;       // Netwerk is acceptable for connection
  int         i ;                // Loop control
  String      sassid ;           // Search string in anetworks

  ini_block.ssid = String ( "none" ) ;                   // No selceted network yet
  // scan for nearby networks:
  dbgprint ( "* Scan Networks *" ) ;
  int numSsid = WiFi.scanNetworks() ;
  if ( numSsid == -1 )
  {
    dbgprint ( "Couldn't get a wifi connection" ) ;
    return ;
  }
  // print the list of networks seen:
  dbgprint ( "Number of available networks: %d",
             numSsid ) ;
  // Print the network number and name for each network found and
  // find the strongest acceptable network
  for ( i = 0 ; i < numSsid ; i++ )
  {
    acceptable = "" ;                                    // Assume not acceptable
    newstrength = WiFi.RSSI ( i ) ;                      // Get the signal strenght
    sassid = WiFi.SSID ( i ) + String ( "|" ) ;          // For search string
    if ( anetworks.indexOf ( sassid ) >= 0 )             // Is this SSID acceptable?
    {
      acceptable = "Acceptable" ;
      if ( newstrength > maxsig )                        // This is a better Wifi
      {
        maxsig = newstrength ;
        ini_block.ssid = WiFi.SSID ( i ) ;               // Remember SSID name
      }
    }
    encryption = WiFi.encryptionType ( i ) ;
    dbgprint ( "%2d - %-25s Signal: %3d dBm Encryption %4s  %s",
               i + 1, WiFi.SSID ( i ).c_str(), WiFi.RSSI ( i ),
               getEncryptionType ( encryption ),
               acceptable ) ;
    // Remember this network for later use
    networks += WiFi.SSID ( i ) + String ( "|" ) ;
  }
  dbgprint ( "--------------------------------------" ) ;
}


//******************************************************************************************
//                                  T I M E R 1 0 S E C                                    *
//******************************************************************************************
// Extra watchdog.  Called every 10 seconds.                                               *
// If totalcount has not been changed, there is a problem and playing will stop.           *
// Note that a "yield()" within this routine or in called functions will cause a crash!    *
//******************************************************************************************
void IRAM_ATTR timer10sec()
{
  static uint32_t oldtotalcount = 7321 ;          // Needed for change detection
  static uint8_t  morethanonce = 0 ;              // Counter for succesive fails
  static uint8_t  t600 = 0 ;                      // Counter for 10 minutes
  uint32_t        bytesplayed ;                   // Bytes send to MP3 converter

  if ( datamode & ( INIT | HEADER | DATA |        // Test op playing
                    METADATA | PLAYLISTINIT |
                    PLAYLISTHEADER |
                    PLAYLISTDATA ) )
  {
    bytesplayed = totalcount - oldtotalcount ;    // Nunber of bytes played in the 10 seconds
    oldtotalcount = totalcount ;                  // Save for comparison in next cycle
    if ( bytesplayed == 0 )                       // Still playing?
    {
      dbgprint ( "No data input" ) ;              // No data detected!
      if ( morethanonce > 10 )                    // Happened too many times?
      {
        dbgprint ( "Going to restart..." ) ;
        ESP.restart() ;                           // Reset the CPU, probably no return
      }
      if ( datamode & ( PLAYLISTDATA |            // In playlist mode?
                        PLAYLISTINIT |
                        PLAYLISTHEADER ) )
      {
        playlist_num = 0 ;                        // Yes, end of playlist
      }
      if ( ( morethanonce > 0 ) ||                // Happened more than once?
           ( playlist_num > 0 ) )                 // Or playlist active?
      {
        datamode = STOPREQD ;                     // Stop player
        ini_block.newpreset++ ;                   // Yes, try next channel
        dbgprint ( "Trying other station/file..." ) ;
      }
      morethanonce++ ;                            // Count the fails
    }
    else
    {
      // Data has been send to MP3 decoder
      // Bitrate in kbits/s is bytesplayed / 10 / 1000 * 8
      mbitrate = ( bytesplayed + 625 ) / 1250 ;   // Measured bitrate
      morethanonce = 0 ;                          // Data seen, reset failcounter
      if ( morethanonce )                         // Recovered from data loss?
      {
        dbgprint ( "Recovered from dataloss" ) ;
        morethanonce = 0 ;                        // Data see, reset failcounter
      }
      oldtotalcount = totalcount ;                // Save for comparison in next cycle
    }
    if ( t600++ == 60 )                           // 10 minutes over?
    {
      t600 = 0 ;                                  // Yes, reset counter
      //dbgprint ( "10 minutes over" ) ;
      publishIP() ;                               // Re-publish IP
    }
  }
}


//******************************************************************************************
//                                  A N A G E T S W                                        *
//******************************************************************************************
// Translate analog input to switch number.  0 is inactive.                                *
// Note that it is adviced to avoid expressions as the argument for the abs function.      *
//******************************************************************************************
uint8_t anagetsw ( uint16_t v )
{
  int      i ;                                     // Loop control
  int      oldmindist = 1000 ;                     // Detection least difference
  int      newdist ;                               // New found difference
  uint8_t  sw = 0 ;                                // Number of switch detected (0 or 1..3)

  if ( v > analogrest )                            // Inactive level?
  {
    for ( i = 0 ; i < NUMANA ; i++ )
    {
      newdist = analogsw[i] - v ;                  // Compute difference
      newdist = abs ( newdist ) ;                  // Make it absolute
      if ( newdist < oldmindist )                  // New least difference?
      {
        oldmindist = newdist ;                     // Yes, remember
        sw = i + 1 ;                               // Remember switch
      }
    }
  }
  return sw ;                                      // Return active switch
}


//******************************************************************************************
//                               T E S T F I L E                                           *
//******************************************************************************************
// Test the performance of LittleFS read.                                                    *
//******************************************************************************************
void testfile ( String fspec )
{
  String   path ;                                      // Full file spec
  File     tfile ;                                     // File containing mp3
  uint32_t len, savlen ;                               // File length
  uint32_t t0, t1, told ;                              // For time test
  uint32_t t_error = 0 ;                               // Number of slow reads

  dbgprint ( "Start test of file %s", fspec.c_str() ) ;
  t0 = millis() ;                                      // Timestamp at start
  t1 = t0 ;                                            // Prevent uninitialized value
  told = t0 ;                                          // For report
  path = String ( "/" ) + fspec ;                      // Form full path
  tfile = LittleFS.open ( path, "r" ) ;                // Open the file
  if ( tfile )
  {
    len = tfile.available() ;                          // Get file length
    savlen = len ;                                     // Save for result print
    while ( len-- )                                    // Any data left?
    {
      t1 = millis() ;                                  // To meassure read time
      tfile.read() ;                                   // Read one byte
      if ( ( millis() - t1 ) > 5 )                     // Read took more than 5 msec?
      {
        t_error++ ;                                    // Yes, count slow reads
      }
      if ( ( len % 100 ) == 0 )                        // Yield reguarly
      {
        yield() ;
      }
      if ( ( ( t1 - told ) / 1000 ) > 0 || len == 0 )
      {
        // Show results for debug
        dbgprint ( "Read %s, length %d/%d took %d seconds, %d slow reads",
                   fspec.c_str(), savlen - len, savlen, ( t1 - t0 ) / 1000, t_error ) ;
        told = t1 ;
      }
      if ( ( t1 - t0 ) > 100000 )                      // Give up after 100 seconds
      {
        dbgprint ( "Give up..." ) ;
        break ;
      }
    }
    tfile.close() ;
    dbgprint ( "EOF" ) ;                               // End of file
  }
}


//******************************************************************************************
//                                  T I M E R 1 0 0                                        *
//******************************************************************************************
// Examine button every 100 msec.                                                          *
//******************************************************************************************
void IRAM_ATTR timer100()
{
  static int     count10sec = 0 ;                 // Counter for activate 10 seconds process
  uint16_t       v ;                              // Analog input value 0..1023
  static uint8_t aoldval = 0 ;                    // Previous value of analog input switch
  uint8_t        anewval ;                        // New value of analog input switch (0..3)

  if ( ++count10sec == 100  )                     // 10 seconds passed?
  {
    timer10sec() ;                                // Yes, do 10 second procedure
    count10sec = 0 ;                              // Reset count
  }
  if ( ( count10sec % 10 ) == 0 )                 // 1 second passed?
  {
    scrollflag = true ;                           // Yes, request scroll of LCD
    if ( ++timeinfo.tm_sec >= 60 )                // Yes, update number of seconds
    {
      timeinfo.tm_sec = 0 ;                       // Wrap after 60 seconds
      if ( ++timeinfo.tm_min >= 60 )
      {
        timeinfo.tm_min = 0 ;                     // Wrap after 60 minutes
        if ( ++timeinfo.tm_hour >= 24 )
        {
          timeinfo.tm_hour = 0 ;                  // Wrap after 24 hours
        }
      }
    }
    time_req = true ;                             // Yes, show current time request
  }
  else
  {
    v = analogRead ( A0 ) ;                       // Read analog value
    anewval = anagetsw ( v ) ;                    // Check analog value for program switches
    if ( anewval != aoldval )                     // Change?
    {
      aoldval = anewval ;                         // Remember value for change detection
      if ( anewval != 0 )                         // Button pushed?
      {
        //dbgprint ( "Analog button %d pushed, v = %d", anewval, v ) ;
        if ( anewval == 1 )                       // Button 1?
        {
          ini_block.newpreset = 0 ;               // Yes, goto first preset
        }
        else if ( anewval == 2 )                  // Button 2?
        {
          ini_block.newpreset = currentpreset + 1 ; // Yes, goto next preset
        }
        else if ( anewval == 3 )                  // Button 3?
        {
          ini_block.newpreset = currentpreset - 1 ; // Yes, goto previous preset
        }
      }
    }
  }
}


//******************************************************************************************
//                        S H O W S T R E A M T I T L E                                    *
//******************************************************************************************
// Show artist and songtitle if present in metadata.                                       *
// Show always if full=true.                                                               *
//******************************************************************************************
void showstreamtitle ( const char *ml, bool full )
{
  char*             p1 ;
  char*             p2 ;
  char              streamtitle[150] ;          // Streamtitle from metadata

  if ( strstr ( ml, "StreamTitle=" ) )
  {
    dbgprint ( "Streamtitle found, %d bytes", strlen ( ml ) ) ;
    dbgprint ( ml ) ;
    p1 = (char*)ml + 12 ;                       // Begin of artist and title
    if ( ( p2 = strstr ( ml, ";" ) ) )          // Search for end of title
    {
      if ( *p1 == '\'' )                        // Surrounded by quotes?
      {
        p1++ ;
        p2-- ;
      }
      *p2 = '\0' ;                              // Strip the rest of the line
    }
    // Save last part of string as streamtitle.  Protect against buffer overflow
    strncpy ( streamtitle, p1, sizeof ( streamtitle ) ) ;
    streamtitle[sizeof ( streamtitle ) - 1] = '\0' ;
  }
  else if ( full )
  {
    // Info probably from playlist
    strncpy ( streamtitle, ml, sizeof ( streamtitle ) ) ;
    streamtitle[sizeof ( streamtitle ) - 1] = '\0' ;
  }
  else
  {
    icystreamtitle = "" ;                       // Unknown type
    return ;                                    // Do not show
  }
  // Save for status request from browser and for MQTT
  icystreamtitle = streamtitle ;
  if ( ( p1 = strstr ( streamtitle, " - " ) ) ) // look for artist/title separator
  {
    *p1++ = '\n' ;                              // Found: replace 3 characters by newline
    p2 = p1 + 2 ;
    if ( *p2 == ' ' )                           // Leading space in title?
    {
      p2++ ;
    }
    strcpy ( p1, p2 ) ;                         // Shift 2nd part of title 2 or 3 places
  }
  displayinfo ( streamtitle, 2 ) ;              // Name of Song & Detail
}


//******************************************************************************************
//                            S T O P _ M P 3 C L I E N T                                  *
//******************************************************************************************
// Disconnect from the server.                                                             *
//******************************************************************************************
void stop_mp3client()
{
  if ( mp3client )
  {
    if ( mp3client->connected() )                    // Need to stop client?
    {
      dbgprint ( "Stopping client" ) ;               // Stop connection to host
      mp3client->flush() ;
      mp3client->stop() ;
      delay ( 500 ) ;
    }
    delete ( mp3client ) ;
    mp3client = NULL ;
  }
}


//******************************************************************************************
//                            C O N N E C T T O H O S T                                    *
//******************************************************************************************
// Connect to the Internet radio server specified by newpreset.                            *
//******************************************************************************************
bool connecttohost()
{
  int         inx ;                                 // Position of ":" in hostname
  char*       pfs ;                                 // Pointer to formatted string
  int         port = 80 ;                           // Port number for host
  String      extension = "/" ;                     // May be like "/mp3" in "skonto.ls.lv:8002/mp3"
  String      hostwoext ;                           // Host without extension and portnumber

  stop_mp3client() ;                                // Disconnect if still connected
  dbgprint ( "Connect to new host %s", host.c_str() ) ;
  displayinfo ( "** Internet radio **", 1 ) ;
  displaytime ( "" ) ;                              // Clear time on LCD screen

  datamode = INIT ;                                 // Start default in metamode
  chunked = false ;                                 // Assume not chunked
  if ( host.endsWith ( ".m3u" ) )                   // Is it an m3u playlist?
  {
    playlist = host ;                               // Save copy of playlist URL
    datamode = PLAYLISTINIT ;                       // Yes, start in PLAYLIST mode
    if ( playlist_num == 0 )                        // First entry to play?
    {
      playlist_num = 1 ;                            // Yes, set index
    }
    dbgprint ( "Playlist request, entry %d", playlist_num ) ;
  }
  // In the URL there may be an extension
  inx = host.indexOf ( "/" ) ;                      // Search for begin of extension
  if ( inx > 0 )                                    // Is there an extension?
  {
    extension = host.substring ( inx ) ;            // Yes, change the default
    hostwoext = host.substring ( 0, inx ) ;         // Host without extension
  }
  // In the URL there may be a portnumber
  inx = host.indexOf ( ":" ) ;                      // Search for separator
  if ( inx >= 0 )                                   // Portnumber available?
  {
    port = host.substring ( inx + 1 ).toInt() ;     // Get portnumber as integer
    hostwoext = host.substring ( 0, inx ) ;         // Host without portnumber
  }
  pfs = dbgprint ( "Connect to %s on port %d, extension %s",
                   hostwoext.c_str(), port, extension.c_str() ) ;
  displayinfo ( pfs, 2 ) ;                          // Preset No.
  mp3client = new WiFiClient() ;
  if ( mp3client->connect ( hostwoext.c_str(), port ) )
  {
    dbgprint ( "Connected to server" ) ;
    // This will send the request to the server. Request metadata.
    mp3client->print ( String ( "GET " ) +
                       extension +
                      String ( " HTTP/1.1\r\n" ) +
                      String ( "Host: " ) +
                      hostwoext +
                      String ( "\r\n" ) +
                      String ( "User-Agent: Esp-radio\r\n" ) +
                      String ( "Icy-MetaData:1\r\n" ) +
                      String ( "Connection: close\r\n\r\n" ) ) ;
    return true ;
  }
  dbgprint ( "Request %s failed!", host.c_str() ) ;
  return false ;
}


//******************************************************************************************
//                               C O N N E C T T O F I L E                                 *
//******************************************************************************************
// Open the local mp3-file.                                                                *
//******************************************************************************************
bool connecttofile()
{
  String path ;                                           // Full file spec
  char*  p ;                                              // Pointer to filename
  displayinfo ( "**** MP3 Player ****", 1 ) ;
  path = host.substring ( 9 ) ;                           // Path, skip the "localhost" part
  mp3file = LittleFS.open ( path, "r" ) ;                 // Open the file
  if ( !mp3file )
  {
    dbgprint ( "Error opening file %s", path.c_str() ) ;  // No luck
    return false ;
  }
  p = (char*)path.c_str() + 1 ;                           // Point to filename
  showstreamtitle ( p, true ) ;                           // Show the filename as title
  displayinfo ( "Playing from local file", 2 ) ;          // Show Source at position 60
  icyname = "" ;                                          // No icy name yet
  chunked = false ;                                       // File not chunked
  return true ;
}


//******************************************************************************************
//                               C O N N E C T W I F I                                     *
//******************************************************************************************
// Connect to WiFi using passwords available in the LittleFS.                              *
// If connection fails, an AP is created and the function returns false.                   *
//******************************************************************************************
bool connectwifi()
{
  char*  pfs ;                                         // Pointer to formatted string
  WiFi.disconnect() ;                                  // After restart the router could
  WiFi.softAPdisconnect(true) ;                        // still keep the old connection
  WiFi.begin ( ini_block.ssid.c_str(),
               ini_block.passwd.c_str() ) ;            // Connect to selected SSID
  dbgprint ( "Try WiFi %s", ini_block.ssid.c_str() ) ; // Message to show during WiFi connect

  //wifi_fpm_auto_sleep_set_in_null_mode ( NULL_MODE ) ; // Disable auto sleep mode

  if (  WiFi.waitForConnectResult() != WL_CONNECTED )  // Try to connect
  {
    dbgprint ( "WiFi Failed!  Trying to setup AP with name %s and password %s.", NAME, NAME ) ;
    WiFi.softAP ( NAME, NAME ) ;                       // This ESP will be an AP
    delay ( 5000 ) ;
    pfs = dbgprint ( "  IP = 192.168.4.1  " ) ;        // Address if AP
    displayinfo ( "*AP mode activated*", 2 ) ;
    return false ;
  }

  pfs = dbgprint ( "IP = %d.%d.%d.%d",
                   WiFi.localIP()[0], 
                   WiFi.localIP()[1], 
                   WiFi.localIP()[2], 
                   WiFi.localIP()[3] ) ;
  displayinfo ( pfs, 3 ) ;                             // Show IP address
  return true ;
}


//******************************************************************************************
//                                   O T A S T A R T                                       *
//******************************************************************************************
// Update via WiFi has been started by Arduino IDE.                                        *
//******************************************************************************************
void otastart()
{
  dbgprint ( "OTA Started" ) ;
}


//******************************************************************************************
//                          R E A D H O S T F R O M I N I F I L E                          *
//******************************************************************************************
// Read the mp3 host from the ini-file specified by the parameter.                         *
// The host will be returned.                                                              *
//******************************************************************************************
String readhostfrominifile ( int8_t preset )
{
  String      path ;                                   // Full file spec as string
  File        inifile ;                                // File containing URL with mp3
  char        tkey[12] ;                               // Key as an array of chars
  String      line ;                                   // Input line from .ini file
  String      linelc ;                                 // Same, but lowercase
  int         inx ;                                    // Position within string
  String      res = "" ;                               // Assume not found

  path = String ( INIFILENAME ) ;                      // Form full path
  inifile = LittleFS.open ( path, "r" ) ;              // Open the file
  if ( inifile )
  {
    sprintf ( tkey, "preset_%02d", preset ) ;          // Form the search key
    while ( inifile.available() )
    {
      line = inifile.readStringUntil ( '\n' ) ;        // Read next line
      linelc = line ;                                  // Copy for lowercase
      linelc.toLowerCase() ;                           // Set to lowercase
      if ( linelc.startsWith ( tkey ) )                // Found the key?
      {
        inx = line.indexOf ( "=" ) ;                   // Get position of "="
        if ( inx > 0 )                                 // Equal sign present?
        {
          line.remove ( 0, inx + 1 ) ;                 // Yes, remove key
          res = chomp ( line ) ;                       // Remove garbage
          break ;                                      // End the while loop
        }
      }
    }
    inifile.close() ;                                  // Close the file
  }
  else
  {
    dbgprint ( "File %s not found, please create one!", INIFILENAME ) ;
  }
  return res ;
}


//******************************************************************************************
//                               R E A D I N I F I L E                                     *
//******************************************************************************************
// Read the .ini file and interpret the commands.                                          *
//******************************************************************************************
void readinifile()
{
  String      path ;                                   // Full file spec as string
  File        inifile ;                                // File containing URL with mp3
  String      line ;                                   // Input line from .ini file

  path = String ( INIFILENAME ) ;                      // Form full path
  inifile = LittleFS.open ( path, "r" ) ;              // Open the file
  if ( inifile )
  {
    while ( inifile.available() )
    {
      line = inifile.readStringUntil ( '\n' ) ;        // Read next line
      analyzeCmd ( line.c_str() ) ;
    }
    inifile.close() ;                                  // Close the file
  }
  else
  {
    dbgprint ( "File %s not found, use save command to create one!", INIFILENAME ) ;
  }
}


//******************************************************************************************
//                            P U B L I S H I P                                            *
//******************************************************************************************
// Publish IP to MQTT broker.                                                              *
//******************************************************************************************
void publishIP()
{
  IPAddress ip ;
  char      ipstr[20] ;                          // Hold IP as string

  if ( ini_block.mqttpubtopic.length() )         // Topic to publish?
  {
    ip = WiFi.localIP() ;
    // Publish IP-adress.  qos=1, retain=true
    sprintf ( ipstr, "%d.%d.%d.%d",
              ip[0], ip[1], ip[2], ip[3] ) ;
    mqttclient.publish ( ini_block.mqttpubtopic.c_str(), 1, true, ipstr ) ;
    dbgprint ( "Publishing IP %s to topic %s",
               ipstr, ini_block.mqttpubtopic.c_str() ) ;
  }
}


//******************************************************************************************
//                            O N M Q T T C O N N E C T                                    *
//******************************************************************************************
// Will be called on connection to the broker.  Subscribe to our topic and publish a topic.*
//******************************************************************************************
void onMqttConnect( bool sessionPresent )
{
  uint16_t    packetIdSub ;
  const char* present = "is" ;                      // Assume Session is present

  if ( !sessionPresent )
  {
    present = "is not" ;                            // Session is NOT present
  }
  dbgprint ( "MQTT Connected to the broker %s, session %s present",
             ini_block.mqttbroker.c_str(), present ) ;
  packetIdSub = mqttclient.subscribe ( ini_block.mqtttopic.c_str(), 2 ) ;
  dbgprint ( "Subscribing to %s at QoS 2, packetId = %d ",
             ini_block.mqtttopic.c_str(),
             packetIdSub ) ;
  publishIP() ;                                     // Topic to publish: IP
}


//******************************************************************************************
//                      O N M Q T T D I S C O N N E C T                                    *
//******************************************************************************************
// Will be called on disconnect.                                                           *
//******************************************************************************************
void onMqttDisconnect ( AsyncMqttClientDisconnectReason reason )
{
  dbgprint ( "MQTT Disconnected from the broker, reason %d, reconnecting...",
             reason ) ;
  if ( mqttcount < MAXMQTTCONNECTS )            // Try again?
  {
    mqttcount++ ;                               // Yes, count number of tries
    mqttclient.connect() ;                      // Reconnect
  }
}


//******************************************************************************************
//                      O N M Q T T S U B S C R I B E                                      *
//******************************************************************************************
// Will be called after a successful subscribe.                                            *
//******************************************************************************************
void onMqttSubscribe ( uint16_t packetId, uint8_t qos )
{
  dbgprint ( "MQTT Subscribe acknowledged, packetId = %d, QoS = %d",
             packetId, qos ) ;
}


//******************************************************************************************
//                              O N M Q T T U N S U B S C R I B E                          *
//******************************************************************************************
// Will be executed if this program unsubscribes from a topic.                             *
// Not used at the moment.                                                                 *
//******************************************************************************************
void onMqttUnsubscribe ( uint16_t packetId )
{
  dbgprint ( "MQTT Unsubscribe acknowledged, packetId = %d",
             packetId ) ;
}


//******************************************************************************************
//                            O N M Q T T M E S S A G E                                    *
//******************************************************************************************
// Executed when a subscribed message is received.                                         *
// Note that message is not delimited by a '\0'.                                           *
//******************************************************************************************
void onMqttMessage ( char* topic, char* payload, AsyncMqttClientMessageProperties properties,
                     size_t len, size_t index, size_t total )
{
  const char*  reply ;                              // Result from analyzeCmd

  // Available properties.qos, properties.dup, properties.retain
  if ( len >= sizeof(cmd) )                         // Message may not be too long
  {
    len = sizeof(cmd) - 1 ;
  }
  strncpy ( cmd, payload, len ) ;                   // Make copy of message
  cmd[len] = '\0' ;                                 // Take care of delimeter
  dbgprint ( "MQTT message arrived [%s], lenght = %d, %s", topic, len, cmd ) ;
  reply = analyzeCmd ( cmd ) ;                      // Analyze command and handle it
  dbgprint ( reply ) ;                              // Result for debugging
}


//******************************************************************************************
//                             O N M Q T T P U B L I S H                                   *
//******************************************************************************************
// Will be executed if a message is published by this program.                             *
// Not used at the moment.                                                                 *
//******************************************************************************************
void onMqttPublish ( uint16_t packetId )
{
  dbgprint ( "MQTT Publish acknowledged, packetId = %d",
             packetId ) ;
}


//******************************************************************************************
//                             S C A N S E R I A L                                         *
//******************************************************************************************
// Listen to commands on the Serial inputline.                                             *
//******************************************************************************************
void scanserial()
{
  static String serialcmd ;                      // Command from Serial input
  char          c ;                              // Input character
  const char*   reply = "" ;                     // Reply string from analyzeCmd
  uint16_t      len ;                            // Length of input string

  while ( Serial.available() )                   // Any input seen?
  {
    c =  (char)Serial.read() ;                   // Yes, read the next input character
    //Serial.write ( c ) ;                       // Echo
    len = serialcmd.length() ;                   // Get the length of the current string
    if ( ( c == '\n' ) || ( c == '\r' ) )
    {
      if ( len )
      {
        strncpy ( cmd, serialcmd.c_str(), sizeof(cmd) ) ;
        reply = analyzeCmd ( cmd ) ;             // Analyze command and handle it
        dbgprint ( reply ) ;                     // Result for debugging
        serialcmd = "" ;                         // Prepare for new command
      }
    }
    if ( c >= ' ' )                              // Only accept useful characters
    {
      serialcmd += c ;                           // Add to the command
    }
    if ( len >= ( sizeof(cmd) - 2 )  )           // Check for excessive length
    {
      serialcmd = "" ;                           // Too long, reset
    }
  }
}


//******************************************************************************************
//                                   M K _ L S A N                                         *
//******************************************************************************************
// Make al list of acceptable networks in .ini file.                                       *
// The result will be stored in anetworks like "|SSID1|SSID2|......|SSIDN|".               *
// The number of acceptable networks will be stored in num_an.                             *
//******************************************************************************************
void mk_lsan()
{
  String      path ;                                   // Full file spec as string
  File        inifile ;                                // File containing URL with mp3
  String      line ;                                   // Input line from .ini file
  String      ssid ;                                   // SSID in line
  int         inx ;                                    // Place of "/"

  num_an = 0 ;                                         // Count acceptable networks
  anetworks = "|" ;                                    // Initial value
  path = String ( INIFILENAME ) ;                      // Form full path
  inifile = LittleFS.open ( path, "r" ) ;              // Open the file
  
  if ( *fixedwifi )                                    // FIXEDWIFI set and not empty?
  {
    line = String ( fixedwifi ) ;
    inx = line.indexOf ( "/" ) ;                       // Find separator between ssid and password
    if ( inx > 0 )                                     // Separator found?
      {
        ssid = ssid.substring ( 5, inx ) ;             // Line holds SSID now
        dbgprint ( "Added SSID %s to acceptable networks",
                  ssid.c_str() ) ;
        anetworks += ssid ;                            // Add to list
        anetworks += "|" ;                             // Separator
        num_an++ ;                                     // Count number oif acceptable networks
      }
  }
  else if ( inifile )
  {
    while ( inifile.available() )
    {
      line = inifile.readStringUntil ( '\n' ) ;        // Read next line
      ssid = line ;                                    // Copy holds original upper/lower case
      line.toLowerCase() ;                             // Case insensitive
      if ( line.startsWith ( "wifi" ) )                // Line with WiFi spec?
      {
        inx = line.indexOf ( "/" ) ;                   // Find separator between ssid and password
        if ( inx > 0 )                                 // Separator found?
        {
          ssid = ssid.substring ( 5, inx ) ;           // Line holds SSID now
          dbgprint ( "Added SSID %s to acceptable networks",
                     ssid.c_str() ) ;
          anetworks += ssid ;                          // Add to list
          anetworks += "|" ;                           // Separator
          num_an++ ;                                   // Count number oif acceptable networks
        }
      }
    }
    inifile.close() ;                                  // Close the file
  }
  else
  {
    dbgprint ( "File %s not found!", INIFILENAME ) ;   // No .ini file
  }
}


//******************************************************************************************
//                             G E T P R E S E T S                                         *
//******************************************************************************************
// Make a list of all preset stations.                                                     *
// The result will be stored in the String presetlist (global data).                       *
//******************************************************************************************
void getpresets()
{
  String              path ;                             // Full file spec as string
  File                inifile ;                          // File containing URL with mp3
  String              line ;                             // Input line from .ini file
  int                 inx ;                              // Position of search char in line
  int                 i ;                                // Loop control
  char                vnr[3] ;                           // 2 digit presetnumber as string

  presetlist = String ( "" ) ;                           // No result yet
  path = String ( INIFILENAME ) ;                        // Form full path
  inifile = LittleFS.open ( path, "r" ) ;                // Open the file
  if ( inifile )
  {
    while ( inifile.available() )
    {
      line = inifile.readStringUntil ( '\n' ) ;          // Read next line
      if ( line.startsWith ( "preset_" ) )               // Found the key?
      {
        i = line.substring(7, 9).toInt() ;               // Get index 00..99
        // Show just comment if available.  Otherwise the preset itself.
        inx = line.indexOf ( "#" ) ;                     // Get position of "#"
        if ( inx > 0 )                                   // Hash sign present?
        {
          line.remove ( 0, inx + 1 ) ;                   // Yes, remove non-comment part
        }
        else
        {
          inx = line.indexOf ( "=" ) ;                   // Get position of "="
          if ( inx > 0 )                                 // Equal sign present?
          {
            line.remove ( 0, inx + 1 ) ;                 // Yes, remove first part of line
          }
        }
        line = chomp ( line ) ;                          // Remove garbage from description
        sprintf ( vnr, "%02d", i ) ;                     // Preset number
        presetlist += ( String ( vnr ) + line +          // 2 digits plus description
                        String ( "|" ) ) ;
      }
    }
    inifile.close() ;                                    // Close the file
  }
}


//******************************************************************************************
//                                  X M L  C A L L B A C K                                 *
//******************************************************************************************
// Process XML tags into variables.                                                        *
//******************************************************************************************
void XML_callback ( uint8_t statusflags, char* tagName, uint16_t tagNameLen,
                    char* data,  uint16_t dataLen )
{
  if ( statusflags & STATUS_START_TAG )
  {
    if ( tagNameLen )
    {
      xmlOpen = String ( tagName ) ;
      //dbgprint ( "Start tag %s",tagName ) ;
    }
  }
  else if ( statusflags & STATUS_END_TAG )
  {
    //dbgprint ( "End tag %s", tagName ) ;
  }
  else if ( statusflags & STATUS_TAG_TEXT )
  {
    xmlTag = String( tagName ) ;
    xmlData = String( data ) ;
    //dbgprint ( Serial.print( "Tag: %s, text: %s", tagName, data ) ;
  }
  else if ( statusflags & STATUS_ATTR_TEXT )
  {
    //dbgprint ( "Attribute: %s, text: %s", tagName, data ) ;
  }
  else if  ( statusflags & STATUS_ERROR )
  {
    //dbgprint ( "XML Parsing error  Tag: %s, text: %s", tagName, data ) ;
  }
}


//******************************************************************************************
//                                  X M L  P A R S E                                       *
//******************************************************************************************
// Parses streams from XML data.                                                           *
//******************************************************************************************
String xmlparse ( String mount )
{
  // Example URL for XML Data Stream:
  // http://playerservices.streamtheworld.com/api/livestream?version=1.5&mount=IHR_TRANAAC&lang=en
  // Clear all variables for use.
  char   tmpstr[200] ;                              // Full GET command, later stream URL
  char   c ;                                        // Next input character from reply
  String urlout ;                                   // Result URL
  bool   urlfound = false ;                         // Result found

  stationServer = "" ;
  stationPort = "" ;
  stationMount = "" ;
  xmlTag = "" ;
  xmlData = "" ;
  stop_mp3client() ;                                // Stop any current wificlient connections
  dbgprint ( "Connect to new iHeartRadio host: %s", mount.c_str() ) ;
  datamode = INIT ;                                 // Start default in metamode
  chunked = false ;                                 // Assume not chunked
  // Create a GET commmand for the request.
  sprintf ( tmpstr, xmlget, mount.c_str() ) ;
  dbgprint ( "%s", tmpstr ) ;
  // Connect to XML stream.
  mp3client = new WiFiClient() ;
  if ( mp3client->connect ( xmlhost, xmlport ) ) {
    dbgprint ( "Connected!" ) ;
    mp3client->print ( String ( tmpstr ) + " HTTP/1.1\r\n"
                       "Host: " + xmlhost + "\r\n"
                       "User-Agent: Mozilla/5.0\r\n"
                       "Connection: close\r\n\r\n" ) ;
    // Check for XML Data.
    while ( true )
    {
      if ( mp3client->available() )
      {
        char c = mp3client->read() ;
        if ( c == '<' )
        {
          c = mp3client->read() ;
          if ( c == '?' )
          {
            xml.processChar ( '<' ) ;
            xml.processChar ( '?' ) ;
            break ;
          }
        }
      }
      yield() ;
    }
    dbgprint ( "XML parser processing..." ) ;
    // Process XML Data.
    while (true) 
    {
      if ( mp3client->available() )
      {
        c = mp3client->read() ;
        xml.processChar ( c ) ;
        if ( xmlTag != "" )
        {
          if ( xmlTag.endsWith ( "/status-code" ) )   // Status code seen?
          {
            if ( xmlData != "200" )                   // Good result?
            {
              dbgprint ( "Bad xml status-code %s",    // No, show and stop interpreting
                          xmlData.c_str() ) ;
              break ;
            }
          }
          if ( xmlTag.endsWith ( "/ip" ) )
          {
            stationServer = xmlData ;
          }
          else if ( xmlTag.endsWith ( "/port" ) )
          {
            stationPort = xmlData ;
          }
          else if ( xmlTag.endsWith ( "/mount"  ) )
          {
            stationMount = xmlData ;
          }
        }
      }
      // Check if all the station values are stored.
      urlfound = ( stationServer != "" && stationPort != "" && stationMount != "" ) ;
      if ( urlfound )
      {
        xml.reset() ;
        break ;
      }
      yield() ;
    }
    tmpstr[0] = '\0' ;                            
    if ( urlfound )
    {
      sprintf ( tmpstr, "%s:%s/%s_SC",                 // Build URL for ESP-Radio to stream.
                        stationServer.c_str(),
                        stationPort.c_str(),
                        stationMount.c_str() ) ;
      dbgprint ( "Found: %s", tmpstr ) ;
    }
    dbgprint ( "Closing XML connection." ) ;
    stop_mp3client () ;
  }
  else
  {
    dbgprint ( "Can't connect to XML host!" ) ;
    tmpstr[0] = '\0' ;                            
  }
  return String ( tmpstr ) ;                           // Return final streaming URL.
}


//******************************************************************************************
//                                   S E T U P                                             *
//******************************************************************************************
// Setup for the program.                                                                  *
//******************************************************************************************
void setup()
{
  FSInfo      fs_info ;                                // Info about LittleFS
  Dir         dir ;                                    // Directory struct for LittleFS
  File        f ;                                      // Filehandle
  String      filename ;                               // Name of file found in LittleFS

  Serial.begin ( 115200 ) ;                            // For debug
  Serial.println() ;
  system_update_cpu_freq ( 160 ) ;                     // Set to 80/160 MHz
  #if defined ( SRAM )
    spiram.Setup() ;                                   // Yes, do set-up
    emptyring() ;                                      // Empty the buffer
  #else
    //ESP.setExternalHeap();                             // Set external memory to use
    ringbuf = (uint8_t *) malloc ( RINGBFSIZ ) ;       // Create ring buffer
    dbgprint ( "External buffer: Address %p, free %d\n",
                                    ringbuf, ESP.getFreeHeap() ) ;
    //ESP.resetHeap();
  #endif

  #ifdef FIXEDWIFI                                     // Set fixedwifi if defined
    fixedwifi = FIXEDWIFI ;
  #endif
  xml.init ( xmlbuffer, sizeof(xmlbuffer),             // Initilize XML stream.
             &XML_callback ) ;
  //memset ( &ini_block, 0, sizeof(ini_block) ) ;      // Init ini_block
  ini_block.mqttbroker = "" ;
  ini_block.mqttport   = 1883 ;                        // Default port for MQTT
  ini_block.mqttuser   = "" ;
  ini_block.mqttpasswd = "" ;
  ini_block.mqtttopic  = "" ;
  ini_block.mqttpubtopic = "" ;
  ini_block.clk_server = "pool.ntp.org" ;              // Default server for NTP
  ini_block.clk_offset = 0 ;                           // Default UTC time zone
  ini_block.clk_dst = 1 ;                              // DST is +1 hour
  ini_block.reqvol = 0 ;
  memset ( ini_block.rtone, 0, 4 )  ;
  ini_block.newpreset = 0 ;
  ini_block.ssid = "" ;
  ini_block.passwd = "" ;
  LittleFS.begin() ;                                   // Enable file system
  LittleFS.info ( fs_info ) ;
  dbgprint ( "FS Total %d, used %d", 
             fs_info.totalBytes, fs_info.usedBytes ) ; // Show some info about the LittleFS
  if ( fs_info.totalBytes == 0 )
  {
    dbgprint ( "No LittleFS found! See the documentation." ) ;
  }
  dir = LittleFS.openDir("/") ;                        // Show files in FS
  while ( dir.next() )                                 // All files
  {
    f = dir.openFile ( "r" ) ;
    filename = dir.fileName() ;
    dbgprint ( "%-32s - %7d",                          // Show name and size
               filename.c_str(), f.size() ) ;
  }
  mk_lsan() ;                                          // Make a list of acceptable networks in ini file.
  listNetworks() ;                                     // Search for WiFi networks
  readinifile() ;                                      // Read .ini file
  getpresets() ;                                       // Get the presets from .ini-file
  WiFi.setPhyMode ( WIFI_PHY_MODE_11N ) ;              // Force 802.11N connection
  WiFi.persistent ( false ) ;                          // Do not save SSID and password
  WiFi.disconnect() ;                                  // The router may keep the old connection
  WiFi.mode ( WIFI_STA ) ;                             // This ESP is a station
  wifi_station_set_hostname ( (char*)NAME ) ;
  dbgprint ( "Starting ESP Version %s...  Free memory %d",
             VERSION,
             system_get_free_heap_size() ) ;           // Print some memory info
  dbgprint ( "Sketch size %d, free size %d",
             ESP.getSketchSize(),
             ESP.getFreeSketchSpace() ) ;              // And sketch info
#if defined ( IR )
  setupIR( IR_PIN ) ;
#endif
  vs1053player.begin() ;                               // Initialize VS1053 player
  if ( vs1053player.getChipVersion() == 4 )            // Check if we are using VS1053B chip
  { 
    vs1053player.loadDefaultVs1053Patches();           // Then load default patch set 
  }
#if defined ( LCD )
  dsp_begin();
  displayinfo ( "      Esp-radio     ", 0 ) ;
  delay ( 10 ) ;
  displayinfo ( "       Loading      ", 2 ) ;
#endif
  delay ( 10 ) ;
  analogrest = ( analogRead ( A0 ) + asw1 ) / 2  ;     // Assumed inactive analog input
  tckr.attach ( 0.100, timer100 ) ;                    // Every 100 msec
  dbgprint ( "Selected network: %-25s", ini_block.ssid.c_str() ) ;
  NetworkFound = connectwifi() ;                       // Connect to WiFi network
  //NetworkFound = false ;                             // TEST, uncomment for no network test
  dbgprint ( "Start server for commands" ) ;
  cmdserver.on ( "/", handleCmd ) ;                    // Handle startpage
  cmdserver.onNotFound ( handleFS ) ;                  // Handle file from FS
  cmdserver.onFileUpload ( handleFileUpload ) ;        // Handle file uploads
  cmdserver.begin() ;
  if ( NetworkFound )                                  // OTA and MQTT only if Wifi network found
  {
    ArduinoOTA.setHostname ( NAME ) ;                  // Set the hostname
    ArduinoOTA.onStart ( otastart ) ;
    ArduinoOTA.begin() ;                               // Allow update over the air
    if ( ini_block.mqttbroker.length() )               // Broker specified?
    {
      // Initialize the MQTT client
      WiFi.hostByName ( ini_block.mqttbroker.c_str(),
                        mqtt_server_IP ) ;             // Lookup IP of MQTT server
      mqttclient.onConnect ( onMqttConnect ) ;
      mqttclient.onDisconnect ( onMqttDisconnect ) ;
      mqttclient.onSubscribe ( onMqttSubscribe ) ;
      mqttclient.onUnsubscribe ( onMqttUnsubscribe ) ;
      mqttclient.onMessage ( onMqttMessage ) ;
      mqttclient.onPublish ( onMqttPublish ) ;
      mqttclient.setServer ( mqtt_server_IP,           // Specify the broker
                             ini_block.mqttport ) ;    // And the port
      mqttclient.setCredentials ( ini_block.mqttuser.c_str(),
                                  ini_block.mqttpasswd.c_str() ) ;
      mqttclient.setClientId ( NAME ) ;
      dbgprint ( "Connecting to MQTT %s, port %d, user %s, password %s...",
                 ini_block.mqttbroker.c_str(),
                 ini_block.mqttport,
                 ini_block.mqttuser.c_str(),
                 ini_block.mqttpasswd.c_str() ) ;
      mqttclient.connect();
    }
  }
  else
  {
    currentpreset = ini_block.newpreset ;              // No network: do not start radio
  }
  delay ( 1000 ) ;                                     // Show IP for a while
  configTime ( ini_block.clk_offset * 3600,
               ini_block.clk_dst * 3600,
               ini_block.clk_server.c_str() ) ;        // GMT offset, daylight offset in seconds
  timeinfo.tm_year = 0 ;                               // Set TOD to illegal
  analogrest = ( analogRead ( A0 ) + asw1 ) / 2  ;     // Assumed inactive analog input
  if ( NetworkFound )
  {
    gettime() ;                                        // Sync time
  }
  #if defined ( SRAM )
    dbgprint ( "Testing SPIRAM getring/putring" ) ;
    for ( int i = 0 ; i < 96 ; i++ )                   // Test for 96 bytes, 3 chunks
    {
      putring ( i ) ;                                  // Store in spiram
      dbgprint ( "Test 1: %d, chunks avl is %d",       // Test, expect 0,0 1,0 2,0 .... 95,3
                i, ringavail() ) ;
    }
    for ( int i = 0 ; i < 96 ; i++ )                   // Test for 100 bytes
    {
      uint8_t c = getring() ;                          // Read from spiram
      dbgprint ( "Test 2: %d, data is %d, ch av is %d",
                i, c, ringavail() ) ;                  // Test, expect 0,0,2 1,1,2 2,2,2 .... 95,95,0
    }
    dbgprint ( "Chunks avl is %d",                     // Test, expect 0
              ringavail() ) ;
  #endif
}


//******************************************************************************************
//                                   L O O P                                               *
//******************************************************************************************
// Main loop of the program.  Minimal time is 20 usec.  Will take about 4 msec if VS1053   *
// needs data.                                                                             *
// Sometimes the loop is called after an interval of more than 100 msec.                   *
// In that case we will not be able to fill the internal VS1053-fifo in time (especially   *
// at high bitrate).                                                                       *
// A connection to an MP3 server is active and we are ready to receive data.               *
// Normally there is about 2 to 4 kB available in the data stream.  This depends on the    *
// sender.                                                                                 *
//******************************************************************************************
void loop()
{
  uint32_t    maxfilechunk  ;                          // Max number of bytes to read from
                                                       // stream or file
  // Try to keep the ringbuffer filled up by adding as much bytes as possible
  if ( datamode & ( INIT | HEADER | DATA |             // Test op playing
                    METADATA | PLAYLISTINIT |
                    PLAYLISTHEADER |
                    PLAYLISTDATA ) )
  {
    if ( localfile )
    {
      maxfilechunk = mp3file.available() ;             // Bytes left in file
      if ( maxfilechunk > 1024 )                       // Reduce byte count for this loop()
      {
        maxfilechunk = 1024 ;
      }
      while ( ringspace() && maxfilechunk-- )
      {
        putring ( mp3file.read() ) ;                   // Yes, store one byte in ringbuffer
        yield() ;
      }
    }
    else
    {
      maxfilechunk = mp3client->available() ;          // Bytes available from mp3 server
      if ( maxfilechunk > 1024 )                       // Reduce byte count for this loop()
      {
        maxfilechunk = 1024 ;
      }
      while ( ringspace() && maxfilechunk-- )
      {
        putring ( mp3client->read() ) ;                // Yes, store one byte in ringbuffer
        yield() ;
      }
    }
    yield() ;
  }
  while ( vs1053player.data_request() && ringavail() ) // Try to keep VS1053 filled
  {
  #if defined ( SRAM )
    if ( spiram.spiramdelay != 0 )                     // Delay before reading SPIRAM?
    {
      spiram.spiramdelay-- ;                           // Yes, count down
      break ;                                          // and skip handling of data
    }
  #endif
    handlebyte_ch ( getring() ) ;                      // Yes, handle it
  }
  yield() ;
  if ( datamode == STOPREQD )                          // STOP requested?
  {
    dbgprint ( "STOP requested" ) ;
    if ( localfile )
    {
      mp3file.close() ;
    }
    else
    {
      stop_mp3client() ;                               // Disconnect if still connected
    }
    handlebyte_ch ( 0, true ) ;                        // Force flush of buffer
    vs1053player.setVolume ( 0 ) ;                     // Mute
    vs1053player.stopSong() ;                          // Stop playing
    emptyring() ;                                      // Empty the ringbuffer
    datamode = STOPPED ;                               // Yes, state becomes STOPPED
    delay ( 500 ) ;
  }
  if ( localfile )
  {
    if ( datamode & ( INIT | HEADER | DATA |           // Test op playing
                      METADATA | PLAYLISTINIT |
                      PLAYLISTHEADER |
                      PLAYLISTDATA ) )
    {
      if ( ( mp3file.available() == 0 ) && ( ringavail() == 0 ) )
      {
        datamode = STOPREQD ;                          // End of local mp3-file detected
      }
    }
  }
  if ( ini_block.newpreset != currentpreset )          // New station or next from playlist requested?
  {
    if ( datamode != STOPPED )                         // Yes, still busy?
    {
      datamode = STOPREQD ;                            // Yes, request STOP
    }
    else
    {
      if ( playlist_num )                              // Playing from playlist?
      { // Yes, retrieve URL of playlist
        playlist_num += ini_block.newpreset -
                        currentpreset ;                // Next entry in playlist
        ini_block.newpreset = currentpreset ;          // Stay at current preset
      }
      else
      {
        host = readhostfrominifile ( ini_block.newpreset ) ; // Lookup preset in ini-file
      }
      dbgprint ( "New preset/file requested (%d/%d) from %s",
                 currentpreset, playlist_num, host.c_str() ) ;
      if ( host != ""  )                               // Preset in ini-file?
      {
        hostreq = true ;                               // Force this station as new preset
      }
      else
      {
        // This preset is not available, return to preset 0, will be handled in next loop()
        ini_block.newpreset = 0 ;                      // Wrap to first station
      }
    }
  }
  if ( hostreq )                                       // New preset or station?
  {
    hostreq = false ;
    currentpreset = ini_block.newpreset ;              // Remember current preset
    
    localfile = host.startsWith ( "localhost/" ) ;     // Find out if this URL is on localhost
    if ( localfile )                                   // Play file from localhost?
    {
      if ( connecttofile() )                           // Yes, open mp3-file
      {
        datamode = DATA ;                              // Start in DATA mode
      }
    }
    else
    {
      if ( host.startsWith ( "ihr/" ) )                // iHeartRadio station requested?
      {
        host = host.substring ( 4 ) ;                  // Yes, remove "ihr/"
        host = xmlparse ( host ) ;                     // Parse the xml to get the host
      }
      connecttohost() ;                                // Switch to new host
    }
  }
  if ( xmlreq )                                        // Directly xml requested?
  {
    xmlreq = false ;                                   // Yes, clear request flag
    host = xmlparse ( host ) ;                         // Parse the xml to get the host
    connecttohost() ;                                  // and connect to this host
  }
  if ( reqtone )                                       // Request to change tone?
  {
    reqtone = false ;
    vs1053player.setTone ( ini_block.rtone ) ;         // Set SCI_BASS to requested value
  }
  if ( resetreq )                                      // Reset requested?
  {
    delay ( 1000 ) ;                                   // Yes, wait some time
    ESP.restart() ;                                    // Reboot
  }
  if ( muteflag )
  {
    vs1053player.setVolume ( 0 ) ;                     // Mute
  }
  else
  {
    vs1053player.setVolume ( ini_block.reqvol ) ;      // Unmute
  }
  if ( testfilename.length() )                         // File to test?
  {
    testfile ( testfilename ) ;                        // Yes, do the test
    testfilename = "" ;                                // Clear test request
  }
  if ( time_req && NetworkFound )                      // Time to refresh time?
  {
    gettime() ;                                        // Yes, get the current time
  }
  #if defined ( LCD )
    if ( time_req )                                    // Time to refresh timetxt?
    {
      time_req = false ;                               // Yes, clear request
      if ( NetworkFound )                              // Time available?
      {
        displaytime ( timetxt ) ;                      // Write to TFT screen
      }
      displayvolume ( vs1053player.getVolume() ) ;     // Show volume on display
    }
    if ( scrollflag )                                  // Time to scroll?
    {
      dsp_update() ;                                   // LCD scroll
      scrollflag = false ;                             // Yes, reset flag
    }
  #endif
  scanserial() ;                                       // Handle serial input
  ArduinoOTA.handle() ;                                // Check for OTA
  #if defined ( IR )
    scanIR() ;                                         // See if IR input
  #endif
}


//**************************************************************************************************
//                             D E C O D E _ S P E C _ C H A R S                                   *
//**************************************************************************************************
// Decode special characters like "&#39;".                                                         *
//**************************************************************************************************
String decode_spec_chars ( String str )
{
  int    inx, inx2 ;                                  // Indexes in string
  char   c ;                                          // Character from string
  char   val ;                                        // Converted character
  String res = str ;

  while ( ( inx = res.indexOf ( "&#" ) ) >= 0 )       // Start sequence in string?
  {
    inx2 = res.indexOf ( ";", inx ) ;                 // Yes, search for stop character
    if ( inx2 < 0 )                                   // Stop character found
    {
      break ;                                         // Malformed string
    }
    res = str.substring ( 0, inx ) ;                  // First part
    inx += 2 ;                                        // skip over start sequence
    val = 0 ;                                         // Init result of 
    while ( ( c = str[inx++] ) != ';' )               // Convert character
    {
      val = val * 10 + c - '0' ;
    }
    res += ( String ( val ) +                         // Add special char to string
             str.substring ( ++inx2 ) ) ;             // Add rest of string
  }
  return res ;
}


//******************************************************************************************
//                            C H K H D R L I N E                                          *
//******************************************************************************************
// Check if a line in the header is a reasonable headerline.                               *
// Normally it should contain something like "icy-xxxx:abcdef".                            *
//******************************************************************************************
bool chkhdrline ( const char* str )
{
  char    b ;                                         // Byte examined
  int     len = 0 ;                                   // Lengte van de string

  while ( ( b = *str++ ) )                            // Search to end of string
  {
    len++ ;                                           // Update string length
    if ( ! isalpha ( b ) )                            // Alpha (a-z, A-Z)
    {
      if ( b != '-' )                                 // Minus sign is allowed
      {
        if ( b == ':' )                               // Found a colon?
        {
          return ( ( len > 5 ) && ( len < 50 ) ) ;    // Yes, okay if length is okay
        }
        else
        {
          return false ;                              // Not a legal character
        }
      }
    }
  }
  return false ;                                      // End of string without colon
}


//**************************************************************************************************
//                            S C A N _ C O N T E N T _ L E N G T H                                *
//**************************************************************************************************
// If the line contains content-length information: set clength (content length counter).          *
//**************************************************************************************************
void scan_content_length ( const char* metalinebf )
{
  if ( strstr ( metalinebf, "Content-Length" ) )      // Line contains content length
  {
    clength = atoi ( metalinebf + 15 ) ;              // Yes, set clength
    dbgprint ( "Content-Length is %d", clength ) ;    // Show for debugging purposes
  }
}


//******************************************************************************************
//                           H A N D L E B Y T E _ C H                                     *
//******************************************************************************************
// Handle the next byte of data from server.                                               *
// Chunked transfer encoding aware. Chunk extensions are not supported.                    *
//******************************************************************************************
void handlebyte_ch ( uint8_t b, bool force )
{
  static int  chunksize = 0 ;                         // Chunkcount read from stream

  if ( chunked && !force && 
       ( datamode & ( DATA |                          // Test op DATA handling
                      METADATA |
                      PLAYLISTDATA ) ) )
  {
    if ( chunkcount == 0 )                            // Expecting a new chunkcount?
    {
       if ( b == '\r' )                               // Skip CR
       {
         return ;      
       }
       else if ( b == '\n' )                          // LF ?
       {
         chunkcount = chunksize ;                     // Yes, set new count
         chunksize = 0 ;                              // For next decode
         return ;
       }
       // We have received a hexadecimal character.  Decode it and add to the result.
       b = toupper ( b ) - '0' ;                      // Be sure we have uppercase
       if ( b > 9 )
       {
         b = b - 7 ;                                  // Translate A..F to 10..15
       }
       chunksize = ( chunksize << 4 ) + b ;
    }
    else
    {
      handlebyte ( b, force ) ;                       // Normal data byte
      chunkcount-- ;                                  // Update count to next chunksize block
    }
  }
  else
  {
    handlebyte ( b, force ) ;                         // Normal handling of this byte
  }
}


//******************************************************************************************
//                           H A N D L E B Y T E                                           *
//******************************************************************************************
// Handle the next byte of data from server.                                               *
// This byte will be send to the VS1053 most of the time.                                  *
// Note that the buffer the data chunk must start at an address that is a muttiple of 4.   *
// Set force to true if chunkbuffer must be flushed.                                       *
//******************************************************************************************
void handlebyte ( uint8_t b, bool force )
{
  static uint16_t  playlistcnt ;                       // Counter to find right entry in playlist
  static bool      firstmetabyte ;                     // True if first metabyte (counter)
  static int       LFcount ;                           // Detection of end of header
  static __attribute__((aligned(4))) uint8_t buf[32] ; // Buffer for chunk
  static int       bufcnt = 0 ;                        // Data in chunk
  static bool      firstchunk = true ;                 // First chunk as input
  String           lcml ;                              // Lower case metaline
  String           ct ;                                // Contents type
  static bool      ctseen = false ;                    // First line of header seen or not
  static bool      redirection = false ;               // Redirection or not
  int              inx ;                               // Pointer in metaline
  int              i ;                                 // Loop control

  if ( datamode == INIT )                              // Initialize for header receive
  {
    ctseen = false ;                                   // Contents type not seen yet
    redirection = false ;                              // No redirect yet
    metaint = 0 ;                                      // No metaint found
    LFcount = 0 ;                                      // For detection end of header
    bitrate = 0 ;                                      // Bitrate still unknown
    dbgprint ( "Switch to HEADER" ) ;
    datamode = HEADER ;                                // Handle header
    totalcount = 0 ;                                   // Reset totalcount
    metaline = "" ;                                    // No metadata yet
    firstchunk = true ;                                // First chunk expected
  }
  if ( datamode == DATA )                              // Handle next byte of MP3/Ogg data
  {
    buf[bufcnt++] = b ;                                // Save byte in chunkbuffer
    if ( bufcnt == sizeof(buf) || force )              // Buffer full?
    {
      if ( firstchunk )
      {
        firstchunk = false ;
        dbgprint ( "First chunk:" ) ;                  // Header for printout of first chunk
        for ( i = 0 ; i < 32 ; i += 8 )                // Print 4 lines
        {
          dbgprint ( "%02X %02X %02X %02X %02X %02X %02X %02X",
                     buf[i],   buf[i + 1], buf[i + 2], buf[i + 3],
                     buf[i + 4], buf[i + 5], buf[i + 6], buf[i + 7] ) ;
        }
      }
      vs1053player.playChunk ( buf, bufcnt ) ;         // Yes, send to player
      bufcnt = 0 ;                                     // Reset count
    }
    totalcount++ ;                                     // Count number of bytes, ignore overflow
    if ( metaint != 0 )                                // No METADATA on Ogg streams or mp3 files
    {
      if ( --datacount == 0 )                          // End of datablock?
      {
        if ( bufcnt )                                  // Yes, still data in buffer?
        {
          vs1053player.playChunk ( buf, bufcnt ) ;     // Yes, send to player
          bufcnt = 0 ;                                 // Reset count
        }
        datamode = METADATA ;
        firstmetabyte = true ;                         // Expecting first metabyte (counter)
      }
    }
    return ;
  }
  if ( datamode == HEADER )                            // Handle next byte of MP3 header
  {
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      // Yes, ignore
    }
    else if ( b == '\n' )                              // Linefeed ?
    {
      LFcount++ ;                                      // Count linefeeds
      if ( chkhdrline ( metaline.c_str() ) )           // Reasonable input?
      {
        lcml = metaline ;                              // Use lower case for compare
        lcml.toLowerCase() ;
        dbgprint ( metaline.c_str() ) ;                // Yes, Show it
        if ( lcml.startsWith ( "location: " ) )        // Redirection?
        {
          redirection = true ;
          if ( lcml.indexOf ( "http://" ) > 8 )        // Redirection with http://?
          {
            host = metaline.substring ( 17 ) ;         // Yes, get new URL
            hostreq = true ;
          }
          else if ( lcml.indexOf ( "https://" ) )      // Redirection with ttps://?
          {
            host = metaline.substring ( 18 ) ;         // Yes, get new URL
            hostreq = true ;
          }
        }
        if ( lcml.indexOf ( "content-type" ) >= 0 )    // Line with "Content-Type: xxxx/yyy"
        {
          ctseen = true ;                              // Yes, remember seeing this
          String ct = metaline.substring ( 14 ) ;      // Set contentstype. Not used yet
          ct.trim() ;
          dbgprint ( "%s seen.", ct.c_str() ) ;
        }
        if ( lcml.startsWith ( "icy-br:" ) )
        {
          bitrate = metaline.substring(7).toInt() ;    // Found bitrate tag, read the bitrate
          if ( bitrate == 0 )                          // For Ogg br is like "Quality 2"
          {
            bitrate = 87 ;                             // Dummy bitrate
          }
        }
        else if ( lcml.startsWith ( "icy-metaint:" ) )
        {
          metaint = metaline.substring(12).toInt() ;   // Found metaint tag, read the value
        }
        else if ( lcml.startsWith ( "icy-name:" ) )
        {
          icyname = metaline.substring(9) ;            // Get station name
          icyname = decode_spec_chars ( icyname ) ;    // Decode special characters in name
          icyname.trim() ;                             // Remove leading and trailing spaces
          displayinfo ( icyname.c_str(), 1 ) ;         // Show station name
        }
        else if ( lcml.startsWith ( "transfer-encoding:" ) )
        {
          // Station provides chunked transfer
          if ( lcml.endsWith ( "chunked" ) )
          {
            chunked = true ;                           // Remember chunked transfer mode
            chunkcount = 0 ;                           // Expect chunkcount in DATA
          }
        }
      }
      metaline = "" ;                                  // Reset this line
      if ( LFcount == 2 )                              // Double linfeed ends header
      {
        bufcnt = 0 ;                                   // Reset buffer count
        if ( ctseen )                                  // Some data seen and a double LF?
        {
          dbgprint ( "Switch to DATA, bitrate is %d"   // Show bitrate
                      ", metaint is %d",               // and metaint
                      mbitrate, metaint ) ;
          datamode = DATA ;                            // Expecting data now
        #if defined ( SRAM )
          spiram.spiramdelay = SPIRAMDELAY ;           // Start delay
        #endif
          //emptyring() ;                              // Empty SPIRAM buffer (not sure about this)
          datacount = metaint ;                        // Number of bytes before first metadata
          bufcnt = 0 ;                                 // Reset buffer count
          vs1053player.switchToMp3Mode() ;
          vs1053player.startSong() ;                   // Start a new song
        }
        if ( redirection )                             // Redirect seen?
        {
           datamode = INIT ;
        }
      }
    }
    else
    {
      metaline += (char)b ;                            // Normal character, put new char in metaline
      LFcount = 0 ;                                    // Reset double CRLF detection
    }
    return ;
  }
  if ( datamode == METADATA )                          // Handle next byte of metadata
  {
    if ( firstmetabyte )                               // First byte of metadata?
    {
      firstmetabyte = false ;                          // Not the first anymore
      metacount = b * 16 + 1 ;                         // New count for metadata including length byte
      if ( metacount > 1 )
      {
        dbgprint ( "Metadata block %d bytes",
                   metacount - 1 ) ;                   // Most of the time there are zero bytes of metadata
      }
      metaline = "" ;                                  // Set to empty
    }
    else
    {
      metaline += (char)b ;                            // Normal character, put new char in metaline
    }
    if ( --metacount == 0 )
    {
      if ( metaline.length() )                         // Any info present?
      {
        // metaline contains artist and song name.  For example:
        // "StreamTitle='Don McLean - American Pie';StreamUrl='';"
        // Sometimes it is just other info like:
        // "StreamTitle='60s 03 05 Magic60s';StreamUrl='';"
        // Isolate the StreamTitle, remove leading and trailing quotes if present.
        showstreamtitle ( metaline.c_str() ) ;         // Show artist and title if present in metadata
      }
      if ( metaline.length() > 1500 )                  // Unlikely metaline length?
      {
        dbgprint ( "Metadata block too long! Skipping all Metadata from now on." ) ;
        metaint = 0 ;                                  // Probably no metadata
        metaline = "" ;                                // Do not waste memory on this
      }
      datacount = metaint ;                            // Reset data count
      bufcnt = 0 ;                                     // Reset buffer count
      datamode = DATA ;                                // Expecting data
    }
  }
  if ( datamode == PLAYLISTINIT )                      // Initialize for receive .m3u file
  {
    // We are going to use metadata to read the lines from the .m3u file
    metaline = "" ;                                    // Prepare for new line
    LFcount = 0 ;                                      // For detection end of header
    datamode = PLAYLISTHEADER ;                        // Handle playlist data
    playlistcnt = 1 ;                                  // Reset for compare
    totalcount = 0 ;                                   // Reset totalcount
    dbgprint ( "Read from playlist" ) ;
  }
  if ( datamode == PLAYLISTHEADER )                    // Read header
  {
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      return ;                                         // Quick return
    }
    else if ( b == '\n' )                              // Linefeed ?
    {
      LFcount++ ;                                      // Count linefeeds
      dbgprint ( "Playlistheader: %s",                 // Show playlistheader
                 metaline.c_str() ) ;
      scan_content_length ( metaline.c_str() ) ;       // Check if it is a content-length line
      metaline = "" ;                                  // Ready for next line
      if ( LFcount == 2 )
      {
        dbgprint ( "Switch to PLAYLISTDATA" ) ;
        datamode = PLAYLISTDATA ;                      // Expecting data now
        return ;
      }
    }
    else
    {
      metaline += (char)b ;                            // Normal character, put new char in metaline
      LFcount = 0 ;                                    // Reset double CRLF detection
    }
  }
  if ( datamode == PLAYLISTDATA )                      // Read next byte of .m3u file data
  {
    if ( ( b > 0x7F ) ||                               // Ignore unprintable characters
         ( b == '\r' ) ||                              // Ignore CR
         ( b == '\0' ) )                               // Ignore NULL
    {
      // Yes, ignore
    }
    else if ( b == '\n' )                              // Linefeed ?
    {
      dbgprint ( "Playlistdata: %s",                   // Show playlistheader
                 metaline.c_str() ) ;
      if ( metaline.length() < 5 )                     // Skip short lines
      {
        return ;
      }
      if ( metaline.indexOf ( "#EXTINF:" ) >= 0 )      // Info?
      {
        if ( playlist_num == playlistcnt )             // Info for this entry?
        {
          inx = metaline.indexOf ( "," ) ;             // Comma in this line?
          if ( inx > 0 )
          {
            // Show artist and title if present in metadata
            showstreamtitle ( metaline.substring ( inx + 1 ).c_str(), true ) ;
          }
        }
      }
      if ( metaline.startsWith ( "#" ) )               // Commentline?
      {
        metaline = "" ;
        return ;                                       // Ignore commentlines
      }
      // Now we have an URL for a .mp3 file or stream.  Is it the rigth one?
      dbgprint ( "Entry %d in playlist found: %s", playlistcnt, metaline.c_str() ) ;
      if ( playlist_num == playlistcnt  )
      {
        inx = metaline.indexOf ( "http://" ) ;         // Search for "http://"
        if ( inx >= 0 )                                // Does URL contain "http://"?
        {
          host = metaline.substring ( inx + 7 ) ;      // Yes, remove it and set host
        }
        else
        {
          host = metaline ;                            // Yes, set new host
        }
        connecttohost() ;                              // Connect to it
      }
      metaline = "" ;
      host = playlist ;                                // Back to the .m3u host
      playlistcnt++ ;                                  // Next entry in playlist
    }
    else
    {
      metaline += (char)b ;                            // Normal character, add it to metaline
    }
    return ;
  }
}


//******************************************************************************************
//                             G E T C O N T E N T T Y P E                                 *
//******************************************************************************************
// Returns the contenttype of a file to send.                                              *
//******************************************************************************************
String getContentType ( String filename )
{
  if      ( filename.endsWith ( ".html" ) ) return "text/html" ;
  else if ( filename.endsWith ( ".png"  ) ) return "image/png" ;
  else if ( filename.endsWith ( ".gif"  ) ) return "image/gif" ;
  else if ( filename.endsWith ( ".jpg"  ) ) return "image/jpeg" ;
  else if ( filename.endsWith ( ".ico"  ) ) return "image/x-icon" ;
  else if ( filename.endsWith ( ".css"  ) ) return "text/css" ;
  else if ( filename.endsWith ( ".zip"  ) ) return "application/x-zip" ;
  else if ( filename.endsWith ( ".gz"   ) ) return "application/x-gzip" ;
  else if ( filename.endsWith ( ".mp3"  ) ) return "audio/mpeg" ;
  else if ( filename.endsWith ( ".pw"   ) ) return "" ;              // Passwords are secret
  return "text/plain" ;
}


//******************************************************************************************
//                         H A N D L E F I L E U P L O A D                                 *
//******************************************************************************************
// Handling of upload request.  Write file to LittleFS.                                      *
//******************************************************************************************
void handleFileUpload ( AsyncWebServerRequest *request, String filename,
                        size_t index, uint8_t *data, size_t len, bool final )
{
  String          path ;                              // Filename including "/"
  static File     f ;                                 // File handle output file
  char*           reply ;                             // Reply for webserver
  static uint32_t t ;                                 // Timer for progress messages
  uint32_t        t1 ;                                // For compare
  static uint32_t totallength ;                       // Total file length
  static size_t   lastindex ;                         // To test same index

  if ( index == 0 )
  {
    path = String ( "/" ) + filename ;                // Form LittleFS filename
    LittleFS.remove ( path ) ;                        // Remove old file
    f = LittleFS.open ( path, "w" ) ;                 // Create new file
    t = millis() ;                                    // Start time
    totallength = 0 ;                                 // Total file lengt still zero
    lastindex = 0 ;                                   // Prepare test
  }
  t1 = millis() ;                                     // Current timestamp
  // Yes, print progress
  dbgprint ( "File upload %s, t = %d msec, len %d, index %d",
               filename.c_str(), t1 - t, len, index ) ;
  if ( len )                                          // Something to write?
  {
    if ( ( index != lastindex ) || ( index == 0 ) )   // New chunk?
    {
      f.write ( data, len ) ;                         // Yes, transfer to LittleFS
      totallength += len ;                            // Update stored length
      lastindex = index ;                             // Remenber this part
    }
  }
  if ( final )                                        // Was this last chunk?
  {
    f.close() ;                                       // Yes, clode the file
    reply = dbgprint ( "File upload %s, %d bytes finished",
                       filename.c_str(), totallength ) ;
    request->send ( 200, "", reply ) ;
  }
}


//******************************************************************************************
//                                H A N D L E F S F                                        *
//******************************************************************************************
// Handling of requesting files from the LittleFS/PROGMEM. Example: /favicon.ico           *
//******************************************************************************************
void handleFSf ( AsyncWebServerRequest* request, const String& filename )
{
  static String          ct ;                           // Content type
  AsyncWebServerResponse *response ;                    // For extra headers

  dbgprint ( "FileRequest received %s", filename.c_str() ) ;
  ct = getContentType ( filename ) ;                    // Get content type
  if ( ( ct == "" ) || ( filename == "" ) )             // Empty is illegal
  {
    request->send ( 404, "text/plain", "File not found" ) ;
  }
  else
  {
    if ( filename.indexOf ( "index.html" ) >= 0 )       // Index page is in PROGMEM
    {
      response = request->beginResponse_P ( 200, ct, index_html ) ;
    }
    else if ( filename.indexOf ( "radio.css" ) >= 0 )   // CSS file is in PROGMEM
    {
      response = request->beginResponse_P ( 200, ct, radio_css ) ;
    }
    else if ( filename.indexOf ( "config.html" ) >= 0 ) // Config page is in PROGMEM
    {
      response = request->beginResponse_P ( 200, ct, config_html ) ;
    }
    else if ( filename.indexOf ( "about.html" ) >= 0 )  // About page is in PROGMEM
    {
      response = request->beginResponse_P ( 200, ct, about_html ) ;
    }
    else if ( filename.indexOf ( "favicon.ico" ) >= 0 ) // Favicon icon is in PROGMEM
    {
      response = request->beginResponse_P ( 200, ct, favicon_ico, sizeof ( favicon_ico ) ) ;
    }
    else
    {
      response = request->beginResponse ( LittleFS, filename, ct ) ;
    }
    // Add extra headers
    response->addHeader ( "Server", NAME ) ;
    response->addHeader ( "Cache-Control", "max-age=3600" ) ;
    response->addHeader ( "Last-Modified", VERSION ) ;
    request->send ( response ) ;
  }
  dbgprint ( "Response sent" ) ;
}


//******************************************************************************************
//                                H A N D L E F S                                          *
//******************************************************************************************
// Handling of requesting files from the LittleFS. Example: /favicon.ico                   *
//******************************************************************************************
void handleFS ( AsyncWebServerRequest* request )
{
  handleFSf ( request, request->url() ) ;               // Rest of handling
}


//******************************************************************************************
//                             A N A L Y Z E C M D                                         *
//******************************************************************************************
// Handling of the various commands from remote webclient, Serial or MQTT.                 *
// Version for handling string with: <parameter>=<value>                                   *
//******************************************************************************************
const char* analyzeCmd ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd

  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              // Separate command from value
    res = analyzeCmd ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}


//******************************************************************************************
//                                 C H O M P                                               *
//******************************************************************************************
// Do some filtering on de inputstring:                                                    *
//  - String comment part (starting with "#").                                             *
//  - Strip trailing CR.                                                                   *
//  - Strip leading spaces.                                                                *
//  - Strip trailing spaces.                                                               *
//******************************************************************************************
String chomp ( String str )
{
  int   inx ;                                         // Index in de input string

  if ( ( inx = str.indexOf ( "#" ) ) >= 0 )           // Comment line or partial comment?
  {
    str.remove ( inx ) ;                              // Yes, remove
  }
  str.trim() ;                                        // Remove spaces and CR
  return str ;                                        // Return the result
}


//******************************************************************************************
//                             A N A L Y Z E C M D                                         *
//******************************************************************************************
// Handling of the various commands from remote webclient, serial or MQTT.                 *
// par holds the parametername and val holds the value.                                    *
// "wifi_00" and "preset_00" may appear more than once, like wifi_01, wifi_02, etc.        *
// Examples with available parameters:                                                     *
//   preset     = 12                        // Select start preset to connect to           *
//   preset_00  = <mp3 stream>              // Specify station for a preset 00-99 *)       *
//   volume     = 95                        // Percentage between 0 and 100                *
//   upvolume   = 2                         // Add percentage to current volume            *
//   downvolume = 2                         // Subtract percentage from current volume     *
//   toneha     = <0..15>                   // Setting treble gain                         *
//   tonehf     = <0..15>                   // Setting treble frequency                    *
//   tonela     = <0..15>                   // Setting bass gain                           *
//   tonelf     = <0..15>                   // Setting treble frequency                    *
//   station    = <mp3 stream>              // Select new station (will not be saved)      *
//   station    = <URL>.mp3                 // Play standalone .mp3 file (not saved)       *
//   station    = <URL>.m3u                 // Select playlist (will not be saved)         *
//   stop                                   // Stop playing                                *
//   resume                                 // Resume playing                              *
//   mute                                   // Mute/unmute the music (toggle)              *
//   wifi_00    = mySSID/mypassword         // Set WiFi SSID and password *)               *
//   mqttbroker = mybroker.com              // Set MQTT broker to use *)                   *
//   mqttport   = 1883                      // Set MQTT port to use, default 1883 *)       *
//   mqttuser   = myuser                    // Set MQTT user for authentication *)         *
//   mqttpasswd = mypassword                // Set MQTT password for authentication *)     *
//   mqtttopic  = mytopic                   // Set MQTT topic to subscribe to *)           *
//   mqttpubtopic = mypubtopic              // Set MQTT topic to publish to *)             *
//   status                                 // Show current URL to play                    *
//   testfile   = <file on LittleFS>        // Test LittleFS reads for debugging purpose   *
//   test                                   // For test purposes                           *
//   debug      = 0 or 1                    // Switch debugging on or off                  *
//   reset                                  // Restart the ESP8266                         *
//   analog                                 // Show current analog input                   *
// Commands marked with "*)" are sensible in ini-file only                                 *
// Note that it is adviced to avoid expressions as the argument for the abs function.      *
//******************************************************************************************
const char* analyzeCmd ( const char* par, const char* val )
{
  String             argument ;                       // Argument as string
  String             value ;                          // Value of an argument as a string
  int                ivalue ;                         // Value of argument as an integer
  static char        reply[250] ;                     // Reply to client, will be returned
  uint8_t            oldvol ;                         // Current volume
  bool               relative ;                       // Relative argument (+ or -)
  int                inx ;                            // Index in string

  strcpy ( reply, "Command accepted" ) ;              // Default reply
  argument = chomp ( par ) ;                          // Get the argument
  if ( argument.length() == 0 )                       // Empty commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  argument.toLowerCase() ;                            // Force to lower case
  value = chomp ( val ) ;                             // Get the specified value
  ivalue = value.toInt() ;                            // Also as an integer
  ivalue = abs ( ivalue ) ;                           // Make it absolute
  relative = argument.indexOf ( "up" ) == 0 ;         // + relative setting?
  if ( argument.indexOf ( "down" ) == 0 )             // - relative setting?
  {
    relative = true ;                                 // It's relative
    ivalue = - ivalue ;                               // But with negative value
  }
  if ( value.startsWith ( "http://" ) )               // Does (possible) URL contain "http://"?
  {
    value.remove ( 0, 7 ) ;                           // Yes, remove it
  }
  if ( value.length() )
  {
    dbgprint ( "Command: %s with parameter %s",
               argument.c_str(), value.c_str() ) ;
  }
  else
  {
    dbgprint ( "Command: %s (without parameter)",
               argument.c_str() ) ;
  }
  if ( argument.indexOf ( "volume" ) >= 0 )           // Volume setting?
  {
    // Volume may be of the form "upvolume", "downvolume" or "volume" for relative or absolute setting
    oldvol = vs1053player.getVolume() ;               // Get current volume
    if ( relative )                                   // + relative setting?
    {
      ini_block.reqvol = oldvol + ivalue ;            // Up by 0.5 or more dB
    }
    else
    {
      ini_block.reqvol = ivalue ;                     // Absolue setting
    }
    if ( ini_block.reqvol > 100 )
    {
      ini_block.reqvol = 100 ;                        // Limit to normal values
    }
    sprintf ( reply, "Volume now is %d",              // Reply new volume
              ini_block.reqvol ) ;
  }
  else if ( argument == "mute" )                      // Mute/unmute request
  {
    muteflag = !muteflag ;                            // Request volume to zero/normal
  }
  else if ( argument.indexOf ( "preset" ) >= 0 )      // Preset station?
  {
    if ( !argument.startsWith ( "preset_" ) )         // But not a station URL
    {
      if ( relative )                                 // Relative argument?
      {
        ini_block.newpreset += ivalue ;               // Yes, adjust currentpreset
      }
      else
      {
        ini_block.newpreset = ivalue ;                // Otherwise set preset station
      }
      sprintf ( reply, "Preset is now %d",            // Reply new preset
                ini_block.newpreset ) ;
      playlist_num = 0 ;
    }
  }
  else if ( argument == "stop" )                      // Stop requested?
  {
    if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                      PLAYLISTHEADER | PLAYLISTDATA ) )
    {
      datamode = STOPREQD ;                           // Request STOP
    }
    else
    {
      strcpy ( reply, "Command not accepted!" ) ;     // Error reply
    }
  }
  else if ( argument == "resume" )                    // Request to resume?
  {
    if ( datamode == STOPPED )                        // Yes, are we stopped?
    {
      hostreq = true ;                                // Yes, request restart
    }
  }
  else if ( argument == "station" )                   // Station in the form address:port
  {
    if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                      PLAYLISTHEADER | PLAYLISTDATA ) )
    {
      datamode = STOPREQD ;                           // Request STOP
    }
    host = value ;                                    // Save it for storage and selection later
    hostreq = true ;                                  // Force this station as new preset
    sprintf ( reply,
              "New preset station %s accepted",       // Format reply
              host.c_str() ) ;
    utf8ascii_ip ( reply ) ;                          // Remove possible strange characters
  }
  else if ( argument == "xml" )
  {
    if ( datamode & ( HEADER | DATA | METADATA | PLAYLISTINIT |
                      PLAYLISTHEADER | PLAYLISTDATA ) )
    {
      datamode = STOPREQD ;                           // Request STOP
    }
    host = value ;                                    // Save it for storage and selection later
    xmlreq = true ;                                   // Run XML parsing process.
    sprintf ( reply,
              "New xml preset station %s accepted",   // Format reply
              host.c_str() ) ;
    utf8ascii_ip ( reply ) ;                          // Remove possible strange characters
  }
  else if ( argument == "status" )                    // Status request
  {
    if ( datamode == STOPPED )
    {
      sprintf ( reply, "Player stopped" ) ;           // Format reply
    }
    else
    {
      sprintf ( reply, "%s - %s", icyname.c_str(),
                icystreamtitle.c_str() ) ;            // Streamtitle from metadata
    }
  }
  else if ( argument.startsWith ( "reset" ) )         // Reset request
  {
    resetreq = true ;                                 // Reset all
  }
  else if ( argument == "testfile" )                  // Testfile command?
  {
    testfilename = value ;                            // Yes, set file to test accordingly
  }
  else if ( argument == "test" )                      // Test command
  {
  #if defined ( SRAM )                                // SPI RAM used?
    rcount = spiram.dataAvailable() ;                 // Yes, get free space
  #endif
    if ( mp3client )
    {
      sprintf ( reply, "Free memory is %d, ringbuf %d, stream %d, bitrate %d kbps",
              system_get_free_heap_size(), rcount, mp3client->available(), bitrate ) ;
    }
    else
    {
      sprintf ( reply, "Free memory is %d, ringbuf %d, No stream available",
              system_get_free_heap_size(), rcount ) ;      
    }
  }
  // Commands for bass/treble control
  else if ( argument.startsWith ( "tone" ) )          // Tone command
  {
    if ( argument.indexOf ( "ha" ) > 0 )              // High amplitue? (for treble)
    {
      ini_block.rtone[0] = ivalue ;                   // Yes, prepare to set ST_AMPLITUDE
    }
    if ( argument.indexOf ( "hf" ) > 0 )              // High frequency? (for treble)
    {
      ini_block.rtone[1] = ivalue ;                   // Yes, prepare to set ST_FREQLIMIT
    }
    if ( argument.indexOf ( "la" ) > 0 )              // Low amplitue? (for bass)
    {
      ini_block.rtone[2] = ivalue ;                   // Yes, prepare to set SB_AMPLITUDE
    }
    if ( argument.indexOf ( "lf" ) > 0 )              // High frequency? (for bass)
    {
      ini_block.rtone[3] = ivalue ;                   // Yes, prepare to set SB_FREQLIMIT
    }
    reqtone = true ;                                  // Set change request
    sprintf ( reply, "Parameter for bass/treble %s set to %d",
              argument.c_str(), ivalue ) ;
  }
  else if ( argument == "rate" )                      // Rate command?
  {
    vs1053player.adjustRate ( ivalue ) ;              // Yes, adjust
  }
  else if ( argument.startsWith ( "clk" ) )           // Parameter fo NTP?
  {
    if ( argument.indexOf ( "server" ) > 0 )          // Server specified?
    {
      ini_block.clk_server = value.c_str() ;          // Yes, set NTP server accordingly
    }
    else if ( argument.indexOf ( "offset" ) > 0 )     // Offset specified?
    {
      ini_block.clk_offset = ivalue ;                 // Yes, set offset 
    }
    else if ( argument.indexOf ( "dst" ) > 0 )        // DST specified?
    {
      ini_block.clk_dst = ivalue ;                    // Yes, set DST accordingly
    }
  }
  else if ( argument.startsWith ( "mqtt" ) )          // Parameter fo MQTT?
  {
    strcpy ( reply, "MQTT broker parameter changed. Save and restart to have effect" ) ;
    if ( argument.indexOf ( "broker" ) > 0 )          // Broker specified?
    {
      ini_block.mqttbroker = value.c_str() ;          // Yes, set broker accordingly
    }
    else if ( argument.indexOf ( "port" ) > 0 )       // Port specified?
    {
      ini_block.mqttport = ivalue ;                   // Yes, set port user accordingly
    }
    else if ( argument.indexOf ( "user" ) > 0 )       // User specified?
    {
      ini_block.mqttuser = value ;                    // Yes, set user accordingly
    }
    else if ( argument.indexOf ( "passwd" ) > 0 )     // Password specified?
    {
      ini_block.mqttpasswd = value.c_str() ;          // Yes, set broker password accordingly
    }
    else if ( argument.indexOf ( "pubtopic" ) > 0 )   // Publish topic specified?
    {
      ini_block.mqttpubtopic = value.c_str() ;        // Yes, set broker password accordingly
    }
    else if ( argument.indexOf ( "topic" ) > 0 )      // Topic specified?
    {
      ini_block.mqtttopic = value.c_str() ;           // Yes, set broker topic accordingly
    }
  }
  else if ( argument == "debug" )                     // debug on/off request?
  {
    DEBUG = ivalue ;                                  // Yes, set flag accordingly
  }
  else if ( argument == "analog" )                    // Show analog request?
  {
    sprintf ( reply, "Analog input = %d units",       // Read the analog input for test
              analogRead ( A0 ) ) ;
  }
  else if ( argument.startsWith ( "wifi" ) )          // WiFi SSID and passwd?
  {
    inx = value.indexOf ( "/" ) ;                     // Find separator between ssid and password
    // Was this the strongest SSID or the only acceptable?
    if ( num_an == 1 )
    {
      ini_block.ssid = value.substring ( 0, inx ) ;   // Only one.  Set as the strongest
    }
    if ( value.substring ( 0, inx ) == ini_block.ssid )
    {
      ini_block.passwd = value.substring ( inx + 1 ) ; // Yes, set password
    }
  }
  else if ( argument == "getnetworks" )               // List all WiFi networks?
  {
    sprintf ( reply, networks.c_str() ) ;             // Reply is SSIDs
  }
  else
  {
    sprintf ( reply, "%s called with illegal parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;                                      // Return reply to the caller
}


//******************************************************************************************
//                             H A N D L E C M D                                           *
//******************************************************************************************
// Handling of the various commands from remote (case sensitive). All commands have the    *
// form "/?parameter[=value]".  Example: "/?volume=50".                                    *
// The startpage will be returned if no arguments are given.                               *
// Multiple parameters are ignored.  An extra parameter may be "version=<random number>"   *
// in order to prevent browsers like Edge and IE to use their cache.  This "version" is    *
// ignored.                                                                                *
// Example: "/?upvolume=5&version=0.9775479450590543"                                      *
// The save and the list commands are handled specially.                                   *
//******************************************************************************************
void handleCmd ( AsyncWebServerRequest* request )
{
  AsyncWebParameter* p ;                                // Points to parameter structure
  static String      argument ;                         // Next argument in command
  static String      value ;                            // Value of an argument
  const char*        reply ;                            // Reply to client
  //uint32_t         t ;                                // For time test
  int                params ;                           // Number of params
  static File        f ;                                // Handle for writing /radio.ini to LittleFS

  //t = millis() ;                                      // Timestamp at start
  params = request->params() ;                          // Get number of arguments
  if ( params == 0 )                                    // Any arguments
  {
    if ( NetworkFound )
    {
      handleFSf ( request, String( "/index.html") ) ;   // No parameters, send the startpage
    }
    else
    {
      handleFSf ( request, String( "/config.html") ) ;  // Or the configuration page if in AP mode
    }
    return ;
  }
  p = request->getParam ( 0 ) ;                         // Get pointer to parameter structure
  argument = p->name() ;                                // Get the argument
  argument.toLowerCase() ;                              // Force to lower case
  value = p->value() ;                                  // Get the specified value
  // For the "save" command, the contents is the value of the next parameter
  if ( argument.startsWith ( "save" ) && ( params > 1 ) )
  {
    reply = "Error saving " INIFILENAME ;               // Default reply
    p = request->getParam ( 1 ) ;                       // Get pointer to next parameter structure
    if ( p->isPost() )                                  // Does it have a POST?
    {
      f = LittleFS.open ( INIFILENAME, "w" ) ;          // Save to inifile
      if ( f )
      {
        f.print ( p->value() ) ;
        f.close() ;
        reply = dbgprint ( "%s saved", INIFILENAME ) ;
      }
    }
  }
  else if ( argument.startsWith ( "list" ) )            // List all presets?
  {
    dbgprint ( "list request from browser" ) ;
    request->send ( 200, "text/plain", presetlist ) ;   // Send the reply
    return ;
  }
  else
  {
    reply = analyzeCmd ( argument.c_str(),              // Analyze it
                         value.c_str() ) ;
  }
  request->send ( 200, "text/plain", reply ) ;          // Send the reply
  //t = millis() - t ;
  // If it takes too long to send a reply, we run into the "LmacRxBlk:1"-problem.
  // Reset the ESP8266.....
  //if ( t > 8000 )
  //{
  //  ESP.restart() ;                                   // Last resource
  //}
}
