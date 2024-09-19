//***************************************************************************************************
//   SPIRAM.h - Driver for 23LC1024 SPI RAM chip                                                    *
//***************************************************************************************************
//                                                                                                  *
//***************************************************************************************************
//

#include "SPI.h"

#define SRAM_CS       PIN_SPI_SS                        // CS pin connected to GPIO 15
#define SRAM_FREQ     20e6                              // 23LC1024 supports theorically up to 20MHz
#define SRAM_SIZE     131072                            // Total size SPI RAM in bytes
#define CHUNKSIZE     32                                // Chunk size
#define SRAM_CH_SIZE  4096                              // Total size SPI RAM in chunks
#define SPIRAMDELAY   SRAM_SIZE                         // Delay before reading from SPIRAM

extern char* dbgprint ( const char* format, ... ) ;

class SPIRAM
{
  public:
                      SPIRAM() ;
                      SPIRAM ( uint8_t cs, uint8_t clockspeedhz ) ;
    uint8_t           prcwinx ;                         // Index in pwchunk (see putring)
    uint8_t           prcrinx ;                         // Index in prchunk (see getring)
    int32_t           spiramdelay = SPIRAMDELAY ;       // Delay before reading from SPIRAM
    void              Setup() ;
    void              Test() ;
    void              bufferReset() ;
    bool              spaceAvailable() ;
    uint16_t          dataAvailable() ;
    uint16_t          getFreeBufferSpace() ;
    void              bufferWrite ( uint8_t *b ) ;
    void              bufferRead ( uint8_t *b ) ;
  private:
    uint8_t           Cs ;
    uint32_t          clkSpeed ;
    uint16_t          chcount ;                         // Number of chunks currently in buffer
    uint32_t          readinx ;                         // Read index
    uint32_t          writeinx ;                        // write index
    void              Read   ( uint32_t addr, uint8_t *buff, uint32_t size ) ;
    void              Write  ( uint32_t addr, uint8_t *buff, uint32_t size ) ;
};


SPIRAM spiram ;


//******************************************************************************************
// SPI RAM routines.                                                                       *
//******************************************************************************************
// Use SPI RAM as a circular buffer with chunks of 32 bytes.                               *
//******************************************************************************************
void SPIRAM::Write ( uint32_t addr, uint8_t* buff, uint32_t size )
{
  SPI.beginTransaction ( SPISettings(SRAM_FREQ, MSBFIRST, SPI_MODE0 ) ) ;
  digitalWrite ( SRAM_CS, LOW ) ;

  SPI.transfer ( 0x02 ) ;                               // Transfer write command
  SPI.transfer ( ( addr >> 16 ) & 0xFF ) ;              // MSB of the address
  SPI.transfer ( ( addr >> 8) & 0xFF ) ;
  SPI.transfer ( addr & 0xFF ) ;                        // LSB of the address

  while ( size-- )
  {
    SPI.transfer ( *buff++ ) ;                          // Transfer data
  }

  digitalWrite ( SRAM_CS, HIGH ) ;
  SPI.endTransaction() ;
}

void SPIRAM::Read ( uint32_t addr, uint8_t* buff, uint32_t size )
{
  SPI.beginTransaction ( SPISettings(SRAM_FREQ, MSBFIRST, SPI_MODE0 ) ) ;
  digitalWrite ( SRAM_CS, LOW ) ;

  SPI.transfer ( 0x03 ) ;                               // Transfer read command
  SPI.transfer ( ( addr >> 16) & 0xFF ) ;               // MSB of the address
  SPI.transfer ( ( addr >> 8) & 0xFF ) ;
  SPI.transfer ( addr & 0xFF ) ;                        // LSB of the address

  while ( size-- )
  {
    *buff++ = SPI.transfer ( 0x00 ) ;                   // Receive data
  }

  digitalWrite ( SRAM_CS, HIGH ) ;
  SPI.endTransaction() ;
}


//******************************************************************************************
//                              S P A C E A V A I L A B L E                                *
//******************************************************************************************
// Returns true if bufferspace is available.                                               *
//******************************************************************************************
bool SPIRAM::spaceAvailable()
{
  return ( chcount < SRAM_CH_SIZE ) ;
}


//******************************************************************************************
//                              D A T A A V A I L A B L E                                  *
//******************************************************************************************
// Returns the number of full chunks available in the buffer.                              *
//******************************************************************************************
uint16_t SPIRAM::dataAvailable()
{
  return chcount ;
}


//******************************************************************************************
//                    G E T F R E E B U F F E R S P A C E                                  *
//******************************************************************************************
// Return the free buffer space in chunks.                                                 *
//******************************************************************************************
uint16_t SPIRAM::getFreeBufferSpace()
{
  return ( SRAM_CH_SIZE - chcount ) ;                   // Return number of chunks available
}


//******************************************************************************************
//                             B U F F E R W R I T E                                       *
//******************************************************************************************
// Write one chunk (32 bytes) to SPI RAM.                                                  *
// No check on available space.  See spaceAvailable().                                     *
//******************************************************************************************
void SPIRAM::bufferWrite ( uint8_t *b )
{
  Write ( writeinx * CHUNKSIZE, b, CHUNKSIZE ) ;        // Put byte in SPIRAM
  writeinx = ( writeinx + 1 ) % SRAM_CH_SIZE ;          // Increment and wrap if necessary
  chcount++ ;                                           // Count number of chunks
}


//******************************************************************************************
//                             B U F F E R R E A D                                         *
//******************************************************************************************
// Read one chunk in the user buffer.                                                      *
// Assume there is always something in the bufferpace.  See dataAvailable()                *
//******************************************************************************************
void SPIRAM::bufferRead ( uint8_t *b )
{
  Read ( readinx * CHUNKSIZE, b, CHUNKSIZE ) ;          // Return next chunk
  readinx = ( readinx + 1 ) % SRAM_CH_SIZE ;            // Increment and wrap if necessary
  chcount-- ;                                           // Count is now one less
}


//******************************************************************************************
//                            B U F F E R R E S E T                                        *
//******************************************************************************************
void SPIRAM::bufferReset()
{
  readinx = 0 ;                                         // Reset ringbuffer administration
  writeinx = 0 ;
  chcount = 0 ;
}


//******************************************************************************************
//                                S P I R A M                                              *
//******************************************************************************************
// Constructor for the SPI RAM.                                                            *
//******************************************************************************************
SPIRAM::SPIRAM()
{
  Cs = SRAM_CS ;
  clkSpeed = SRAM_FREQ ;
}

SPIRAM::SPIRAM ( uint8_t cs, uint8_t clockspeedhz )
{
  Cs = cs ;
  clkSpeed = clockspeedhz ;
}


//******************************************************************************************
//                                S P I R A M S E T U P                                    *
//******************************************************************************************
void SPIRAM::Setup()
{
  SPI.begin() ;
  pinMode ( Cs, OUTPUT ) ;
  digitalWrite ( Cs, HIGH ) ;
  delay ( 50 ) ;
  digitalWrite ( Cs, LOW ) ;
  delay ( 50 ) ;
  digitalWrite ( Cs, HIGH ) ;

  SPI.beginTransaction ( SPISettings ( clkSpeed, MSBFIRST, SPI_MODE0 ) ) ;
  digitalWrite ( Cs, LOW ) ;
  SPI.transfer ( 0x01 ) ;                               // Write mode register
  SPI.transfer ( 0x40 ) ;                               // Set seq mode
  digitalWrite ( Cs, HIGH ) ;
  SPI.endTransaction();

  bufferReset() ;                                       // Reset ringbuffer administration
}


//******************************************************************************************
//                                S P I R A M T E S T                                      *
//******************************************************************************************
void SPIRAM::Test()
{
  uint8_t writeData[CHUNKSIZE] = { 0xDE, 0xAD, 0xBE, 0xEF } ;
  uint8_t readData[CHUNKSIZE] = { 0x00 } ;

  bufferWrite ( writeData ) ;                           // Write to SPI RAM
  bufferRead ( readData ) ;                             // Read from SPI RAM

  bool match = true ;
  for ( int i = 0; i < CHUNKSIZE; ++i )
  {
    if ( readData[i] != writeData[i] )
    {
      match = false ;
      break ;
    }
  }

  if ( match )
  {
    dbgprint ( "SPI RAM detected and functioning correctly" ) ;
  }
  else
  {
    dbgprint ( "Failed to verify SPI RAM. Check connections and settings" ) ;
  }
}
