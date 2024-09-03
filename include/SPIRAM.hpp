//***************************************************************************************************
//   SPIRAM.h - Driver for 23LC1024 SPI RAM chip                                                    *
//***************************************************************************************************
//                                                                                                  *
//***************************************************************************************************
//

#include "SPI.h"


class SPIRAM
{
  public:
                      SPIRAM() ;
                      SPIRAM ( uint8_t cs, uint8_t clockspeedhz ) ;
    uint8_t           prcwinx ;                             // Index in pwchunk (see putring)
    uint8_t           prcrinx ;                             // Index in prchunk (see getring)
    int32_t           spiramdelay = SPIRAMDELAY ;           // Delay before reading from SPIRAM
    void              Setup() ;
    void              bufferReset() ;
    bool              spaceAvailable() ;
    uint16_t          dataAvailable() ;
    uint16_t          getFreeBufferSpace() ;
    void              bufferWrite ( uint8_t *b ) ;
    void              bufferRead ( uint8_t *b ) ;
  private:
    uint8_t           Cs ;
    uint32_t          clkSpeed ;
    uint16_t          chcount ;                             // Number of chunks currently in buffer
    uint32_t          readinx ;                             // Read index
    uint32_t          writeinx ;                            // write index
    void              spiramRead   ( uint32_t addr, uint8_t *buff, uint32_t size ) ;
    void              spiramWrite  ( uint32_t addr, uint8_t *buff, uint32_t size ) ;
};


SPIRAM spiram ;


//******************************************************************************************
// SPI RAM routines.                                                                       *
//******************************************************************************************
// Use SPI RAM as a circular buffer with chunks of 32 bytes.                               *
//******************************************************************************************
void SPIRAM::spiramWrite ( uint32_t addr, uint8_t *buff, uint32_t size )
{
  int i = 0;
  SPI.beginTransaction ( SPISettings(SRAM_FREQ, MSBFIRST, SPI_MODE0 ) ) ;
  while ( size-- )
  {
    digitalWrite ( SRAM_CS, LOW ) ;

    uint32_t data = ( 0x02 << 24 ) | (addr++ & 0x00ffffff ) ;
    SPI.transfer16 ( data >> 16 ) ;       // Transfer MSB
    SPI.transfer16 ( data & 0xFFFF ) ;    // Transfer LSB
    SPI.transfer ( buff[i++] ) ;

    digitalWrite ( SRAM_CS, HIGH ) ;

    if (i % 32 == 0)
    {
      yield() ;                           // Yield to reset the watchdog every 32 iterations
    }
  }

  SPI.endTransaction();
}

void SPIRAM::spiramRead ( uint32_t addr, uint8_t *buff, uint32_t size )
{
  int i = 0;
  SPI.beginTransaction ( SPISettings(SRAM_FREQ, MSBFIRST, SPI_MODE0 ) ) ;
  while ( size-- )
  {
    digitalWrite ( SRAM_CS, LOW ) ;

    uint32_t data = ( 0x03 << 24 ) | ( addr++ & 0x00ffffff ) ;
    SPI.transfer16 ( data >> 16 ) ;       // Transfer MSB
    SPI.transfer16 ( data & 0xFFFF ) ;    // Transfer LSB
    buff[i++] = SPI.transfer ( 0x00 ) ;

    digitalWrite ( SRAM_CS, HIGH ) ;

    if (i % 32 == 0)
    {
      yield() ;                           // Yield to reset the watchdog every 32 iterations
    }
  }
  SPI.endTransaction();
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
  spiramWrite ( writeinx * CHUNKSIZE, b, CHUNKSIZE ) ;  // Put byte in SRAM
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
  spiramRead ( readinx * CHUNKSIZE, b, CHUNKSIZE ) ;   // return next chunk
  readinx = ( readinx + 1 ) % SRAM_CH_SIZE ;           // Increment and wrap if necessary
  chcount-- ;                                          // Count is now one less
}


//******************************************************************************************
//                            B U F F E R R E S E T                                        *
//******************************************************************************************
void SPIRAM::bufferReset()
{
  readinx = 0 ;                                        // Reset ringbuffer administration
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
  Cs = cs;
  clkSpeed = clockspeedhz;
}


//******************************************************************************************
//                                S P I R A M S E T U P                                    *
//******************************************************************************************
void SPIRAM::Setup()
{
  SPI.begin();
  pinMode ( Cs, OUTPUT ) ;
  digitalWrite ( Cs, HIGH ) ;
  delay ( 50 ) ;
  digitalWrite ( Cs, LOW ) ;
  delay ( 50 ) ;
  digitalWrite ( Cs, HIGH ) ;

  SPI.beginTransaction ( SPISettings ( clkSpeed, MSBFIRST, SPI_MODE0 ) ) ;
  digitalWrite ( Cs, LOW ) ;
  SPI.transfer ( 0x01 ) ;                             // Write mode register
  SPI.transfer ( 0x00 ) ;                             // Set byte mode
  digitalWrite ( Cs, HIGH ) ;
  SPI.endTransaction();

  bufferReset() ;                                     // Reset ringbuffer administration
}
