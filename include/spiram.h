//******************************************************************************************
// SPI RAM routines.                                                                       *
//******************************************************************************************
// Use SPI RAM as a circular buffer with chunks of 32 bytes.                               *
//******************************************************************************************

#include <SPI.h>

#define SRAM_SIZE  131072                   // Total size SPI ram in bytes
#define CHUNKSIZE      32                   // Chunk size
#define SRAM_CH_SIZE 4096                   // Total size SPI ram in chunks

#define SRAM_CS        10                   // GPIO1O SRAM CS pin
#define SRAM_FREQ    10e6                   // The 23LC1024 supports theorically up to 20MHz

// SRAM opcodes
#define SRAM_READ    0x03
#define SRAM_WRITE   0x02

// Global variables
uint16_t   chcount ;                       // Number of chunks currently in buffer
uint32_t   readinx ;                       // Read index
uint32_t   writeinx ;                      // write index


//******************************************************************************************
//                              S P A C E A V A I L A B L E                                *
//******************************************************************************************
// Returns true if bufferspace is available.                                               *
//******************************************************************************************
bool spaceAvailable()
{
  return ( chcount < SRAM_CH_SIZE ) ;
}


//******************************************************************************************
//                              D A T A A V A I L A B L E                                  *
//******************************************************************************************
// Returns the number of chunks available in the buffer.                                   *
//******************************************************************************************
uint16_t dataAvailable()
{
  return chcount ;
}


//******************************************************************************************
//                    G E T F R E E B U F F E R S P A C E                                  *
//******************************************************************************************
// Return the free buffer space in chunks.                                                 *
//******************************************************************************************
uint16_t getFreeBufferSpace()
{
  return ( SRAM_CH_SIZE - chcount ) ;                // Return number of chuinks available
}


//******************************************************************************************
//                             B U F F E R W R I T E                                       *
//******************************************************************************************
// Write one chunk (32 bytes) to SPI RAM.                                                  *
// No check on available space.  See spaceAvailable().                                     *
//******************************************************************************************
void spiramWrite ( uint32_t addr, uint8_t *buff, uint32_t size )
{
 	uint32_t i = 0;
  digitalWrite ( SRAM_CS, LOW ) ;
  SPI.transfer ( SRAM_WRITE ) ;
  SPI.transfer((uint8_t)(addr >> 16) & 0xff);
  SPI.transfer((uint8_t)(addr >> 8) & 0xff);
  SPI.transfer((uint8_t)addr);
	for ( i = 0; i < size; i++ )
	{
    SPI.transfer(buff[i]);
	}
  digitalWrite(SRAM_CS, HIGH);
}

void bufferWrite ( uint8_t *b )
{
  spiramWrite ( writeinx, b, CHUNKSIZE ) ;          // Put byte in SRAM
  writeinx = ( writeinx + 1 ) % SRAM_CH_SIZE ;      // Increment and wrap if necessary
  chcount++ ;                                       // Count number of chunks
}


//******************************************************************************************
//                             B U F F E R R E A D                                         *
//******************************************************************************************
// Read one chunk in the user buffer.                                                      *
// Assume there is always something in the bufferpace.  See dataAvailable()                *
//******************************************************************************************
uint32_t spiramRead ( uint32_t addr, uint8_t *buff, uint32_t size )
{
  uint32_t i = 0 ;
  digitalWrite ( SRAM_CS, LOW ) ;
  SPI.transfer ( SRAM_READ ) ;
  SPI.transfer((uint8_t)(addr >> 16) & 0xff);
  SPI.transfer((uint8_t)(addr >> 8) & 0xff);
  SPI.transfer((uint8_t)addr);
	for ( i = 0; i < size; i++ )
	{
    buff[i] = SPI.transfer(0x00);
	}
  digitalWrite(SRAM_CS, HIGH);
  return i;
}

void bufferRead ( uint8_t *b )
{
  spiramRead ( readinx, b, CHUNKSIZE ) ;            // return next chunk
  readinx = ( readinx + 1 ) % SRAM_CH_SIZE ;        // Increment and wrap if necessary
  chcount-- ;                                       // Count is now one less
}


//******************************************************************************************
//                            B U F F E R R E S E T                                        *
//******************************************************************************************
void bufferReset()
{
  readinx = 0 ;                                     // Reset ringbuffer administration
  writeinx = 0 ;
  chcount = 0 ;
}

//******************************************************************************************
//                                S P I R A M S E T U P                                    *
//******************************************************************************************

void spiramtest()
{
  uint8_t  i ;
  uint32_t b = random ( 100 ) ;                     // Random fill value
  uint8_t chunk[32] ;

  for ( i = 0 ; i < sizeof(chunk) ; i++ )           // Fill chunk
  {
    chunk[i] = b ;
  }
  Serial.printf ( "Chunk filled with value 0x%02X\n", b ) ;
  for ( i = 0 ; i < 8 ; i++ )                       // Write to SPI ram
  {
    if ( spaceAvailable() )
    {
      bufferWrite ( chunk ) ;
    }
    else
    {
      Serial.printf ( "No more space!\n" ) ;
    }
  }

  Serial.printf ( "Filled space now is %d chunks\n", dataAvailable() ) ;
  Serial.printf ( "Free space now is %d chunks\n", getFreeBufferSpace() ) ;

  for ( i = 0 ; i < 8 ; i++ )                       // Read from SPI ram
  {
    memset ( chunk, 0, sizeof(chunk) ) ;            // Clear buffer
    if ( dataAvailable() )
    {
      bufferRead ( chunk ) ;
    }
    else
    {
      Serial.printf ( "No data avavilabel!\n" ) ;
    }
  }
  Serial.printf ( "SPI RAM test complete!\n" ) ;
}

void spiramSetup()
{
  SPI.begin();
  SPI.setBitOrder  ( MSBFIRST ) ;
  SPI.setDataMode  ( SPI_MODE0 ) ;
  SPI.setFrequency ( SRAM_FREQ ) ;

  pinMode ( SRAM_CS, OUTPUT ) ;
  digitalWrite ( SRAM_CS, HIGH ) ;
  delay(50);
  digitalWrite ( SRAM_CS, LOW ) ;
  delay(50);
  digitalWrite ( SRAM_CS, HIGH ) ;

  bufferReset() ;
}
