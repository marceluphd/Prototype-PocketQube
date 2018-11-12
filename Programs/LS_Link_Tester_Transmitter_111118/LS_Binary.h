//LS_Binary.h
/*
*******************************************************************************************************************************
  Easy Build LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 11/11/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  This program writes and reads variables to and from the buffer address passed. It is normally used together with the LoRa
  TX and RX buffers.


  To Do:

*******************************************************************************************************************************
*/

void Write_Byte(uint8_t addr, uint8_t x, uint8_t localbuff[])
{
  localbuff[addr] = x;
}


void Write_Int(uint8_t addr, int x,  uint8_t localbuff[])
{
  localbuff[addr] = lowByte(x);
  localbuff[addr + 1] = highByte(x);
}


void Write_UInt(uint8_t addr, uint16_t  x,  uint8_t localbuff[])
{
  localbuff[addr] = lowByte(x);
  localbuff[addr + 1] = highByte(x);
}


void Write_Float(uint8_t addr, float x,  uint8_t localbuff[])
{
  uint8_t i, j;

  union
  {
    uint8_t b[4];
    float f;
  } data;
  data.f = x;

  for (i = 0; i < 4; i++)
  {
    j = data.b[i];
    localbuff[addr + i] = j;
  }
}


uint8_t Read_Byte(uint8_t addr, uint8_t localbuff[])
{
  return localbuff[addr];
}


float Read_Float(uint8_t addr, uint8_t localbuff[])
{
  uint8_t i, j;

  union
  {
    uint8_t b[4];
    float f;
  } readdata;

  for (i = 0; i < 4; i++)
  {
    j = localbuff[addr + i];
    readdata.b[i] = j;
  }
  return readdata.f;
}


int Read_Int(uint8_t addr, uint8_t localbuff[])
{
  uint8_t lowbyte, highbyte;
  lowbyte = localbuff[addr];
  highbyte = localbuff[addr + 1];
  return (lowbyte + (highbyte * 256));
}


uint16_t  Read_UInt(uint8_t addr, uint8_t localbuff[])
{
  uint8_t lowbyte, highbyte;
  lowbyte = localbuff[addr];
  highbyte = localbuff[addr + 1];
  return (lowbyte + (highbyte * 256));
}
