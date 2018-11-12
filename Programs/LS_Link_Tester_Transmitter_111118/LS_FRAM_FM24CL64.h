//I2CFRAM_FM24CL64.h
/*
*******************************************************************************************************************************
  Easy Build LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 18/08/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  This program provides a standard set of memory read and write routines for an I2C FRAM. Tested with Fujitsu MB85RC16PNF.

  To Do:


*******************************************************************************************************************************
*/

#include <Wire.h>
const int16_t Memory_I2C_Addr = 0x50;                     //I2C address of FM24CL64 FRAM

void Memory_Start()
{
  Wire.begin();
}

void Memory_End()
{
  //left empty for future use
}


/***************************************************************************
  Write Routines
 ***************************************************************************
 */

void Memory_WriteByte(uint16_t  addr, uint8_t x)
{
  uint8_t msb_addr = highByte(addr);            
  uint8_t lsb_addr = lowByte(addr);

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.write(x);
  Wire.endTransmission();
}


void Memory_WriteFloat(uint16_t  addr, float x)
{
  uint8_t index, val;
  uint8_t msb_addr = highByte(addr);            
  uint8_t lsb_addr = lowByte(addr);

  union
  {
    uint8_t b[4];
    float f;
  } data;

  data.f = x;

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);

  for (index = 0; index < 4; index++)
  {
    val = data.b[index];
    Wire.write(val);                                    //write the data
  }

  Wire.endTransmission();
}


void Memory_WriteInt(uint16_t  addr, int x)
{

  uint8_t msb_addr = highByte(addr);              
  uint8_t lsb_addr = lowByte(addr);
  uint8_t msb_data = highByte(x);
  uint8_t lsb_data = lowByte(x);


  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.write(lsb_data);
  Wire.write(msb_data);
  Wire.endTransmission();
}


void Memory_WriteUInt(uint16_t  addr, uint16_t  x)
{

  uint8_t msb_addr = highByte(addr);              
  uint8_t lsb_addr = lowByte(addr);
  uint8_t msb_data = highByte(x);
  uint8_t lsb_data = lowByte(x);


  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.write(lsb_data);
  Wire.write(msb_data);
  Wire.endTransmission();
}


void Memory_WriteULong(uint16_t  addr, uint32_t x)
{
  uint8_t index, val;
  uint8_t msb_addr = highByte(addr);            
  uint8_t lsb_addr = lowByte(addr);

  union
  {
    uint8_t b[4];
    uint32_t f;
  } data;

  data.f = x;

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);

  for (index = 0; index < 4; index++)
  {
    val = data.b[index];
    Wire.write(val);                                   //write the data
  }

  Wire.endTransmission();
}



/***************************************************************************
  Read Routines
 **************************************************************************
 */

uint8_t Memory_ReadByte(uint16_t  addr)
{
  uint8_t data;
  uint8_t msb_addr = highByte(addr);            
  uint8_t lsb_addr = lowByte(addr);

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.endTransmission();
  Wire.requestFrom((Memory_I2C_Addr), 1);
  data = Wire.read();
  return data;
}


int Memory_ReadInt(uint16_t  addr)
{

  uint8_t lsb_data, msb_data;

  uint8_t msb_addr = highByte(addr);
  uint8_t lsb_addr = lowByte(addr);


  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.endTransmission();
  Wire.requestFrom(Memory_I2C_Addr, 2);
  lsb_data = Wire.read();
  msb_data = Wire.read();

  return (lsb_data + (msb_data * 256));
}


uint16_t  Memory_ReadUInt(uint16_t  addr)
{

  uint8_t lsb_data, msb_data;

  uint8_t msb_addr = highByte(addr);
  uint8_t lsb_addr = lowByte(addr);


  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.endTransmission();
  Wire.requestFrom(Memory_I2C_Addr, 2);
  lsb_data = Wire.read();
  msb_data = Wire.read();

  return (lsb_data + (msb_data * 256));
}




float Memory_ReadFloat(uint16_t  addr)
{
  uint8_t val;

  uint8_t msb_addr = highByte(addr);
  uint8_t lsb_addr = lowByte(addr);

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.endTransmission();
  Wire.requestFrom(Memory_I2C_Addr, 4);

  union
  {
    uint8_t b[4];
    float f;
  } readdata;


  for (int16_t index = 0; index < 4; index++)
  {
    val = Wire.read();          // read the byte
    readdata.b[index] = val;
  }
  return readdata.f;
}

unsigned long Memory_ReadULong(uint16_t  addr)
{
  uint8_t val;
  uint8_t msb_addr = highByte(addr);
  uint8_t lsb_addr = lowByte(addr);

  Wire.beginTransmission(Memory_I2C_Addr);
  Wire.write(msb_addr);
  Wire.write(lsb_addr);
  Wire.endTransmission();
  Wire.requestFrom(Memory_I2C_Addr, 4);

  union
  {
    uint8_t b[4];
    uint32_t f;
  } readdata;

  for (int16_t index = 0; index < 4; index++)
  {
    val = Wire.read();          // read the byte
    readdata.b[index] = val;
  }

  return readdata.f;
}




/***************************************************************************
  Start of general purpose memory routines
***************************************************************************/

uint16_t  Memory_CRC(uint16_t  startaddr, uint16_t  endaddr)
{
  uint16_t  i, CRC;

  CRC = 0xffff;                                              //start value for CRC16
  uint8_t j;

  for (i = startaddr; i <= endaddr; i++)                     //element 4 is first character after $$$$ at start
  {
    CRC ^= ((uint16_t)Memory_ReadByte(i) << 8);
    for (j = 0; j < 8; j++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }
  return CRC;

}


void Print_Memory(uint16_t start_addr, uint16_t end_addr)
{
  //print the contents of Memory
   
  uint8_t value;
  
  for (uint16_t a = start_addr; a <= end_addr; a++) 
  {
    value = Memory_ReadByte(a);
    if ((a % 16) == 0) 
    {
      Serial.println();
      Serial.print(F("0x")); 
      if (a < 0x10) 
      {
       Serial.print('0');
      }
      Serial.print(a, HEX); 
      Serial.print(F(": "));
    }
    if (value < 0x10) 
      Serial.print('0');
    Serial.print(value, HEX); 
    Serial.print(F(" "));
  }
  Serial.println();
}



void Memory_Set(uint16_t  startaddr, uint16_t  endaddr, uint8_t lval)
{
  uint16_t  i;
  
  for (i = startaddr; i <= endaddr; i++)
  {
    Memory_WriteByte(i, lval);
  }
}
