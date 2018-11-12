//Link_Tester_Transmitter_111118.ino


#define programname "LS_Link_Tester_Transmitter_111118"
#define programversion "V1.0"
#define dateproduced "11/11/18"
#define aurthorname "Stuart Robinson"

#include <Arduino.h>
#include "LS_Program_Definitions.h"

/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 11/11/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  Changes:
  

  To do:
  Check this len = sizeof(lora_TXBUFF); not causing overun when TXBUFF - 256
  


******************************************************************************************************
*/


//#include "LoRaSAT_Board_Definitions.h"        // pin definitions for LoRaSAT boards        
#include "Pro_Mini_Mikrobus_Shield_Board_Definitions.h"

/*
******************************************************************************************************
  Definitions for packet types used in this program
******************************************************************************************************
*/

/*
const uint8_t ACK = 'A';                   //Acknowledge
const uint8_t ClearToSendCommand = 'c';    //clear to send a packet
const uint8_t LocationCSVPacket = 'S';     //Short loacation packet in CSV ASCII format
const uint8_t LocationBinaryPacket = 's';  //Short location packet in binary format
const uint8_t Testpacket = 'T';            //Test packet
const uint8_t LMLCSVPacket = '8';          //short LML payload; lat,lon,alt in CSV ASCII format
const uint8_t TestMode1 = '1';             //used to switch to Testmode1 settings
const uint8_t TestMode2 = '2';             //used to switch to Testmode2 settings
const uint8_t Broadcast = '*';             //Broadcast address
const uint8_t HABPacket = '$';             //HAB Style location payload in CSV ASCII format
*/

//Global Program Variables

uint16_t seq = 0;
uint16_t resets = 0;
uint16_t batch = 65535;
uint16_t testloop = 0;

uint8_t  keypress;
uint8_t  modenumber;
uint8_t  ramc_ThisNode;
uint8_t  lora_TestPower = 10;                //is also start up tone power

#include <SPI.h>
#include "LS_Test_Settings.h"
#include "LS_LoRa.h"
#include "LS_Binary.h"
#include "LS_FSK_RTTY.h"

#ifdef UseBatchNumber
#include Memory_Library                  //include previously defined Memory Library
#endif

void loop()
{

  testloop++;

  Serial.print(F("Test Loop "));
  Serial.println(testloop);
  #ifdef UseBatchNumber
  Serial.print(F("batch "));
  Serial.println(batch);
  #endif
  Serial.println();
  Serial.flush();

  Serial.println(F("Loop Start Tone "));
  Serial.flush();

  lora_Tone(500, LoopStartTone_lengthmS, start_power);       //Transmit a pseudo FM tone
  delay(250);

  init_TXLoRaTest();                                         //setup for test mode 1

  Serial.println("Start Mode1 Packets");
  Serial.println();
  for (lora_TestPower = start_power; lora_TestPower >= end_power; lora_TestPower--)
  {
    Serial.print(lora_TestPower);
    Serial.println("dBm  Packets");

#ifdef SendTestpacket
    lora_Custom();
    Serial.print("Send Test Packet ");
    Serial.flush();
    Send_Test_Packet('1');
    Serial.println();
    delay(packet_delay);
#endif

#ifdef SendBinaryLocation
    lora_Custom();
    Serial.print("Send Binary Location Packet ");
    Serial.flush();
    Send_Binary_Location_Packet();
    Serial.println();
    delay(packet_delay);
#endif

#ifdef SendLoRaSATPacket
    lora_Custom();
    Serial.print("Send LoRaSAT Packet ");
    Serial.flush();
    Send_LoRaSAT_Packet();
    Serial.println();
    delay(packet_delay);
#endif
    Serial.println();
  }

  Serial.println("Finished Test Packets");

  Serial.println();
  delay(mode_delaymS);


}


void Send_LoRaSAT_Packet()
{
  uint8_t  Count;
  Count = buildLoRaSATPayload();
  lora_TXEnd = Count;
  PrintPacket();
  Serial.print(F(" "));
  digitalWrite(LED1, HIGH);
  lora_Send(0, Count, HABPacket, Broadcast, ThisNode, 10, lora_TestPower, NoStrip);   //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
  digitalWrite(LED1, LOW);
  Serial.print(F("TX Time "));
  Serial.print(lora_TXTime);
  Serial.print("mS ");
}



uint8_t buildLoRaSATPayload()
{
  //build the long tracker payload
  uint16_t index, index2, hours, mins, seconds, alt, sats, volts, internaltemperature, resets;
  uint16_t ramc_Current_config, TRStatus, runmAhr, GPSfixtime;
  uint16_t CRC, len;
  uint8_t  Count;
  uint16_t  max_length = 256;
  float calibration;
  char LatArray[10], LonArray[10], CalibrationArray[10];            //dtostrf function needs char arrays
  uint8_t node[2];
  seq++;
  hours = 10;
  mins = 10;
  seconds = 10;
  dtostrf(TestLatitude, 7, 5, LatArray);                     
  dtostrf(TestLongitude, 7, 5, LonArray);
  alt = TestAltitude;
  sats = 5;
  volts = 3999;
  internaltemperature = -10;
  resets = 11;
  ramc_Current_config = 0x02;
  TRStatus = 0x81;
  runmAhr = 45;
  calibration = 2000;
  dtostrf(calibration, 3, 1, CalibrationArray);
  GPSfixtime = 4.5;

  len = sizeof(lora_TXBUFF);
  memset(lora_TXBUFF, 0, len);                                 //clear array to 0s

  node[0] = ramc_ThisNode;
  node[1] = 0;

  Count = snprintf((char*) lora_TXBUFF,
                   max_length,
                   "$$$$%s%s,%d,%02d:%02d:%02d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d",
                   Flight_ID,
                   node,
                   seq,
                   hours,
                   mins,
                   seconds,
                   LatArray,
                   LonArray,
                   alt,
                   sats,
                   volts,
                   internaltemperature,
                   resets,
                   ramc_Current_config,
                   TRStatus,
                   runmAhr,
                   CalibrationArray,
                   GPSfixtime
                  );

  CRC = 0xffff;                                               //start value for CRC16

  for (index = 4; index < Count; index++)                     //element 4 is first character after $$$$ at start
  {
    CRC ^= (((unsigned int)lora_TXBUFF[index]) << 8);
    for (index2 = 0; index2 < 8; index2++)
    {
      if (CRC & 0x8000)
        CRC = (CRC << 1) ^ 0x1021;
      else
        CRC <<= 1;
    }
  }

  lora_TXBUFF[Count++] = '*';
  lora_TXBUFF[Count++] = Hex((CRC >> 12) & 15);                //add the checksum bytes to the end
  lora_TXBUFF[Count++] = Hex((CRC >> 8) & 15);
  lora_TXBUFF[Count++] = Hex((CRC >> 4) & 15);
  lora_TXBUFF[Count] = Hex(CRC & 15);
  return Count;
}


char Hex(uint8_t  lchar)
{
  //used in CRC calculation in buildHABPacket
  char Table[] = "0123456789ABCDEF";
  return Table[lchar];
}


void PrintPacket()
{
  uint8_t  index;
  for (index = 0; index <= lora_TXEnd; index++)
  {
    Serial.write(lora_TXBUFF[index]);
  }
}


void Send_Test1Mode_Packet()
{
  //causes RX to switch mode
  Serial.println(F("Send_Test1Mode_Packet()"));
  lora_TXBUFF[0] = '1';
  //lora_TXBUFF[1] = lowByte(batch);
  //lora_TXBUFF[2] = highByte(batch);
  digitalWrite(LED1, HIGH);
  lora_Send(0, 0, TestMode1, Broadcast, ThisNode, 10, 17, NoStrip);
  digitalWrite(LED1, LOW);
  delay(1000);                                                       //leave time for receiver to print running totals
}


void Send_Binary_Location_Packet()
{
  Serial.print(F(" "));
  Serial.print(TestLatitude, 6);
  Serial.print(F(" "));
  Serial.print(TestLongitude, 6);
  Serial.print(F(" "));
  Serial.print(TestAltitude);
  Serial.print(F(" "));
  Serial.print(config_byte);
  Serial.print(F(" "));
  Write_Float(0, TestLatitude, lora_TXBUFF);
  Write_Float(4, TestLongitude, lora_TXBUFF);
  Write_UInt(8, TestAltitude, lora_TXBUFF);
  Write_Byte(10, config_byte, lora_TXBUFF);
  lora_TXEnd = 10;
  digitalWrite(LED1, HIGH);
  lora_Send(0, lora_TXEnd, LocationBinaryPacket, Broadcast, ThisNode, 10, lora_TestPower, NoStrip);
  digitalWrite(LED1, LOW);
  Serial.print(F("TX Time "));
  Serial.print(lora_TXTime);
  Serial.print("mS ");
}



void Send_Test_Packet(char lmode)
{
  //build and send test packet

  if (lora_TestPower > 9)
  {
    lora_TXBUFF[0] = '1';
    lora_TXBUFF[1] = ((lora_TestPower - 10) + 0x30);
  }
  else
  {
    lora_TXBUFF[0] = '0';
    lora_TXBUFF[1] = (lora_TestPower + 0x30);
    
  }

  lora_TXBUFF[2] = lmode;

  lora_TXEnd = 2;
  PrintPacket();
  
  Serial.print(F(" "));
  digitalWrite(LED1, HIGH);
  lora_Send(0, lora_TXEnd, Testpacket, Broadcast, ThisNode, 10, lora_TestPower, NoStrip);
  digitalWrite(LED1, LOW);
  Serial.print(F("TX Time "));
  Serial.print(lora_TXTime);
  Serial.print("mS ");
}



void Led_FlashStart()
{
  uint8_t  index;
  for (index = 0; index <= 4; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void System_Error()
{
  while (1)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void init_TXLoRaTest()
{
  //setup for testmode1
  Serial.println(F("init_TXLoRaTest1()"));
  float freq_temp;
  lora_Setup();
  lora_SetFreq(LoRa_Frequency, CalibrationOffset);
  freq_temp = lora_GetFreq();
  Serial.print(F("Set to Frequency "));
  Serial.print(freq_temp, 3);
  Serial.println(F("Mhz"));
  lora_SetModem2(LoRa_Test_Bandwidth, LoRa_Test_SpreadingFactor, LoRa_Test_CodeRate, Explicit);	//setup the LoRa modem parameters for test
}

/*
void lora_Setup()
{
  //initialise LoRa device registers and check its responding
  lora_ResetDev();        //clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);   //RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x2B);    //RegOcp
  lora_Write(lora_RegLna, 0x23);    //RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF); //RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);  //RegPreambleLsb, default

  if (!lora_CheckDevice())
  {
    Serial.println();
    Serial.print(F("LoRa Device Error"));
    Serial.println();
  }
}
*/

void lora_Custom()
{
  //used to have a custom set of LoRa register settings for each test mode
}

/*
void lora_RXONLoRa()
{
  //puts LoRa device into listen mode and receives packet exits with packet in array lora_RXBUFF[]
  lora_RXPacketL = 0;
  lora_RXPacketType = 0;
  lora_RXDestination = 0;
  lora_RXSource = 0;
  lora_RXStart = 0;
  lora_RXEnd = 0;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegFifoRxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegIrqFlagsMask, 0x9F);                //only allow rxdone and crc error
  lora_Write(lora_RegOpMode, 0x8D);
  lora_BackGroundRSSI = lora_Read(lora_RegRssiValue);    //get the background noise level now receiver is on
}
*/

void setup()
{
  pinMode(LED1, OUTPUT);		                  //for PCB LED
  Led_FlashStart();

  Serial.begin(115200);                        //setup Serial console ouput
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(dateproduced));
  Serial.println(F(aurthorname));
  Serial.print(F("Node "));
  Serial.print(ThisNode);
  Serial.println();

  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  pinMode(lora_NReset, OUTPUT);			           //LoRa Device reset line
  pinMode (lora_NSS, OUTPUT);			             //LoRa Device select line
  digitalWrite(lora_NSS, HIGH);
  digitalWrite(lora_NReset, HIGH);

  ramc_ThisNode = ThisNode;

  if (lora_CheckDevice() == true)
  {
    init_TXLoRaTest();
    digitalWrite(LED1, HIGH);
    Serial.print(F("Start Test Tone "));
    lora_Tone(1000, 1000, 2);                   //Transmit an FM tone, 1000hz, 1000ms, 2dBm
    lora_TXOFF();                               //so off time is recorded correctly
    digitalWrite(LED1, LOW);
    Serial.println(F(" - Finished"));
    Serial.println();
  }
  else
  {
    Serial.println(F("LoRa Device Error"));
    System_Error();
  }

  lora_Print();
  Serial.println();

  #ifdef UseBatchNumber
  batch = Memory_ReadUInt(addr_ResetCount);
  batch++;
  Memory_WriteUInt(addr_ResetCount, batch);
  Serial.print(F("Batch "));
  Serial.println(batch);
  #endif

}
