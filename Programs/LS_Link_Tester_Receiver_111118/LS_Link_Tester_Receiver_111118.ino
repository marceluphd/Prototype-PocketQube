#define programname "Link_Tester_Receiver_270818"
#define programversion "V2.8"
#define aurthorname "Stuart Robinson"
#include <Arduino.h>
#define LoRa_Device_in_MB1
/*
**************************************************************************************************

  LoRaTracker Programs for Arduino

  Copyright of the author Stuart Robinson - 27/08/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without
  the explicit permission of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
  intended purpose and free from errors.

  Changes:
  150318 - Added option to drive a 20x4 LCD display via a PCF8574 backpack
  150318 - Added option to log output to an SD card
  150418 - Added relay option, receiver can send received test packets out on a sepearate frequency so that
  they can be logged on a receiver that is remote from the test
  150418 - Added option for receiver to be the relay endpoint, receiving realyed packets from a remote receiver
  150418 - Added option for SD1306 OLED display
  110518 - Chnaged batch numbering of logs to use receiver generated batch number
  110518 - Added switch option to cause a new log file to be started
  140518 - Added detection of packets not recognised
  150518 - Added count of not recognised packets
  010618 - Moved lora_SendRXBuffer() into LoRa_Basic.h
  010618 - Tested as working for RelayEndpoint
  010618 - Link_Tester_Receiver_140518_V4 saved as Link_Tester_Receiver_010618

  To do:
  Add option to use LoRaTracker display backpack
  Add select relay endpoint by pressing switch at power up
  Remove double log start on reset.
  Remove logFile.flush() commands to be compatible with ESP32, use logFile.open() instead.




******************************************************************************************************
*/



/*
******************************************************************************************************
  Definitions for packet types used in this program
******************************************************************************************************
*/

const char ACK = 'A';                   //Acknowledge
const char ClearToSendCommand = 'c';    //clear to send a packet
const char LocationBinaryPacket = 's';  //Short location packet in binary format
const char Testpacket = 'T';            //Test packet
const char TestMode1 = '1';             //used to switch to Testmode1 settings
const char Broadcast = '*';             //Broadcast address
const char HABPacket = '$';             //HAB Style location payload in CSV ASCII format
const char InfoRelay = '5';             //used to relay screen SNR and RSSI in ping test.

byte modenumber;
byte keypress;
byte lora_TestPower = 2;                //is also start up tone power
boolean SD_Found = false;               //set if SD card found at program startup
boolean RelayEndpoint;                  //used to check record if relay endpoint is active
boolean Int_Guard = true;               //used to prevent multiple interrupts, when true interrupts are ignored
boolean Open_New_Log = false;
uint32_t Mode1_Packet_Count = 0;
uint32_t Mode1_Cycles = 0;
uint16_t Not_Recognised_Count;
uint32_t CRC_Error_Count = 0;

int8_t Saved_SNR;
int8_t Saved_RSSI;

uint16_t lora_Test1Count[20];                   //buffer where counts of received packets are stored
uint16_t Current_Batch = 0;

float TRLat;                                    //tracker transmitter co-ordinates
float TRLon;
uint16_t TRAlt;

#include "Test_Settings.h"

#include Board_Definition                         //include previously defined board definition file

#ifdef UseDisplay
#include Display_Library                          //include previously defined Display Library
#endif



#include <SPI.h>
#include "LoRa_Test.h"
#include "Binary_Test.h"
#include <Wire.h>

#ifdef UseSD

#ifdef UseESP32
#include <mySD.h>                            //https://github.com/nhatuan84/esp32-micro-sdcard/blob/master/mySD.h
File logFile;
#endif

#ifndef UseESP32
#include <SdFat.h>                           //https://github.com/greiman/SdFat
SdFat SD;
File logFile;
#endif

#endif

char file_name[] = "xxxxx.txt";                // prototype file name for SD
char extension[] = ".txt";  // sometimes the extension gets modified

void loop()
{
#ifdef UseSD

  if (Open_New_Log)
  {
    Serial.println(F("Request for new log"));
    Open_New_Log = false;
    clear_Counts();
    Current_Batch++;
    setup_SDLOG(Current_Batch);
    SD_WriteCurrentLoRaSettings();
    Int_Guard = true;                                 //stop interrupts till end of packet receive routine

  }
#endif

  checkforpacket();
}


void checkforpacket()
{
  //check if a packet has been received
  byte lora_Ltemp;

  lora_Ltemp = lora_readRXready2();                   //this reads the DIO0 pin which creates less EMI than polling the LoRa device registers.

  if (lora_Ltemp == 64)
  {
    lora_RXOFF();                                     //stop more packets comming in
    digitalWrite(LED1, HIGH);
    Serial.print(F("RX "));
    lora_ReadPacket();


#ifdef EnableRelay
    init_LoRaRelay();
    lora_RXBUFF[lora_RXPacketL - 3] = lora_PacketSNR;
    lora_RXBUFF[lora_RXPacketL - 2] = lora_PacketRSSI;
    lora_SendRXBuffer(0, (lora_RXPacketL - 2), lora_RXPacketType, lora_RXDestination, ThisNode, 10, relay_Power, 0);
    Serial.print(F("(Relayed) "));
    init_RXLoRaTest1();
#endif

    Saved_SNR =  lora_PacketSNR;
    Saved_RSSI =  lora_PacketRSSI;

#ifdef RelayEndpoint
    lora_PacketSNR = lora_RXBUFF[lora_RXPacketL - 5];
    lora_PacketRSSI = lora_RXBUFF[lora_RXPacketL - 4];
#endif

    lora_ReceptionInfo();
    Serial.print(F("  "));
    lora_AddressInfo();

#ifdef UseSD
    lora_ReceptionInfoSD2(lora_PacketSNR, lora_PacketRSSI);
    lora_AddressInfoSD();
#endif

    Serial.print(F("  "));
    processPacket();
    lora_RXONLoRa();                                 //ready for next
    digitalWrite(LED1, LOW);
  }

  if (lora_Ltemp == 96)
  {
    CRC_Error_Count++;
    Serial.println(F("CRC Error"));
#ifdef UseSD
    logFile.println(F("CRC Error"));
#endif
    lora_RXONLoRa();                                //ready for next
  }

  Int_Guard = false;                                //clear the guard variable

}



void processPacket()
{
  byte lTXpower;
  uint16_t i, tempuint;
  float tempfloat1, tempfloat2;
  byte tempbyte;
  uint32_t uptime;

  if (lora_RXPacketType == InfoRelay)
  {
    tempfloat1 = Read_Float(0, lora_RXBUFF);
    tempbyte = Read_Byte(4, lora_RXBUFF);

    if (tempbyte == 127)
    {
      Serial.println();
      Serial.print(F("Signal Relay, RSSI "));
      Serial.print(tempfloat1, 1);
      Serial.println(F("dBm"));

#ifdef UseDisplay
      Display_Clear();
      Display_SetCurPos(0, 0);
      Serial.print(F("Signal Relay"));
      Display_SetCurPos(0, 1);
      disp.print(F("RSSI "));
      disp.print(tempfloat1, 1);
      disp.print(F(" dBm  "));
      Display_Update();
#endif
    }
    else
    {
      Serial.print(F("Packet Relay, RSSI "));
      Serial.print(tempfloat1, 1);
      Serial.print(F("dBm"));
      Serial.print(F(", SNR "));
      Serial.print(tempbyte);
      Serial.println(F("dB"));

#ifdef UseDisplay
      Display_Clear();
      Display_SetCurPos(0, 0);
      disp.print(F("Packet Relay"));
      Display_SetCurPos(0, 1);
      disp.print(F("RSSI "));
      disp.print(tempfloat1, 0);
      disp.print(F("dBm"));
      Display_SetCurPos(0, 2);
      disp.print(F("SNR "));
      disp.print(tempbyte);
      disp.print(F("dB"));
      writescreen_PacketsBatch();
      Display_Update();
#endif
    }
    return;
  }

  if (lora_RXPacketType == Testpacket)
  {
    lTXpower = ((lora_RXBUFF[0] - 48) * 10) +  (lora_RXBUFF[1] - 48);

#ifdef UseDisplay
    writescreen_TestPacket();
    writescreen_SNR();
    writescreen_RSSI();
    writescreen_PacketsBatch();
    Display_Update();
#endif

    if (lora_RXBUFF[2] == '1')
    {
      if (Mode1_Cycles > 0)
      {
        Mode1_Packet_Count++;
        i = lora_Test1Count[lTXpower];
        i++;
        lora_Test1Count[lTXpower] = i;
      }

    }

    Serial.print(F(" ("));
    Serial.print(lTXpower);
    Serial.print(F("dBm) "));
    Serial.println();

#ifdef UseSD
    logFile.print(F(" ("));
    logFile.print(lTXpower);
    logFile.print(F("dBm) "));
    logFile.println();
#endif
    return;
  }

  if (lora_RXPacketType == LocationBinaryPacket)
  {
    tempfloat1 = Read_Float(0, lora_RXBUFF);
    Serial.print(tempfloat1, 5);
    Serial.print(F("  "));
    tempfloat2 = Read_Float(4, lora_RXBUFF);
    Serial.print(tempfloat2, 5);
    Serial.print(F("  "));
    tempuint = Read_UInt(8, lora_RXBUFF);
    Serial.print(tempuint);
    Serial.print(F("  "));
    tempbyte = Read_Byte(10, lora_RXBUFF);
    Serial.print(tempbyte);
    Serial.println();

#ifdef UseDisplay
    writescreen_LocationBinaryPacket(tempfloat1, tempfloat2, tempuint);
    writescreen_SNR();
    writescreen_RSSI();
    writescreen_PacketsBatch();
    Display_Update();
#endif

#ifdef UseSD
    tempfloat1 = Read_Float(0, lora_RXBUFF);
    logFile.print(tempfloat1, 5);
    logFile.print(F("  "));
    tempfloat2 = Read_Float(4, lora_RXBUFF);
    logFile.print(tempfloat2, 5);
    logFile.print(F("  "));
    tempuint = Read_UInt(8, lora_RXBUFF);
    logFile.print(tempuint);
    logFile.print(F("  "));
    tempbyte = Read_Byte(10, lora_RXBUFF);
    logFile.print(tempbyte);
    logFile.println();
#endif

    return;
  }

  if (lora_RXPacketType == HABPacket)
  {
    lora_RXBuffPrint(0);
    Serial.println();
    extract_HABPacket(lora_RXStart);

#ifdef UseSD
    if (lora_RXPacketType == HABPacket)
    {
      lora_RXBuffPrintSD(0);
      logFile.println();
    }
#endif

#ifdef UseDisplay
    writescreen_HABPacket(TRLat, TRLon, TRAlt);
    writescreen_SNR();
    writescreen_RSSI();
    writescreen_PacketsBatch();
    Display_Update();
#endif
    return;
  }

  if (lora_RXPacketType == TestMode1)
  {
    //this is a command to switch to TestMode1

    Serial.println();
    Serial.println();
    Serial.println(F("Switch Mode1 "));

#ifdef UseSD
    logFile.println();
    logFile.println();
    logFile.print(F("Switch Mode1 - "));
    logFile.println(Current_Batch);
    logFile.println();
#endif

    if (Mode1_Cycles > 0)
    {
      print_Test1Count();
    }

    Serial.println();
    Mode1_Cycles++;
    return;
  }



  //if we get to here the packet type is not recognised.

  Not_Recognised_Count++;
  uptime = millis() / 1000;

  Serial.println();
  Serial.print(F("Packet? "));
  Serial.println(lora_RXPacketType);
  Serial.print(F("Uptime seconds "));
  Serial.println(uptime);
  Serial.println();

#ifdef UseSD
  logFile.println();
  logFile.println();
  logFile.print(F("Packet? "));
  logFile.println(lora_RXPacketType);
  logFile.print(F("Uptime seconds "));
  logFile.println(uptime);
  logFile.println();
  logFile.flush();                                   //close the file on SD to ensure that not recognised stuff is written, its important
  logFile = SD.open(file_name, FILE_WRITE);          //open log ready for more stuff
#endif


#ifdef UseDisplay
  digitalWrite(BUZZ, HIGH);
  Display_SetCurPos(0, 0);
  disp.print(F("                    "));             //make sure first line is cleared of text
  Display_SetCurPos(0, 0);
  disp.print(F("Packet? "));
  disp.print(lora_RXPacketType);
  disp.print(F("  Num "));
  disp.print(Not_Recognised_Count);
  writescreen_SNR2(Saved_SNR);
  writescreen_RSSI2(Saved_RSSI);
  Display_Update();
  digitalWrite(BUZZ, LOW);
#endif


#ifndef RelayEndpoint
  init_RXLoRaTest1();                          //after relay go back to normal receive setup if not Relay Endpoint
#endif

}


void clear_Counts()
{
  byte i;
  Mode1_Cycles = 0;
  Mode1_Packet_Count = 0;
  Not_Recognised_Count = 0;
  lora_RXpacketCount = 0;
  Serial.println(F("Clear packet counts"));
  for (i = 17; i >= 2; i--)
  {
    lora_Test1Count[i] = 0;
  }
}


void print_Test1Count()
{
  //prints running totals of the power of received packets
  byte i;
  unsigned long j;

#ifdef UseSD
  //logFile.println();
  logFile.print(F("Mode1 Test Packets "));
  logFile.println(Mode1_Packet_Count);
  logFile.print(F("Mode1 Test Cycles "));
  logFile.println(Mode1_Cycles);
  logFile.print(F("Total Packets "));
  logFile.println(lora_RXpacketCount);
  logFile.print(F("CRC Errors "));
  logFile.println(CRC_Error_Count);
  logFile.print(F("Not recognised packets "));
  logFile.println(Not_Recognised_Count);
  logFile.print(F("Batch "));
  logFile.println(Current_Batch);

  for (i = 17; i >= 2; i--)
  {
    logFile.print(i);
    logFile.print(F("dBm,"));
    j = lora_Test1Count[i];
    logFile.print(j);
    logFile.print(F("  "));
  }
  logFile.println();

  logFile.print(F("CSV"));
  for (i = 17; i >= 2; i--)
  {
    logFile.print(F(","));
    j = lora_Test1Count[i];
    logFile.print(j);
  }
  logFile.println();
  logFile.println();
  logFile.flush();
#endif

  Serial.print(F("Mode1 Test Packets "));
  Serial.println(Mode1_Packet_Count);
  Serial.print(F("Mode1 Test Cycles "));
  Serial.println(Mode1_Cycles);
  Serial.print(F("Packets "));
  Serial.println(lora_RXpacketCount);
  Serial.print(F("CRC Errors "));
  Serial.println(CRC_Error_Count);
  Serial.print(F("Not recognised packets "));
  Serial.println(Not_Recognised_Count);
  Serial.print(F("Batch "));
  Serial.println(Current_Batch);


  for (i = 17; i >= 2; i--)
  {
    Serial.print(i);
    Serial.print(F("dBm,"));
    j = lora_Test1Count[i];
    Serial.print(j);
    Serial.print(F("  "));
  }

  Serial.println();

  Serial.print(F("CSV"));
  for (i = 17; i >= 2; i--)
  {
    Serial.print(F(","));
    j = lora_Test1Count[i];
    Serial.print(j);
  }
  Serial.println();
}


void extract_HABPacket(byte passedRXStart)
{
  //extracts data from received HAB packets where first fields are lat,lon,alt.
  byte tempbyte;
  byte savedRXStart;

  savedRXStart = lora_RXStart;                            //save current value of lora_RXStart
  lora_RXStart = passedRXStart;                           //use lora_RXStart as its global

  do
  {
    tempbyte =  lora_RXBUFF[lora_RXStart++];
  }
  while ( tempbyte == '$');
  lora_RXStart--;

  lora_RXStart = next_Comma(lora_RXStart);
  lora_RXStart = next_Comma(lora_RXStart);
  lora_RXStart = next_Comma(lora_RXStart);


  TRLat = extract_Float();             //Lat
  TRLon = extract_Float();             //Lon
  TRAlt = extract_Uint();              //Alt

  lora_RXStart = next_Comma(lora_RXStart);
  lora_RXStart = next_Comma(lora_RXStart);

  lora_RXStart = savedRXStart;                              //restore lora_RXStart, just in case
}


float extract_Float()
{
  //extracts a float in ASCII format from buffer
  char temp[12];
  byte tempptr = 0;
  byte bufferdata;
  float tempfloat;
  do
  {
    bufferdata =  lora_RXBUFF[lora_RXStart++];
    temp[tempptr++] = bufferdata;
  }
  while (bufferdata != ',');
  temp[tempptr] = 0;  //terminator for string
  tempfloat = (float)atof(temp);
  return tempfloat;
}


float extract_Uint()
{
  //extracts an unsigned int in ASCII format from buffer
  char temp[12];
  byte tempptr = 0;
  byte buffdata;
  uint16_t tempint;
  do
  {
    buffdata =  lora_RXBUFF[lora_RXStart++];
    temp[tempptr++] = buffdata;
  }
  while (buffdata != ',');
  temp[tempptr] = 0;  //terminator for string
  tempint = (unsigned int)atof(temp);
  return tempint;
}


byte next_Comma(byte localpointer)
{
  //skips through HAB packet (in CSV format) to next  comma
  byte bufferdata;
  do
  {
    bufferdata =  lora_RXBUFF[localpointer++];
  }
  while (bufferdata != ',');
  return localpointer;
}


void systemerror()
{
  while (1)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void led_FlashStart()
{
  byte i;
  for (i = 0; i <= 4; i++)
  {
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
  }
}


void lora_Setup1()
{
  //initialise LoRa device registers and check its responding
  lora_ResetDev();                               //clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);              //RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x0B);                 //RegOcp
  lora_Write(lora_RegLna, 0x23);                 //RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF);      //RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);         //RegPreambleLsb, default
  lora_Write(lora_RegFdevLsb, Deviation);        //LSB of deviation, 5kHz
  if (!lora_CheckDevice())
  {
    Serial.println();
    Serial.print(F("LoRa Device Error"));
    Serial.println();
  }
}


void lora_Custom1()
{
  //used to have a custom set of LoRa register settings for each test mode
}


void init_RXLoRaTest1()
{
  //setup for testmode
  modenumber = 1;
  lora_Setup1();
  lora_SetFreq(Frequency1, CalibrationOffset);
  lora_SetModem2(test1_Bandwidth, test1_SpreadingFactor, test1_CodeRate, Explicit);  //setup the LoRa modem parameters for test
}


void init_LoRaRelay()
{
  lora_Setup1();
  lora_SetFreq(Frequency3, CalibrationOffset);
  lora_SetModem2(relay_Bandwidth, relay_SpreadingFactor, relay_CodeRate, Explicit);  //setup the LoRa modem parameters for test
}

/*****************************************************************************/
//  Start Display routines
/*****************************************************************************/

#ifdef UseDisplay
void writescreen_TestPacket()
{
  byte tempbyte, tempbyte1, power;
  Display_Clear();
  Display_SetCurPos(0, 0);
#ifdef RelayEndpoint
  disp.print(("R "));
#endif
  disp.print(F("Test "));
  tempbyte = lora_RXBUFF[0] - 48;                    //convert ASCII to number
  tempbyte1 = lora_RXBUFF[1] - 48;                   //convert ASCII to number
  power = (tempbyte * 10) + tempbyte1;
  disp.print(power);
  disp.print(F("dBm"));
  Display_Update();
}


void writescreen_LocationBinaryPacket(float tempfloat1, float tempfloat2, uint16_t tempint)
{
  Display_Clear();
  Display_SetCurPos(0, 0);

#ifdef RelayEndpoint
  disp.print(F("Relay "));
#endif

  disp.print(F("Binary Location"));

  Display_SetCurPos(0, 5);
  disp.print(F("Lat "));
  disp.print(tempfloat1, 5);
  Display_SetCurPos(0, 6);
  disp.print(F("Lon "));
  disp.print(tempfloat2, 5);
  Display_SetCurPos(0, 7);
  disp.print(F("Alt "));
  disp.print(tempint);
  Display_Update();

}


void writescreen_HABPacket(float tempfloat1, float tempfloat2, uint16_t tempint)
{

  Display_Clear();
  Display_SetCurPos(0, 0);
#ifdef RelayEndpoint
  disp.print(F("Relay "));
#endif
  disp.print(F("HAB Packet"));

  Display_SetCurPos(0, 5);
  disp.print(F("Lat "));
  disp.print(tempfloat1, 5);
  Display_SetCurPos(0, 6);
  disp.print(F("Lon "));
  disp.print(tempfloat2, 5);
  Display_SetCurPos(0, 7);
  disp.print(F("Alt "));
  disp.print(tempint);
  Display_Update();

}


void writescreen_SNR()
{
  Display_SetCurPos(0, 2);
  disp.print(F("SNR "));
  disp.print(lora_PacketSNR);                 //now print the SNR
  disp.print(F("dB   "));
  Display_Update();
}


void writescreen_SNR2(int8_t tempsnr)
{
  Display_SetCurPos(0, 2);
  disp.print(F("SNR "));
  disp.print(tempsnr);                       //now print the saved SNR
  disp.print(F("dB   "));
  Display_Update();
}



void writescreen_RSSI()
{
  Display_SetCurPos(0, 1);
  disp.print(F("RSSI "));
  disp.print(lora_PacketRSSI);                 //now print the SNR
  disp.print(F("dBm   "));
  Display_Update();
}


void writescreen_RSSI2(int8_t temprssi)
{
  Display_SetCurPos(0, 1);
  disp.print(F("RSSI "));
  disp.print(temprssi);                 //now print the SNR
  disp.print(F("dBm   "));
  Display_Update();
}


void writescreen_PacketsBatch()
{
  Display_SetCurPos(0, 3);
  disp.print(F("Pkts "));                       //send count to LCD
  disp.print(Mode1_Packet_Count);               //send count to LCD
  disp.print(F("  Cyc "));
  disp.print(Mode1_Cycles);
  Display_SetCurPos(15, 0);
  disp.print(Current_Batch);
  Display_Update();
}
#endif

/*****************************************************************************/
//  End Display routines
/*****************************************************************************/


/*****************************************************************************/
//  Start SD Card routines
/*****************************************************************************/

#ifdef UseSD

void lora_AddressInfoSD()
{
  //print the information for packet last received
  logFile.print(F("RXType,"));
  logFile.write(lora_RXPacketType);
  logFile.print(F("  Destination,"));
  logFile.write(lora_RXDestination);
  logFile.print(F("  Source,"));
  logFile.write(lora_RXSource);

  logFile.print(F("  Length,"));
  logFile.print(lora_RXPacketL);
  logFile.print(F("  "));
}


void lora_ReceptionInfoSD2(int8_t tempsnr, int8_t temprssi)
{
  //print the information for packet last received
  //note, lora_PacketSNR has already been converted into a signed value
  //lora_PacketRSSI is a signed value also

  logFile.print(F("SNR,"));
  logFile.print(tempsnr);
  logFile.print(F("dB"));

  logFile.print(F("  RSSI,"));
  logFile.print(temprssi);
  logFile.print(F("dB   "));
}


void lora_ReceptionInfoSD()
{
  //print the information for packet last received
  //note, lora_PacketSNR has already been converted into a signed value
  //lora_PacketRSSI is a signed value also

  logFile.print(F("SNR,"));
  logFile.print(lora_PacketSNR);
  logFile.print(F("dB"));

  logFile.print(F("  RSSI,"));
  logFile.print(lora_PacketRSSI);
  logFile.print(F("dB   "));
}



boolean setup_SDLOG(uint16_t lbatch)
{

  logFile.flush();
  logFile.close();

#ifdef UseDisplay
  Display_Clear();
  Display_SetCurPos(0, 0);
  disp.print(F("Start New log"));
  Display_SetCurPos(0, 1);
  disp.print(Current_Batch);
  disp.print(F(".txt"));
  Display_Update();
#endif

  Serial.println(F("Start New log"));
  Serial.print(F("SD card..."));

  if (!SD.begin(SD_CS))                        //check if the SD card is present and can be initialised
  {
    Serial.println(F("Failed, or not present"));
#ifdef UseDisplay
    Display_Clear();
    Display_SetCurPos(0, 0);
    disp.print(F("SD Card error   "));
    Display_Update();
    delay(1000);
#endif
    SD_Found = false;
    return false;                                 //don't do anything more:
  }

  Serial.print(F("Initialized OK"));
  SD_Found = true;

  sprintf(file_name, "%d%s", lbatch, extension);
  logFile = SD.open(file_name, FILE_WRITE);

  Serial.print(F("...Writing to "));
  Serial.write(file_name[0]);

  if (lbatch > 9)
  {
    Serial.write(file_name[1]);
  }
  if (lbatch > 99)
  {
    Serial.write(file_name[2]);
  }
  if (lbatch > 999)
  {
    Serial.write(file_name[3]);
  }
  if (lbatch > 9999)
  {
    Serial.write(file_name[4]);
  }

  Serial.println(F(".txt"));

#ifdef UseDisplay
  Display_SetCurPos(0, 1);
  disp.print(Current_Batch);
  disp.print(F(".txt"));
  Display_Update();
  delay(1000);
#endif

  Int_Guard = true;

  return true;
}



void lora_RXBuffPrintSD(byte PrintType)
{
  //Print contents of RX buffer as ASCII, decimal or HEX
  //PrintType = 0 = ASCII
  //PrintType = 1 = Decimal
  //PrintType = 2 = HEX
  byte bufferData;

  logFile.write(lora_RXPacketType);
  logFile.write(lora_RXDestination);
  logFile.write(lora_RXSource);

  for (byte index = lora_RXStart; index <= lora_RXEnd; index++)
  {
    if (PrintType == 0)
    {
      logFile.write(lora_RXBUFF[index]);
    }

    if (PrintType == 1)
    {
      logFile.print(lora_RXBUFF[index]);
      logFile.print(F(" "));
    }

    if (PrintType == 2)
    {
      bufferData = lora_RXBUFF[index];
      if (bufferData < 0x10)
      {
        logFile.print(F("0"));
      }
      logFile.print(bufferData, HEX);
      logFile.print(F(" "));
    }
  }
}


void SD_WriteCurrentLoRaSettings()
{
  //prints the current LoRa settings, reads device registers
  float tempfloat;
  int16_t tempint;
  byte regdata;
  unsigned long bw;

  Serial.println(F("Write LoRa settings to log"));

  if (SD_Found)
  {
    tempfloat = lora_GetFreq();
    logFile.print(tempfloat, 3);
    logFile.print(F("MHz  ("));

    tempint = CalibrationOffset;
    tempfloat = (float)(tempint / 1000);
    logFile.print(tempfloat, 1);

    logFile.print(F("Khz)"));

    regdata = lora_Read(lora_RegModemConfig1);
    regdata = regdata & 0xF0;
    bw = lora_returnbandwidth(regdata);
    logFile.print(F("  BW"));
    logFile.print(bw);

    regdata = lora_Read(lora_RegModemConfig2);
    regdata = (regdata & 0xF0) / 16;
    logFile.print(F("  SF"));
    logFile.print(regdata);

    regdata = lora_Read(lora_RegModemConfig1);
    regdata = regdata & B00001110;
    regdata = regdata / 2; //move right one
    regdata = regdata + 4;

    logFile.print(F("  CR4/"));
    logFile.print(regdata);

    regdata = lora_Read(lora_RegModemConfig3);
    regdata = regdata & B00001000;
    logFile.print(F("  LDROPT_"));
    if (regdata == 8)
    {
      logFile.println(F("ON"));
    }
    else
    {
      logFile.println(F("OFF"));
    }
  }
  else
  {
    Serial.println(F("No SD card !"));
  }
}
#endif

/*****************************************************************************/
//  End SD Card routines
/*****************************************************************************/


void switch_press()
{
  //interrupt routine called when switch is pressed
  //byte index;

  if (Int_Guard)                         //unless the guard is true ignore interrupt
  {
    return;
  }

  delay(100);

  Int_Guard = true;                              //disable multiple interrupts having any affect

  if (!digitalRead(SWITCH1))
  {
    digitalWrite(LED1, HIGH);                    //Flash LED and de-bounce a bit
    Serial.println(F("Switch1 pressed"));
    Open_New_Log = true;                         //set flag for new log
    delay(100);
    digitalWrite(LED1, LOW);
  }
}


uint16_t generate_batch()
{
  uint16_t index, tempint;

  for (index = 0; index <= 1000; index++)
  {
    tempint =  tempint + analogRead(ADC_Channel);
  }

  Serial.print(F("Random Seed "));
  Serial.println(tempint);
  randomSeed((uint16_t) tempint);
  Current_Batch = random(1, 32767);                 //generater random batch number
}



void setup_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(SWITCH1), switch_press, CHANGE);   //This is a hardware interrupt
}

void setup()
{
  uint16_t tempint = 0;
  uint16_t i;
  float freq_temp;

  pinMode(LED1, OUTPUT);		                     //setup pin for PCB LED
  led_FlashStart();

  Serial.begin(115200);                           //setup Serial console ouput
  Serial.println();
  Serial.println();
  Serial.println(F("Reset"));
  Serial.println(F(programname));
  Serial.println(F(programversion));
  Serial.println(F(aurthorname));
  Serial.println();

  Serial.println();
  pinMode(lora_NReset, OUTPUT);		               //setup pin for LoRa device reset line
  digitalWrite(lora_NReset, HIGH);
  pinMode (lora_NSS, OUTPUT);		                 //setup pin for LoRa device slave select
  digitalWrite(lora_NSS, HIGH);
  pinMode (lora_DIO0, INPUT);                    //setup pin for LoRa device slave select
  pinMode (BUZZ, OUTPUT);                    //setup pin for LoRa device slave select

  generate_batch();

  SPI.begin();				                           //initialize SPI
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  if (lora_CheckDevice() == true)
  {
#ifdef EnableTone

#ifdef RelayEndpoint
    init_LoRaRelay();
#endif

#ifndef RelayEndpoint
    init_RXLoRaTest1();                          //do the initial LoRa Setup for testmode 1
#endif
    digitalWrite(BUZZ, HIGH);
    Serial.println(F("LoRa Tone"));
    digitalWrite(LED1, HIGH);
    lora_Tone(1000, 1000, 2);                    //Transmit an FM tone, 1000hz, 3000ms
    digitalWrite(LED1, LOW);
    Serial.println();
    digitalWrite(BUZZ, LOW);
#endif
  }
  else
  {
    Serial.println(F("LoRa Device Error"));
    systemerror();
  }

  Serial.println();


#ifdef RelayEndpoint
  init_LoRaRelay();
#endif

#ifndef RelayEndpoint
  init_RXLoRaTest1();                               //do the initial LoRa Setup for testmode 1
#endif

  lora_RXONLoRa();

  Serial.println();
  Serial.print(F("Receiver ready "));
  freq_temp = lora_GetFreq();
  Serial.print(freq_temp, 3);
  Serial.println(F("Mhz"));

#ifdef UseDisplay
  Serial.println(F("Display Started"));
  Display_Setup();
  Display_SetTextSize(1);
  Display_Clear();
  Display_SetCurPos(0, 0);
  disp.print(F("Display Started"));
  Display_Update();
  delay(1000);
  Display_Clear();
  Display_SetCurPos(0, 0);
  disp.print(F("Receiver ready "));
  Display_SetCurPos(0, 1);
  disp.print(freq_temp, 3);
  disp.print(F("Mhz"));
#ifdef RelayEndpoint
  Display_SetCurPos(0, 2);
  disp.print(F("Relay Endpoint"));
#endif
  Display_Update();
  delay(1000);
#endif

#ifdef UseSD
  setup_SDLOG(Current_Batch);                            //setup SD and delay a bit to ensure any pending ints cleared

  if (SD_Found)
  {
    SD_WriteCurrentLoRaSettings();
    logFile.flush();
    Open_New_Log = false;
  }
#endif

  delay(1500);

  pinMode(SWITCH1, INPUT_PULLUP);                 //setup switch
  Int_Guard = true;                               //set the guard variable to after attachInterrupt
  setup_interrupts();


#ifdef UseDisplay
  Display_Clear();
  Display_SetCurPos(0, 0);
  disp.print(F("Listening "));
  Display_Update();
#endif

  Serial.println(F("Listening "));

}


