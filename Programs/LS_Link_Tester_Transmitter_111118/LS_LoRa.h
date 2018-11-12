//LoRa4.h
/*
*******************************************************************************************************************************
  Easy Build LoRa Programs for Arduino

  Copyright of the author Stuart Robinson - 11/11/18

  http://www.LoRaTracker.uk

  These programs may be used free of charge for personal, recreational and educational purposes only.

  This program, or parts of it, may not be used for or in connection with any commercial purpose without the explicit permission
  of the author Stuart Robinson.

  The programs are supplied as is, it is up to individual to decide if the programs are suitable for the intended purpose and
  free from errors.

  This is set of LoRa routines was introduced to make it easier to transition to environments that dont have the full set of ATMEGA 
  style libraries. 

  Note: This library must be considered as a work in progress, no guarantees. 
  
  To Do:
Add check for RX and TX packet longer than buffer can cope with.
Check for phantom packets fix
Done 111118 Check buffer assignements are int
Change assignments to be unequivocal, so that varible length is defined, not assumed;
Done 111118 Signed variabled to be changed to - int8_t, int16_t, int32_t, int64_t
Done 111118 UnSigned variabled to be changed to - uint8_t, uint16_t, uint32_t, uint64_t
Detect buffer end for buffers < 256 
Test RX and TX buffer size error checks
Check impact of 3 packet addressing bytes on buffer size error checks 
*******************************************************************************************************************************
*/


//Constant names for bandwidth settings
const uint8_t BW7800 = 0;          //7.8khz
const uint8_t BW10400 = 16;        //10.4khz
const uint8_t BW15600 = 32;        //15.6khz
const uint8_t BW20800 = 48;        //20.8khz
const uint8_t BW31200 = 64;        //31.2khz
const uint8_t BW41700 = 80;        //41.7khz
const uint8_t BW62500 = 96;        //62.5khz
const uint8_t BW125000 = 112;      //125khz
const uint8_t BW250000 = 128;      //250khz
const uint8_t BW500000 = 144;      //500khz

//Constant names for Spreading Factors
const uint8_t SF6 = 6;
const uint8_t SF7 = 7;
const uint8_t SF8 = 8;
const uint8_t SF9 = 9;
const uint8_t SF10 = 10;
const uint8_t SF11 = 11;
const uint8_t SF12 = 12;

//Constant names for coding rate settings
const uint8_t CR45 = 2;            //4:5
const uint8_t CR46 = 4;            //4:6
const uint8_t CR47 = 6;            //4:7
const uint8_t CR48 = 8;            //4:8

//Constant names for LoRa Header Settings
const uint8_t Explicit    = 0;     //Use to set explicit header
const uint8_t Implicit    = 1;     //Use to set implicit header

//Miscellaneous definitions
//const uint8_t Deviation = 0x52;
const uint8_t LowDoptON = 0x08;    //value to turn low data rate optimization on
const uint8_t LowDoptOFF = 0x00;   //value to turn low data rate optimization off
const uint8_t PrintASCII = 0;      //value to cause buffer print to appear as ASCII
const uint8_t PrintDecimal = 1;    //value to cause buffer print to appear as decimal numbers
const uint8_t PrintHEX = 2;        //value to cause buffer print to appear as hexadecimal numbers
const uint8_t Strip = 1;           //value for setting stip of addressing information on packet
const uint8_t NoStrip = 0;         //value for sending addressing information as part of packet


//SX1278 Register names
const uint8_t lora_RegFifo = 0x00;
const uint8_t lora_WRegFifo = 0x80;
const uint8_t lora_RegOpMode = 0x01;
const uint8_t lora_RegFdevLsb = 0x05;
const uint8_t lora_RegFrMsb = 0x06;
const uint8_t lora_RegFrMid = 0x07;
const uint8_t lora_RegFrLsb = 0x08;
const uint8_t lora_RegPaConfig = 0x09;
const uint8_t lora_RegOcp = 0x0B;
const uint8_t lora_RegLna = 0x0C;
const uint8_t lora_RegFifoAddrPtr = 0x0D;
const uint8_t lora_RegFifoTxBaseAddr = 0x0E;
const uint8_t lora_RegFifoRxBaseAddr = 0x0F;
const uint8_t lora_RegFifoRxCurrentAddr = 0x10;
const uint8_t lora_RegIrqFlagsMask = 0x11;
const uint8_t lora_RegIrqFlags = 0x12;
const uint8_t lora_RegRxNbBytes = 0x13;
const uint8_t lora_RegRxHeaderCntValueMsb = 0x14;
const uint8_t lora_RegRxHeaderCntValueLsb = 0x15;
const uint8_t lora_RegRxPacketCntValueMsb = 0x16;
const uint8_t lora_RegRxPacketCntValueLsb = 0x17;
const uint8_t lora_RegPktSnrValue = 0x19;
const uint8_t lora_RegPktRssiValue = 0x1A;
const uint8_t lora_RegRssiValue = 0x1B;
const uint8_t lora_RegFsiMSB = 0x1D;
const uint8_t lora_RegFsiLSB = 0x1E;
const uint8_t lora_RegModemConfig1 = 0x1D;
const uint8_t lora_RegModemConfig2 = 0x1E;
const uint8_t lora_RegSymbTimeoutLsb = 0x1F;
const uint8_t lora_RegPreambleLsb = 0x21;
const uint8_t lora_RegPayloadLength = 0x22;
const uint8_t lora_RegFifoRxByteAddr = 0x25;
const uint8_t lora_RegModemConfig3 = 0x26;
const uint8_t lora_RegFeiMsb = 0x28;
const uint8_t lora_RegFeiMid = 0x29;
const uint8_t lora_RegFeiLsb = 0x2A;
const uint8_t lora_RegPacketConfig2 = 0x31;
const uint8_t lora_RegDioMapping = 0x40;
const uint8_t lora_RegDioMapping2 = 0x41;
const uint8_t lora_RegVersion = 0x42;
const uint8_t lora_RegPllHop = 0x44;


uint8_t  lora_RXBUFF[lora_RXBUFF_Size];           //buffer where received packets are stored
uint8_t  lora_TXBUFF[lora_TXBUFF_Size];           //buffer for packet to send

uint8_t  lora_RXStart;                            //start of packet data in RXbuff
uint8_t  lora_RXEnd;                              //end of packet data in RXbuff
uint8_t  lora_RXPacketType;                       //type number of received packet
uint8_t  lora_RXDestination;                      //destination address of received packet
uint8_t  lora_RXSource;                           //source address of received packet
uint16_t lora_RXpacketCount;                      //count of received packets

uint8_t  lora_TXStart;                            //start of packet data in TX buffer
uint8_t  lora_TXEnd;                              //end of packet data in TX buffer
uint8_t  lora_TXPacketType;                       //type number of transmitted packet
uint8_t  lora_TXDestination;                      //destination address of transmitted packet
uint8_t  lora_TXSource;                           //source address of transmitted packet
uint16_t lora_TXpacketCount;                      //count of transmitted packets

int8_t lora_BackGroundRSSI;                       //measured background noise level
int8_t lora_PacketRSSI;                           //RSSI of received packet
int8_t lora_PacketSNR;                            //signal to noise ratio of received packet
uint16_t  lora_RXPacketL;                          //length of packet received, includes source, destination and packet type.
uint16_t  lora_TXPacketL;                          //length of packet received, includes source, destination and packet type.
uint8_t  lora_Power;                              //power setting for mode

uint16_t lora_TXTime;                             //used to record TX On time
uint16_t lora_StartTXTime;                        //used to record when TX starts
uint16_t lora_RXTime;                             //used to record RX On time
uint16_t lora_StartRXTime;                        //used to record when RX starts
uint16_t lora_CurrentFreq;                        //used to store current frequncy, needed for afc function
   
//precalculated values for frequency error calculation
const PROGMEM  float bandwidth[] = {0.1309, 0.1745, 0.2617, 0.3490, 0.5234, 0.6996, 1.049, 2.097, 4.194, 8.389};

/*
**************************************************************************************************
  Library Functions
**************************************************************************************************
*/


uint32_t lora_returnbandwidth(uint8_t BWregvalue)
{
  switch (BWregvalue)
  {
    case 0:
      return 7800;

    case 16:
      return 10400;

    case 32:
      return 15600;

    case 48:
      return 20800;

    case 64:
      return 31200;

    case 80:
      return 41700;

    case 96:
      return 62500;

    case 112:
      return 125000;

    case 128:
      return 250000;

    case 144:
      return 500000;

    default:
      break;
  }
  return 0;
}


float lora_CalcSymbolTime(float Bandwidth, uint8_t SpreadingFactor)
{
  //calculates symbol time from passed bandwidth (lbandwidth) and Spreading factor (lSF)and returns in mS
  float symbolTimemS;
  symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
  symbolTimemS = (1000 / symbolTimemS);
  return symbolTimemS;
}


uint8_t lora_GetOptimisation(uint8_t Bandwidth, uint8_t SpreadingFactor)
{
  //from the passed bandwidth (bandwidth) and spreading factor this routine
  //calculates whether low data rate optimisation should be on or off

  uint32_t tempBandwidth;
  float symbolTime;
  tempBandwidth = lora_returnbandwidth(Bandwidth);
  symbolTime = lora_CalcSymbolTime(tempBandwidth, SpreadingFactor);

#ifdef LORADEBUG
  Serial.print(F("Symbol Time "));
  Serial.print(symbolTime, 3);
  Serial.println(F("mS"));
#endif

  if (symbolTime > 16)
  {
#ifdef LORADEBUG
    Serial.println(F("LDR Opt on"));
#endif
    return LowDoptON;
  }
  else
  {
#ifdef LORADEBUG
    Serial.println(F("LDR Opt off"));
#endif
    return LowDoptOFF;
  }


}


void lora_ResetDev()
{
  //resets the LoRa device
  digitalWrite(lora_NReset, LOW);    //take reset line low
  delay(5);
  digitalWrite(lora_NReset, HIGH);   //take it high
}


void lora_Write(uint8_t Reg, uint8_t RegData)
{
  //write a byte to a LoRa register
  digitalWrite(lora_NSS, LOW);       //set NSS low
  SPI.transfer(Reg | 0x80);          //mask address for write
  SPI.transfer(RegData);             //write the byte
  digitalWrite(lora_NSS, HIGH);      //set NSS high
}


uint8_t lora_Read(uint8_t Reg)
{
  //read a byte from a LoRa register
  uint8_t RegData;
  digitalWrite(lora_NSS, LOW);       //set NSS low
  SPI.transfer(Reg & 0x7F);          //mask address for read
  RegData = SPI.transfer(0);         //read the byte
  digitalWrite(lora_NSS, HIGH);      //set NSS high
  return RegData;
}


uint16_t lora_GetFrequencyError()
{
  uint16_t msb, mid, lsb;
  int16_t freqerr;
  uint16_t bw;
  float bwconst;
  
  msb = lora_Read(lora_RegFeiMsb);
  mid = lora_Read(lora_RegFeiMid);
  lsb = lora_Read(lora_RegFeiLsb);
  bw = lora_Read(lora_RegModemConfig1) & (0xF0);
  bw = bw >> 4;
  bwconst = pgm_read_float_near(bandwidth + bw);
  
  freqerr = msb << 12;                     //shift lower 4 bits of msb into high 4 bits of freqerr
  mid = (mid << 8) + lsb;
  mid = (mid >> 4);
  freqerr = freqerr + mid;

  freqerr = (int16_t) (freqerr * bwconst);

  return freqerr;
}




void lora_SetFreq(uint64_t freq64, int16_t loffset)
{
  lora_CurrentFreq = freq64;
  freq64 = freq64 + loffset;
  freq64 = ((uint64_t)freq64 << 19) / 32000000;
  lora_Write(lora_RegFrMsb, (uint8_t)(freq64 >> 16));
  lora_Write(lora_RegFrMid, (uint8_t)(freq64 >> 8));
  lora_Write(lora_RegFrLsb, (uint8_t)(freq64 >> 0));
}


uint32_t lora_GetFreq2()
{
  //get the current set LoRa device frequency
  uint8_t Msb, Mid, Lsb;
  int16_t uinttemp;
  float floattemp;
  Msb = lora_Read(lora_RegFrMsb);
  Mid = lora_Read(lora_RegFrMid);
  Lsb = lora_Read(lora_RegFrLsb);
  floattemp = ((Msb * 0x10000ul) + (Mid * 0x100ul) + Lsb);
  floattemp = ((floattemp * 61.03515625) / 1000000ul);
  uinttemp = (uint32_t)(floattemp * 1000000);
  return uinttemp;
}


float lora_GetFreq()
{
  //get the current set LoRa frequency
  uint8_t lora_LFMsb, lora_LFMid, lora_LFLsb;
  uint16_t lora_Ltemp;
  float lora_Ltemp1;
  lora_LFMsb = lora_Read(lora_RegFrMsb);
  lora_LFMid = lora_Read(lora_RegFrMid);
  lora_LFLsb = lora_Read(lora_RegFrLsb);
  lora_Ltemp = ((lora_LFMsb * 0x10000ul) + (lora_LFMid * 0x100ul) + lora_LFLsb);
  lora_Ltemp1 = ((lora_Ltemp * 61.03515625) / 1000000ul);
  return lora_Ltemp1;
}


uint8_t lora_CheckDevice()
{
  //check there is a device out there, writes a register and read back
  uint8_t RegData;
  lora_Write(lora_RegFrMid, 0xAA);
  RegData = lora_Read(lora_RegFrMid);    //Read RegFrMid
  if (RegData == 0xAA )
  {
    return true;
  }
  else
  {
    return false;
  }
}


void lora_Setup()
{
  //initialize LoRa device registers and check its responding
  lora_ResetDev();                            //Clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);           //RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x2B);              //RegOcp
  lora_Write(lora_RegLna, 0x23);              //RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF);   //RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);      //RegPreambleLsb, default
  lora_Write(lora_RegFdevLsb, Deviation);     //LSB of deviation, 5kHz
  lora_Write(lora_RegPllHop, 0x2D);           //make sure fast hop mode not set
}


void lora_TXONDirect(uint8_t TXPower)
{
  //turns on transmitter,in direct mode for FSK and audio  power level is from 2 to 17
#ifdef LORADEBUG
  Serial.print(TXPower);
  Serial.print(F("dBm "));
#endif
  lora_StartTXTime = millis();
  lora_Write(lora_RegPaConfig, (TXPower + 0xEE));
  lora_Write(lora_RegOpMode, 0x0B);           //TX on direct mode, low frequency mode
}


void lora_TXOFF()
{
  //turns off transmitter
 lora_TXTime = (millis() - lora_StartTXTime);
 lora_Write(lora_RegOpMode, 0x08);           //TX and RX to sleep, in direct mode
#ifdef LORADEBUG
  Serial.print(lora_TXTime);
  Serial.println(F("mS"));
#endif
}


void lora_DirectSetup()
{
  //setup LoRa device for direct modulation mode
  lora_Write(lora_RegOpMode, 0x08);
  lora_Write(lora_RegPacketConfig2, 0x00);    //set continuous mode
}



void lora_Tone(uint16_t ToneFrequency, uint16_t ToneLength, uint8_t TXPower)
{
  //Transmit an FM audio tone without using tone library. Uses simple delayMicroseconds values that are assumed to be no more than
  //16383us or about 60Hz, lengths for low frequency tones will not be accurate. 
  
  uint32_t ToneDelayus, Tone_end_mS;
  ToneDelayus = ToneFrequency / 2;
  
  #ifdef LORADEBUG  
  Serial.print(F("lora_Tone()  "));
  Serial.print(F("Delay "));
  Serial.print(ToneDelayus);
  Serial.print(F("uS  "));
  #endif
  
  lora_DirectSetup();
  lora_Write(lora_RegFdevLsb, Deviation);     //We are generating a tone so set the deviation, 5kHz
  Tone_end_mS = millis() + ToneLength;
  lora_TXONDirect(TXPower);
  pinMode(lora_TonePin, OUTPUT);
  
  while (millis() < Tone_end_mS)
  {
  digitalWrite(lora_TonePin, HIGH);
  delayMicroseconds(ToneDelayus);
  digitalWrite(lora_TonePin, LOW);
  delayMicroseconds(ToneDelayus);   
  }
  
  pinMode(lora_TonePin, INPUT);
  lora_TXOFF();
}


void lora_SetModem2(uint8_t Bandwidth, uint8_t SpreadingFactor, uint8_t CodeRate, uint8_t Header)
{
  uint8_t optimisation;
  lora_Write(lora_RegOpMode, 0x08);          //RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOpMode, 0x88);          //go into LoRa mode
  lora_Write(lora_RegModemConfig1, (Bandwidth + CodeRate + Header));
  lora_Write(lora_RegModemConfig2, (SpreadingFactor * 16 + 7));
  optimisation = lora_GetOptimisation(Bandwidth, SpreadingFactor);
  lora_Write(lora_RegModemConfig3, optimisation);
}


void lora_Print()
{
  //prints the contents of LoRa registers to serial monitor
  uint8_t Loopv1, Loopv2, Reg, RegData;

  Serial.println(F("LoRa Registers"));
  Serial.print(F("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println();
  Reg = 0;
  for (Loopv1 = 0; Loopv1 <= 7; Loopv1++)
  {
    Serial.print(F("0x"));
    Serial.print(Loopv1, HEX);              //print the register number
    Serial.print(F("0  "));
    for (Loopv2 = 0; Loopv2 <= 15; Loopv2++)
    {
      RegData = lora_Read(Reg);
      if (RegData < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(RegData, HEX);           //print the register number
      Serial.print(F(" "));
      Reg++;
    }
    Serial.println();
  }
}


/*
**************************************************************************************************
  Library Functions RX
**************************************************************************************************
*/

void lora_RXBuffPrint(uint8_t PrintType)
{
  //Print contents of RX buffer as ASCII,decimal or HEX
  //PrintType = 0 = ASCII
  //PrintType = 1 = Decimal
  //PrintType = 2 = HEX
  uint8_t bufferData, index;
  Serial.write(lora_RXPacketType);
  Serial.write(lora_RXDestination);
  Serial.write(lora_RXSource);

  for (index = lora_RXStart; index <= lora_RXEnd; index++)
  {
    if (PrintType == 0)
    {
      Serial.write(lora_RXBUFF[index]);
    }

    if (PrintType == 1)
    {
      Serial.print(lora_RXBUFF[index]);
      Serial.print(F(" "));
    }

    if (PrintType == 2)
    {
      bufferData = lora_RXBUFF[index];
      if (bufferData < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print(bufferData, HEX);
      Serial.print(F(" "));
    }
  }
}


void lora_RXOFF()
{
  lora_Write(lora_RegOpMode, 0x89);                //TX and RX to sleep, in direct mode
  lora_RXTime = lora_RXTime + (millis() - lora_StartRXTime);
}


void lora_AddressInfo()
{
  //print the information for packet last received
  Serial.print(F("RXType,"));
  Serial.write(lora_RXPacketType);
  Serial.print(F("  Destination,"));
  Serial.write(lora_RXDestination);
  Serial.print(F("  Source,"));
  Serial.write(lora_RXSource);

  Serial.print(F("  Length,"));
  Serial.print(lora_RXPacketL);
  Serial.print(F(" "));
}


void lora_ReceptionInfo()
{
  //print the information for packet last received
  //note, lora_PacketSNR has already been converted into a signed value
  //lora_PacketRSSI is a signed value also

  Serial.print(F("SNR,"));
  Serial.print(lora_PacketSNR);
  Serial.print(F("dB"));

  Serial.print(F("  RSSI,"));
  Serial.print(lora_PacketRSSI);
  Serial.print(F("dB "));
}


int8_t lora_returnRSSI(uint8_t RegData)
{
  RegData = (157 - RegData) * (-1);
  return RegData;
}


int8_t lora_returnSNR(uint8_t RegData)
{

  if (RegData > 127)
  {
    RegData =  ((255 - RegData) / 4) * (-1);
  }
  else
  {
    RegData = RegData / 4;
  }
  return RegData;
}


uint8_t lora_ReadPacket()
{
  uint8_t index, RegData, Error;
  lora_RXpacketCount++;
  
  lora_RXPacketL = lora_Read(lora_RegRxNbBytes);
  
  if (lora_RXPacketL > (lora_RXBUFF_Size - 1))
  {
   lora_RXPacketL =  (lora_RXBUFF_Size - 1);
   Error = RXbuffersize;
   
  #ifdef LORADEBUG
  Serial.println(F("RXBUFF size error"));
  #endif
}
  
  //lora_RXPacketL = min((lora_Read(lora_RegRxNbBytes)),(lora_RXBUFF_Size-1));   //ensure long packet cannot overwrite buffer end
  
  lora_PacketRSSI = lora_returnRSSI(lora_Read(lora_RegPktRssiValue));
  lora_PacketSNR = lora_returnSNR(lora_Read(lora_RegPktSnrValue));

  lora_Write(lora_RegFifoAddrPtr, 0);        //set RX FIFO ptr

  digitalWrite(lora_NSS, LOW);               //start the burst read
  SPI.transfer(lora_RegFifo);                //address for burst read

  lora_RXPacketType = SPI.transfer(0);
  lora_RXDestination = SPI.transfer(0);
  lora_RXSource = SPI.transfer(0);

  lora_RXStart = 0;
  lora_RXEnd = lora_RXPacketL - 4;           //calculate the end of the packet in the buffer

  for (index = lora_RXStart; index <= lora_RXEnd; index++)
  {
    RegData = SPI.transfer(0);
    lora_RXBUFF[index] = RegData;
  }
  digitalWrite(lora_NSS, HIGH);
  
  return Error;

}


void lora_RXONLoRa()
{
  //puts lora device into listen mode and receives packet exits with packet in array lora_RXBUFF(256)

  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegFifoRxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);
  lora_Write(lora_RegIrqFlagsMask, 0x9F);                //only allow rxdone and crc error
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegDioMapping, 0x00);                  //DIO0 will be RXDone
  lora_Write(lora_RegOpMode, 0x8D);
  lora_StartRXTime = millis();
  lora_BackGroundRSSI = lora_Read(lora_RegRssiValue);    // get the background noise level
}



uint8_t lora_readRXready()
{
  uint8_t RegData;
  RegData = lora_Read(lora_RegIrqFlags);
  return RegData;
}


uint8_t lora_readRXready2()
{
  //reads the DIO0 pin to see if a packet is ready

  if (digitalRead(lora_DIO0))
  {
    return lora_Read(lora_RegIrqFlags);
  }
  else
  {
    return 0;
  }
}


uint8_t lora_waitPacket(char waitPacket, uint16_t waitSeconds)
{
  //wait time specified for a incoming packet, 0 = no timeout
  //returns a value of 0 for timeout, 1 for packet received

  //char PacketType;
  int8_t PacketType = 0;                            //function can exit before PacketType is assigned but not not used, this creates a compiler warning 
  uint8_t RegData;
  uint16_t endtime;
  endtime = (millis() + (waitSeconds * 1000));
  lora_RXONLoRa();
#ifdef LORADEBUG
  Serial.print(F("Wait "));
  Serial.write(waitPacket);
  Serial.println();
#endif

  do
  {
    RegData = lora_readRXready();

    if (RegData == 64)
    {
      lora_ReadPacket();
      lora_AddressInfo();
      Serial.println();
      PacketType = lora_RXPacketType;
      lora_RXONLoRa();                          //ready for next and clear flags
    }

    if ((waitSeconds > 0) && (millis() >= endtime))
    {
#ifdef LORADEBUG
      Serial.println(F("Timeout"));
#endif
      return 0;
    }

    if (Serial.available() > 0)
    {
#ifdef LORADEBUG
      Serial.println(F("Serial in ?"));
#endif
      keypress = 1;
      while (Serial.read() != -1);             //clear serial input buffer
      return 0;                                //treat a keypress as a timeout so return 0
    }

  }
  while ((PacketType !=  waitPacket));

  return 1;
}

/*
**************************************************************************************************
  Library Functions TX
**************************************************************************************************
*/



void lora_TXBuffPrint(uint8_t PrintType)
{
  //Print contents of TX buffer as ASCII,decimal or HEX

  uint8_t index, bufferData;
  Serial.write(lora_TXPacketType);
  Serial.write(lora_TXDestination);
  Serial.write(lora_TXSource);

  for (index = lora_TXStart; index <= lora_TXEnd; index++)
  {
    if (PrintType == 0)
    {
      Serial.write(lora_TXBUFF[index]);
    }

    if (PrintType == 1)
    {
      Serial.print(lora_TXBUFF[index]);
      Serial.print(F(" "));
    }

    if (PrintType == 2)
    {
      bufferData = lora_TXBUFF[index];
      if (bufferData < 0x10)
      {
        Serial.print(F("0"));
      }
      Serial.print(bufferData, HEX);
      Serial.print(F(" "));
    }
  }
}


void lora_TXONLoRa(uint8_t TXPower)
{
  //turns on LoRa transmitter, Sends packet, power level is from 2 to 17
#ifdef LORADEBUG
  Serial.print(TXPower);
  Serial.print(F("dBm  "));
#endif
  lora_StartTXTime = millis();
  lora_Write(lora_RegPaConfig, (TXPower + 0xEE));   //set TX power
  lora_Write(lora_RegOpMode, 0x8B);                 //TX on direct mode, low frequency mode
}



uint8_t lora_Send(uint8_t TXBuffStart, uint16_t TXBuffEnd, char TXPacketType, char TXDestination, char TXSource, uint32_t TXTimeout, uint8_t TXPower, uint8_t StripAddress)
{
  uint8_t index, BufferData, RegData, Error;
  uint8_t TXPacketL = 0;

  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegIrqFlagsMask, 0xF7);
  lora_Write(lora_RegFifoTxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);            //start burst read

  digitalWrite(lora_NSS, LOW);                      //Set NSS low
  SPI.transfer(lora_WRegFifo);                      //address for burst write

  if (!StripAddress)
  {
    SPI.transfer(TXPacketType);                     //Write the packet type
    SPI.transfer(TXDestination);                    //Destination node
    SPI.transfer(TXSource);                         //Source node
    TXPacketL = 3;                                  //We have added 3 header bytes
  }

  if (TXBuffEnd > (lora_TXBUFF_Size - 1))
  {
   TXBuffEnd =  (lora_TXBUFF_Size - 1);
   Error = TXSbuffersize;
   
  #ifdef LORADEBUG
  Serial.println(F("TXBUFF size error"));
  #endif
}
  
  for (index = TXBuffStart; index <= TXBuffEnd; index++)
  {
    TXPacketL++;

    if (TXPacketL > 253)                            //check for overlong packet here, wraps around from limit at 251 to 0
    {
      TXPacketL--;                                  //remove increment to packet length
      break;
    }
    BufferData = lora_TXBUFF[index];
    SPI.transfer(BufferData);
  }

  digitalWrite(lora_NSS, HIGH);                     //finish the burst write
  lora_Write(lora_RegPayloadLength, TXPacketL);
  TXTimeout = TXTimeout * 945;                      //convert seconds to mS, delay in TX done loop is 1ms

  lora_TXONLoRa(TXPower);

  do
  {
    delay(1);
    TXTimeout--;
    RegData = lora_Read(lora_RegIrqFlags);
  }
  while (TXTimeout > 0 && RegData == 0) ;           //use a timeout counter, just in case the TX sent flag is missed

if (TXTimeout == 0)
{
  Error = Error + TXtimeout;
  #ifdef LORADEBUG
  Serial.println(F("Send timeout error"));
  #endif
}

 lora_TXOFF();
 
 return Error;
}


uint8_t lora_QueuedSend(uint8_t TXBuffStart, uint8_t TXBuffEnd, char TXPacketType, char TXDestination, char TXSource, uint32_t TXTimeout, uint8_t TXPower, uint8_t Attempts, uint8_t StripAddress)
{
  //wait time specified for a incoming packet, 0 = no timeout
  //returns a value of 0 for timeout, 1 for packet sent and acknowledged

  uint16_t tempAttempts = Attempts;             //to store value of lora_Attempts

#ifdef LORADEBUG
  Serial.print(F("Queue "));
  Serial.write(TXPacketType);
  Serial.write(TXDestination);
  Serial.write(TXSource);
  Serial.print(F(" Attempts "));
  Serial.println(Attempts);
#endif

  do
  {

    if (!lora_waitPacket(ClearToSendCommand, 0))    //returns a value of 0 for timeout, 1 for packet received, no timeout
    {
#ifdef LORADEBUG
      Serial.println(F("Exit QueuedSend"));
#endif
      return 0;                                     //either timeout or key exit
    }

    delay(1000);

#ifdef LORADEBUG
    Serial.print(F("Send "));
    Serial.write(TXPacketType);
    Serial.write(TXDestination);
    Serial.write(TXSource);
    Serial.print(F(" "));
#endif

    lora_Send(TXBuffStart, TXBuffEnd, TXPacketType, TXDestination, TXSource, TXTimeout, TXPower, StripAddress);

    if (lora_waitPacket(ACK, 5) == 1)               //when returns the value of lora_RXPacketType may be updated
    {
      return 1;
    }

    if (Serial.available() > 0)
    {
#ifdef LORADEBUG
      Serial.println(F("Serial in ?"));
#endif
      keypress = 1;
      while (Serial.read() != -1);                  //clear serial input buffer
      return 0;
    }

    if (tempAttempts != 0)
    {
      Attempts--;
    }
    else
    {
      Attempts = 1;
    }

  }
  while ( (lora_RXPacketType != ACK) && (Attempts > 0) );

#ifdef LORADEBUG
  Serial.println(F("Timeout Waiting for ACK"));
#endif
  return 0;
}
