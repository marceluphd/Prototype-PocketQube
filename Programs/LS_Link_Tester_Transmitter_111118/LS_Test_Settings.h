//LSTest_Settings.h
/*
******************************************************************************************************

LoRaTracker Programs for Arduino

Copyright of the author Stuart Robinson - 11/11/18

http://www.LoRaTracker.uk

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.

This test settinfs file is identical to the transmitter settings file

To Do:

******************************************************************************************************
*/

//#define DEBUG
//#define LORADEBUG
//#define FSKRTTYDEBUG
//#define UseBatchNumber                            //enable this define to send batch number with mode change packet, needs EEPROM or FRAM to work

//#define ListenCommand                             //enable this define to enable listen for remote commands 

const uint16_t Payload_buffer = 256;
const uint16_t Output_len_max = 256;

#define SendTestpacket                              //Enable send of test packet, these are the packets counted be the receiver
#define SendBinaryLocation                        //Enable send of binary short location payload
#define SendLoRaSATPacket                         //Enable send of LoRaSAT data packet
#define SendFSKRTTY                               //Enable senf of FSK RTTY data

const uint8_t start_power = 17;                     //Start power for transmissions
const uint8_t end_power = 2;                        //End power for transmissions

const uint16_t lora_RXBUFF_Size = 256;
const uint16_t lora_TXBUFF_Size = 256;

//*******  Setup Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t LoRa_Frequency = 434400000;        //LoRa frequency
const int32_t CalibrationOffset = 0;            //adjustment for frequency in Hz, assumed at room temp
#define LoRa_Test_Bandwidth BW125000              //LoRa bandwidth
#define LoRa_Test_SpreadingFactor SF7             //LoRa spreading factor
#define LoRa_Test_CodeRate CR48                   //LoRa coding rate


//FSK Modem Parameters
const uint32_t FSK_Frequency = 434400000;         //FSK data and FSK RTTY frequency
const int32_t FSK_CalibrationOffset = 0;          //adjustment for frequency in Hz, assumed at room temp
const uint8_t Deviation = 0x52;                   //deviation for FSK (FM) tones

//GPS co-ordinates to use for the test location transmission
const float TestLatitude  = 51.48230;             //GPS co-ordinates to use for test
const float TestLongitude  = -3.18136;            //Cardiff castle keep, used for testing purposes  
const float TestAltitude = 25.5;


const uint8_t Flight_ID[15] = "LoRaSAT";          //used for the FSK RTTY output HAB style   
const uint8_t ThisNode = '1';                     //node number goes out as part of packet addressing 
const uint8_t config_byte = 0;                    //goes out with some packets
const uint16_t LoopStartTone_lengthmS = 250;      //length of LoopStartTone in mS
const uint16_t ModeStartDelaymS = 1000;           //delay in mS after sending mode change at start of loop, gives RX time to print packet totals.
const uint16_t mode_delaymS = 500;                //mS delay between modes
const uint16_t packet_delay = 600;                //mS delay between packets, minimum for BW125000, SF12 test packet, need to allow for possible relay delay

//********************************************************


//**************************************************************************************************
// Define which Memory to use for storage of reset count - used a batch counter
//**************************************************************************************************

#define Memory_Library "LS_EEPROM.h"                //define this file if the internal EEPROM is in use, for devices which have it!
//#define Memory_Library "LS_FRAM_24CL64.h"         //define this file if an I2C FRAM is in use
