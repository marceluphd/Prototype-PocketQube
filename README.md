# LoRaSAT PocketQube Satellite

This is a proof of concept prototype of a LoRa based processor and radio board for a PocketQube satellite. It follows much of the design principles of the successful [$50SAT PocketQube satellite](http://www.50dollarsat.info/). A 1P PocketQube is 50mm x 50mm and this allows for boards that are approximately 40mm square. 

The concept behind this design is that the main board is primarily intended for use in a PocketQube satellite but that the board can also be used for terrestrial remote sensor applications. 

The radio transceiver module (RFM98) uses a LoRa device developed by Semtech and released in 2013. Typical LoRa modules are the same size (15mmx15mm) as previous FSK data modules such as the RFM22B used in $50SAT. LoRa uses a proprietary form of spread spectrum radio technology and is capable of receiving signals that are up to 20dB below noise level. This give LoRa significant range\distance advantages over previous generation FSK devices, LoRa typically goes 10 times further for the same transmitter power. 

The very long range capability of LoRa has resulted in a new family of sensor based applications, in particular [The Things Network](https://www.thethingsnetwork.org/).

The current distance record for these simple LoRa modules is 702km;

[702km LoRa](https://www.thethingsnetwork.org/article/ground-breaking-world-record-lorawan-packet-received-at-702-km-436-miles-distance?source=techstories.org)

Note that that the 702km, ground to high altitude balloon, was achieved at 868Mhz with only 25mW of transmit power. An amateur PocketQube satellite running at 434Mhz and the full 100mW of the LoRa device would have a link budget advantage of 12dBm, which should increase the achievable distance to 2,800km. 

LoRa devices can also transmit the FSK RTTY data that was used with such success on $50SAT. This mode allowed radio amateurs worldwide to receive the core data of the satellites performance at distances of more than 2,000km.      

LoRa has significant potential for use in very small and low power satellites. The small size of a PocketQube implies low amounts of power from solar panels which in turns means limited power for data transmissions. The success of $50SAT did demonstrate that 100mW FSK data could be received from a satellite at a range of 1000km but this did require the use of good quality low noise amplifiers and antennas. LoRa, with its much greater range capability has the potential to be received with very simple hand-held receivers, such as that shown in the picture below;

<br><br>

![Picture 1](/Pictures/ProMiniShields.jpg)

This PocketQube processor\radio board is designed to be used with two other boards, all stacked vertically. There would be a battery and solar power controller board and a sensor board, giving a 3 board stack. The power and I\O connections between the board run vertically on the board edges through a series of standard 0.1" spaced connectors. 

For terrestrial outdoor applications the stack of boards can be conveniently mounted in housings made from plastic plumbing pipe and end caps. The outline of a typical 50mm internal diameter pipe is shown, when used in this mode the boards will have the corners milled off. 

<br><br>
## Processor and Radio Board Features.


![Picture 1](/Pictures/LoRaSAT_PCB_Layout.jpg)
<br><br>

The prototype board has been designed with these features;

Based on the Arduino environment, single sided PCB, optional microSD card holder on rear side.

0.1uA deep sleep current

ATmega1284P processor, 128Kbyte flash memory, 16Kbyte RAM, 4Kbyte EEPROM, 32 I/O pins, 8Mhz crystal.

LoRa\FSK radio transceiver, SMA antenna socket. Versions available for 434Mhz, 868Mhz and 915Mhz Industrial Scientific and Medical (ISM) bands 

8Kbyte FRAM for power off storage, write endurance of 100,000,000,000,000 times.

TPL5110 power down timer, preset from seconds to 2 hours. Enables processor to be powered down completely.

TPL5010 interrupt sleep timer, preset from seconds to 2 hours

TC54 Low voltage lockout

DS18B20 Temperature Sensor 

Polyfuse on battery input

Reverse polarity protection

Reset and power up push buttons

Two hardware UARTs, one for program upload and debug, second for sensors such as GPS etc.    

Stackable design. Battery and solar power management board intended for bottom, processor and radio board in middle, sensor or GPS board for top.

Indicator LED

Access to serial boot loader and ISP programming.

Design can be built DIY, auto assembly not required.

## Board details
All measurements in mm

Board is 40 x 40

Edge clearance from tracks and components 0.5

Origin 0,0 bottom left 

ConA centre 1.25,20

ConB centre 20, 38.75 

ConC centre 38.75,20

ConP program port) centre 4.45,27.56


## Use of BASIC

It should be noted that [Great COW Basic](http://gcbasic.sourceforge.net/Typesetter/index.php/Home) (GCB) will run on the ATmega1284P, so is a useful option if an alternative language was needed, perhaps for teaching purposes. In applications where an Arduino C\C++ application has memory and speed issues, Great Cow Basic can have advantages, see [GCB versus Arduino](https://www.youtube.com/watch?v=qdloFhULa3I) for some examples. 

<br><br>
### Stuart Robinson
### November 2018

 