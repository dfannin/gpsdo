# GPSDO
GPS Disciplined Oscillator Project (Arduino-based, 10MHz,1MHz,10kHz outputs ) 
## Description
The purpose of the GPS Displined Oscillator is to generate a highly accurate and precise 10 MHz reference frequency for use for test equipment calibration and as a reference frequency for radio and other electronic equipment. An additonal benefit is the generation of a highly accurate time source, sutable for use as a Straum-0 clock source.  The following design is a combination of a GPS receiver reference frequency, and a stable voltage controlled oscillator (VCXO) to generate a reference frequency using a digital Phase-locked loop circuit.

The GPSDO uses an arduino board (Mega2560) to provide display output (16x2 char display), control LEDs (for PPS, GPS Fix, PLL Lock and GPS faults), as well as USB serial output). 

Features for the GPSDO include 10MHz, 1 MHz and 10kHz reference frequency square wave outputs, plus a 10 MHz sine wave output (5 volt p-p @ 50 ohms). A one pulse per second (PPS) output is also included.   An LCD displays the GPS information (time, location, DOP, satellites, altitude), along with control LEDs that display when the  GPS Fix  and PLL Lock has been acheived.   

The companion schematics for the PLL circuit is also provided.

## Block Diagram
### GPS
A GPS receiver that generates a 10kHz reference frequency, as well as a PPS signal. An older Navman Jupiter-T  TU60-D120 was used, as they provide the required reference frequency (many newer GPS modules do not provide this, so YMMV).
### Oscillator
Isotemp 141-10 10MHz voltage controlled oscillator, which uses 0-5 volts as the control voltage range (postive slope). These can be found used on Ebay. 
### Phase Locked Loop
A phase-locked loop digital logic circuit, which uses a 3 x divide by 10 stages to divide 10MHz signal to 10 kHz, and a exclusive-or as the PLL phase detector. The phase detector compares the GPS reference frequency and VCXO frequency, and generates a control voltage to adjust the VCXO.   A second-order RC loop filter is used.  
### Arduino and Display
The Arduino provides the control and display circuit for driving the LCD output, and measuring the state of the GPS and VXCO. Additonally, the Arduino reads the GPS output messages (NMEA) for time, date and location information. 
### Power Supply
A 12 volt SMPS was used to drive a 5 volt regulator circuit for the power supply.  
## Software
The basic arduino sketch provides the overall GPSDO features, and is licenses under the MIT Open-Source License.
Libraries used include TinyGPS++ Library from http://arduiniana.org/libraries/tinygpsplus/ (LGPL 2.1 License), ClickButton (GPL v3 license), and Statistic (released to public domain).  The user will need to download and install these addtional libraries. Addtionally, the LiquidCrystal\_I2C library was used for the display.
