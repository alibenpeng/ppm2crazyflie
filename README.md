# Standalone PPM to Crazyflie transmitter module

Control a Crazyflie nano quadcopter with any RC-remote with a PPM output.

Should compile with the Arduino-IDE or with my arduino-buildroot and
run on the Atmega\*8 family, probably others.

Connections:

AVR                    | Connect to..
-----------------------|-------------
PB0      (Arduino D08) | PPM Input
PB1      (Arduino D09) | nRF24L01+ CSN (SS)
PB2      (Arduino D10) | nRF24L01+ CE
PB3 MOSI (Arduino D11) | nRF24L01+ MOSI
PB4 MISO (Arduino D12) | nRF24L01+ MISO
PB5 SCK  (Arduino D13) | nRF24L01+ SCK
PD2      (Arduino D02) | LED with resistor (optional)

Remember that the nRF24L01+ is tolerant to 5V signal levels but needs 3.3V to run.

