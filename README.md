# GPSClock
Arduino GPS Clock for Uno and Ublox 6m

This script for Arduino Uno does a SoftwareSerial connection to a Ublox 6m GPS breakout od D2 and D3.
The Ublox 6m is not configured.
NMEA strings are parsed for basic date and time information, which is wtritten to an HD7044 compatible LCD.

Additional algorithms are included for Gridsquare and lineal distance to a Point of Interest (GPS coords).  

See https://vk3hn.wordpress.com/2018/06/27/diy-arduino-gps-clock/ for a writeup, video and pictures.

Paul Taylor VK3HN.  June 2018.  
