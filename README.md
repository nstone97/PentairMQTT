# PentairMQTT
Arduino sketch to interface with the Pentair EasyTouch system using MQTT as the backend.

This sketch was designed around the Adafruit Feather M0 WIFI (https://www.adafruit.com/product/3010) using a MAX3485
to communicate with the EasyTouch pool controller.  The MAX3485 is connected to Serial1 on the feather using pin 9 as the
control pin for Transmit/Recieve (High/Low respectively).

This sketch requires:
  Adafruit WINC1500 library, detailed here: https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/using-the-wifi-module
  PubSubClient for MQTT: https://github.com/knolleary/pubsubclient
  
I'll do my best to upload a simple schematic for the MAX3485 soon!
