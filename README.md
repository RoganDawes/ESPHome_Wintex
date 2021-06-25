# ESPHome Wintex

This code implements an ESPHome custom component that provides two significant features:

1. A TCP-UART bridge, similar to the COMIP or COM-WiFi. This allows use of the 
   Wintex or mobile applications to interact with the panel.

2. An on-board routine that exposes the alarm zones as binary_sensors, so that
   each zone can be reported to Home Assistant. As each zone is triggered by
   movement, or openening and closing the door, each binary_sensor in the 
   config yaml gets updated, and reported through to Home Assistant.

See example.yaml for an illustration of how to use this code.

The target platform is an ESP32 connected to a Texecom Premier 832 panel, 
by means of some sort of level shifter. I used a TXB0104, supporting two
independent UART's, but a simple voltage divider can also work, I understand.

Choose whatever pins you prefer for the UART - the ESP32 is pretty flexible
about which pins are used for what peripheral.

This code *should* also work on an ESP8266, although you would have to choose
either the stream server, or the internal polling component, because the ESP8266
does not have two complete UART's. It *might* work with the SoftwareSerial
implementation, although I have not tested that at all. The required baud rate is
quite slow, although there may not be an implementation that deals with 2 STOP bits.

NOTE: This code is very specific to the Premier 832 panels. It does a direct memory
read at a particular address, for a specific number of zones (32). If your panel
maintains the zone status at a different address, that would obviously need to be
updated! The appropriate address was found by using Wintex through the StreamServer,
and monitoring the logs while viewing the Diagnostics for the Control Panel tab. The
version of Wintex I have only shows 16 zones at a time, so the address required
needs to be derived from the addresses read for each set of 16 zones. The StreamServer
logs Wintex reads and writes at Debug level, so the protocol messages can be seen
in the log stream.
