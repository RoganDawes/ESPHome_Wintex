# ESPHome Wintex

This code implements an ESPHome custom component that provides two significant features:

1. A TCP-UART bridge, similar to the COMIP or COM-WiFi. This allows use of the 
   Wintex or mobile applications to interact with the panel.

2. An on-board routine that exposes the alarm zones as binary_sensors, so that
   each zone can be reported to Home Assistant. As each zone is triggered by
   movement, or openening and closing the door, each binary_sensor in the 
   config yaml gets updated, and reported through to Home Assistant.

See example.yaml for an illustration of how to use this code.
