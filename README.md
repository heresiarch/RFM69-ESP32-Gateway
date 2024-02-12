# RFM69-ESP32-Gateway
Example RFM69 ESP32 Gateway using mqtt over TLS. The Gateway receives sensor data over 868 Mhz ISM band. 

## TODO
- document wiring of RFM69 and ESP32
- harden code
- decouple time critical radio receive task from mqtt publish using xTaskCreatePinnedToCore, ringbuffer and semaphore
- reduce size of TLS certificates using elliptic curves
-  

