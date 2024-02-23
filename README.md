# RFM69-ESP32-Gateway
An illustrative example of an RFM69 ESP32 Gateway harnessing MQTT over TLS. This Gateway acquires sensor data from the 868 MHz ISM band and transmits it wirelessly via MQTT to an MQTT broker. To configure the settings, WifiManager is employed. Overcoming the constraint of WiFiManagerParameter, which lacks support for multiline input fields, required encoding the ca.crt, client.crt, and client.key certificate files using the base64 CLI with the -w 0 option. This method ensured that the resulting string was consolidated into a single line, facilitating its integration during the initial WifiManager setup. Subsequently, to decode the data, the mbedtls_base64_decode function was utilized, yielding valid certificates for WifiClientSecure.
## TODO
- document wiring of RFM69 and ESP32
- harden code
- decouple time critical radio receive task from mqtt publish using xTaskCreatePinnedToCore, ringbuffer and semaphore
- reduce size of TLS certificates using elliptic curves 

