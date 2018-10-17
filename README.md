# ESP32 AES67 audio over IP (over WiFi) trial prototype application

This is a experimental application which aims to send audio packets via WiFi, while following the AES67 audio-over-IP requirements.

You need a working ESP32 (two of them), and [ESP-IDF](https://github.com/espressif/esp-idf) installed to your host computer.

## Results for now

AES67 L16 48kHz requires to send RTP packets every 250 _micro_ seconds in a particular format, but I acheived only about 2000 microseconds (= 2ms) for now.  
Sending _vanilla_ packets takes only 100 microseconds, so somewhere around the AES67 payload encoding must be taking time.

## Links
- The old one (which acheived only over 5ms)  
  https://github.com/sfc-arch/esp32_aes67_sender_old
- The receiver (this becomes a AP itself too)  
  https://github.com/sfc-arch/esp32_aes67_receiver