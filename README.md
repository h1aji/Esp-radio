# Esp-radio
Internet radio based on ESP8266 and VS1053.  
This fork is optimised for Platformio and replaced ILI9163C display with LCD2004.

### Features
- Can connect to MP3, OGG or AAC audio streams.
- Can connect to a standalone mp3 file on a server.
- Can play iHeartRadio stations.
- Supports .m3u playlists.
- Uses a minimal number of components.
- Handles bitrate up to 192 kbps.
- Has a preset list of maximum 100 favorite radio stations in configuration file.
- Configuration file can be edited through web interface.
- Can be controlled via web browser.
- Can be controlled via Android app.
- Can be controlled over MQTT.
- Can be controlled over Serial Input.
- Optional infra-red control support.
- 20 kB ring buffer to provide smooth playback.
- Optional SPI RAM for larger buffer.
- Software update over WiFi (OTA).
- Bass and treble control.
- Configuration available if no WiFi connection can be established.
- The strongest available WiFi network is automatically selected.
- Heavily commented source code, easy to add extra functionality.
- Debug information through serial output.
- LittleFS filesystem used for configuration of WiFi SSIDs, passwords and small MP3-files.


### Wiring

|  ESP-12F  |  VS1053  |  23LC1024  |  TSOP4838  |  LCD2004  |
|-----------|----------|------------|------------|------------
|  GPIO04   |          |            |            |    SDA    |
|  GPIO05   |          |            |            |    SCL    |
|  GPIO00   |   CS     |            |            |           |
|  GPIO14   |   SCK    |    SCK     |            |           |
|  GPIO12   |   MISO   |  SO/SIO1   |            |           |
|  GPIO13   |   MOSI   |  SI/SIO0   |            |           |
|  GPIO16   |   DCS    |            |            |           |
|  GPIO02   |          |            |    OUT     |           |
|  GPIO15   |          |     CS     |            |           |
|  GPIO09   |   DREQ   |            |            |           |
|  GPIO10   |   RST    |            |            |           |


### Photos
<details>

![Photo1](./doc/photo3.jpg)
![Photo2](./doc/photo2.jpg)
![Photo3](./doc/photo1.jpg)
</details>
