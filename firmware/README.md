## Setting up tool chains to build these sketches

I use Visual Studio Code and the arduino-cli command line for my Arduino development. 
I won't cover the VC Code install here but here's an outline of the steps to configure the arduino cli:

* install the arduino-cli tool: https://arduino.github.io/arduino-cli/0.21/installation/
* run 

```arduino-cli config init```

Make a note of the path to the YAML configuration file.

* edit the YAML configuration file and add the two additionla board urls show below:

```yaml
board_manager:
  additional_urls:
    - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json
    - https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
daemon:
  port: "50051"
directories:
  data: /users/riley/.arduino15
  downloads: /users/riley/.arduino15/staging
  user: /users/riley/Arduino
library:
  enable_unsafe_install: false
logging:
  file: ""
  format: text
  level: info
metrics:
  addr: :9090
  enabled: true
output:
  no_color: false
sketch:
  always_export_binaries: false
updater:
  enable_notification: true
  ```

* confirm the boards are available for installation with tehse two commands:
```sh
$ arduino-cli core search 'esp'

ID          Version Name              
arduino:avr 1.8.5   Arduino AVR Boards
esp32:esp32 2.0.5   esp32             
```

```sh
$ arduino-cli core search 'adafruit'

ID             Version Name                                        
adafruit:avr   1.4.15  Adafruit AVR Boards                         
adafruit:nrf52 1.3.0   Adafruit nRF52                              
adafruit:samd  1.7.10  Adafruit SAMD Boards                        
TeeOnArdu:avr  1.0.3   Adafruit TeeOnArdu                          
adafruit:wiced 0.6.6   Adafruit WICED                              
arduino:avr    1.8.5   Arduino AVR Boards                          
arduino:samd   1.8.13  Arduino SAMD Boards (32-bits ARM Cortex-M0+)
```

* Install the two board toolchains:

```sh
$ arduino-cli board install esp32:esp32
$ arduino-cli core install adafruit:nrf52
```

* install these Arduino libraries: 

```bash
$ arduino-cli lib install ArduinoJson GxEPD Adafruit_GFX_Library Adafruit_BusIO
```

* If you are on a Linux platform, ensure your user is a member of the dialout group: 

```sudo usermod -a -G dialout $USER```

```arduino-cli compile -b esp32:esp32:esp32 wrist```

As of today, build results will look something like this:

```
Used library         Version Path                                                                                            
BluetoothSerial      2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/BluetoothSerial
ArduinoJson          6.19.4  /data/part0/users/riley/Arduino/libraries/ArduinoJson                                           
SD                   2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/SD             
FS                   2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/FS             
SPI                  2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/SPI            
GxEPD                3.1.3   /data/part0/users/riley/Arduino/libraries/GxEPD                                                 
Adafruit_GFX_Library 1.11.3  /data/part0/users/riley/Arduino/libraries/Adafruit_GFX_Library                                  
Adafruit_BusIO       1.13.2  /data/part0/users/riley/Arduino/libraries/Adafruit_BusIO                                        
Wire                 2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/Wire           
WiFi                 2.0.0   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5/libraries/WiFi           

Used platform Version Path                                                                  
esp32:esp32   2.0.5   /data/part0/users/riley/.arduino15/packages/esp32/hardware/esp32/2.0.5
```
