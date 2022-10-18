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

* Install the two board toolchains; wrist is ESP32, frontkick is nrf52:

```sh
$ arduino-cli board install esp32:esp32
$ arduino-cli core install adafruit:nrf52
```

* install these Arduino libraries: 

```bash
$ arduino-cli lib install ArduinoJson GxEPD Adafruit_GFX_Library Adafruit_BusIO
$ arduino-cli lib install "SparkFun u-blox GNSS Arduino Library" BME680 MicroNMEA
```

* Linux-only; see https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/update-bootloader-use-command-line 

```
$ pip3 install --user adafruit-nrfutil
```
Also on Linux, you must add ```$HOME/.local/bin``` to your ```$PATH```

* If you are on a Linux platform, ensure your user is a member of the dialout group: 

```sudo usermod -a -G dialout $USER```

```arduino-cli compile -b esp32:esp32:esp32 wrist```

As of today, build results will look something like this:

```
Used library         Version Path                                                                                            
BluetoothSerial      2.0.0   
ArduinoJson          6.19.4                                            
SD                   2.0.0              
FS                   2.0.0               
SPI                  2.0.0               
GxEPD                3.1.3                                                    
Adafruit_GFX_Library 1.11.3                                    
Adafruit_BusIO       1.13.2                                          
Wire                 2.0.0              
WiFi                 2.0.0              

Used platform Version Path                                                                  
esp32:esp32   2.0.5   
```

```
$ arduino-cli compile -b adafruit:nrf52:feather52840 frontkick
```

```
Library Adafruit_nRFCrypto has been declared precompiled:
Using precompiled library in /data/part0/users/riley/.arduino15/packages/adafruit/hardware/nrf52/1.3.0/libraries/Adafruit_nRFCrypto/src/cortex-m4/fpv4-sp-d16-hard


Used library                         Version Path                                                                                                        
Wire                                 1.0                         
Bluefruit52Lib                       0.21.0            
Adafruit_nRFCrypto                   0.0.5         
Adafruit_TinyUSB_Arduino             1.7.0   
Adafruit_LittleFS                    0.11.0         
InternalFileSytem                    0.11.0         
BME680                               1.0.10                                                              
SPI                                  1.0                          
SparkFun_u-blox_GNSS_Arduino_Library 2.2.16                                
MicroNMEA                            2.0.6                                                            

Used platform  Version Path                                                                     
adafruit:nrf52 1.3.0   
```