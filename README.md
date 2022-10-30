# Frontkick

This project is the third in a series of experimental data loggers for skydivers. This is the first in the series to experiment
with the design of real time guidance under canopy and alternatives for displaying those data.

## Past projects

* **Sidekick** - first project; based on off-the-shelf Adafruit components
* **Dropkick** - second project; custom PCB experiments with two different u-blox receivers (CAM-M8 and SAM-M8). Ultimately the SAM-M8 delivered better overall reception performance.

## This project

The design is composed of two separate modules.  A sensor pack gathers data.  It uses a Bluetooth interface to send sensor data to the receiving unit.  The receiver logs data. It also does some experimental interpretation of the incoming sensor data and uses that to display guidance and other information on either an e-Paper or a more traditional TFT LCD display.

### Sensor Pack Components

| Mfr ID      | Manufacturer       | Description |
|-------------|--------------------|------------------------|
|[4062](https://www.adafruit.com/product/4062) | Adafruit | Adafruit Feather nRF52840 Express|
|[3775](https://www.mikroe.com/13dof-click)| Mikroe | 13DOF Click
|[GPS-00177](https://www.sparkfun.com/products/177)|SparkFun Electronics|Antenna GPS Embedded SMA|
| [GPS-17285](https://www.sparkfun.com/products/17285)|SparkFun Electronics|GPS Breakout - NEO-M9N, SMA (Qwiic)|

### E-Paper Wrist Display Components

| Mfr ID      | Manufacturer       | Description |
|-------------|--------------------|------------------------|
|[N/A](https://www.aliexpress.us/item/3256801850594766.html?gatewayAdapt=glo2usa4itemAdapt&_randl_shipto=US) | LilyGo |LILYGO TTGO T5 V2.2 ESP32 2.9" EPaper Plus Module|

### TFT Wrist Display Components
| Mfr ID      | Manufacturer       | Description |
|-------------|--------------------|------------------------|
|[N/A](http://www.lilygo.cn/prod_view.aspx?TypeId=50053&Id=1380&FId=t3:50053:3) | LilyGo |LILYGO® TTGO T-WATCH 2020 V2 GPS IPS Open Source ESP32 WIFI Bluetooth Capacitive Touch Screen|

Note: currently LilyGo's web site only shows a V3 watch - I am testing using a V2 version of the product. The V2
version remains listed for sale on the LilyGo sales pages
on AliExpress and Banggood.

## Is an e-Paper display suitable for real-time applications?

That's one of the things that I'm working to answer with this project.  Overall, e-Paper displays are much slower to update than, say, an OLED or dedicated LCD. By "much", I mean "almost agonizingly much" as the display update time is currently around 300ms in these experiments.  Partial updating shows promise, but the stock display management drivers will probably require optimization to work adequately. We'll see.

## Google Fonts 

I used some Google fonts in this project. E-paper bitmaps generated from the Google Comfortaa font are used in the Wrist display project.  These fonts are free for commercial use. More information 
can be found at the [Google Fonts web site](https://developers.google.com/fonts/faq).

The e-paper graphics library required these fonts be in Adafruit GFX format. I used this excellent web-based converter to generate the GFX files from the original Truetype format distribed by Google:  https://rop.nl/truetype2gfx/ 

## Technical References
[BMI088 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

[Application note: BMI08x FIFO usage](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an005.pdf)


[BME680 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf)

[Adafruit Guide: Feather nRF52840 Express](https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather)

[u-blox M9N-00B GNSS IC datasheet](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf)