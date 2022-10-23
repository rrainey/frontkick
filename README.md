# Frontkick

This project is the third in a series of experimental data loggers for skydivers. This is the first in the series to experiment
with the design for real time guidance under canopy and alternatives for displaying those data.

## Past projects

* **Sidekick** - first project; based on off-the-shelf Adafruit commponents
* **Dropkick** - second project; custom PCB experiments with two different u-blox receivers (CAM-M8 and SAM-M8). Ultimately the SAM-M8 delivered better overall reception performance.

## This project

The design is composed of two separate modules.  A sensor pack gathers data.  It uses a Bluetooth interface to send sensor data to the receiving unit.  The receiver logs data. It also does some experimental interpretation of the incoming sensor data and uses that to display guidance and other information on a e-Paper display.

## Is an e-Paper display suitable for real-time applications?

That's one of the things that I'm working to answer with this project.  Overall, e-Paper displays are much slower to update than, say, an OLED or dedicated LCD. By "much", I mean "almost agonizingly much" as the display update time is currently around 300ms in these experiments.  Partial updating shows promise, but the stock display management drivers will probably require optimization to work adequately. We'll see.

## Google Fonts 

I used some Google fonts in this project. E-paper bitmaps generated from the Google Comfortaa font are used in the Wrist display project.  These fonts are free for commercial use. More information 
can be found at the [Google Fonts web site](https://developers.google.com/fonts/faq).

The e-paper graphics library required these fonts be in Adafruit GFX format. I used this excellent web-based converter to generate the GFX files from the original Truetype format distribed by Google:  https://rop.nl/truetype2gfx/ 