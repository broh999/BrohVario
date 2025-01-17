# BrohVario

**Arduino paragliding tool**

## Author notes

This project is forked from [IvkoPivko](https://github.com/IvkoPivko) [MiniVario-Arduino](https://github.com/IvkoPivko/MiniVario-Arduino) (Awesome code!) and was developed using the [jarzebski](https://github.com/jarzebski) [Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611) library (I attached a .zip version).

I forked it to adapt to my hardware, I want to recycle an arduino nano board I owned, but I also want to understand the code... so I translated all the comments and variables names to English.

## Release notes

I work on two version:
- FULL version: include two selectlable mode, Audio vario and bluetooth pressure sensor, and can be powered by a 2S LiPo
- BT_USB: work only in BT mode and is powered from USB.

### FULL V 20

- Completely translated to English
- Updated to work with 2S lipo battery (and sending voltage via BT)
- TODO: working on low voltage audible warning

### BT_USB Verion 1

I decide to minimize the vario because my principal goal is to use it as a bluetooth external sensor to pair with [XCtrack](http://xctrack.org/).

In this version:

- The project is powered directly from the USB port
- There's not switch to select "bluetooth mode", it ever start in bluetooth mode
- Of course, there's no battery and no voltage divider to read the battery charge

I use it with a small powerbank, both inside a pocket of my paragliding harness. Some useful information about this solution:

- If you squeeze the pocket, the sensor pressure change and XCTrack read a fake descent. When the pocket is released, the pressure decreases and XC read a fake climb. This happens for few seconds, but is better to choose a pocket that will be not squeezed and released during normal flight.
- Most power-banks can switch off if the current draw isn't enough. This circuit is close to this value. Initially I resolve this issue leaving connected also the smartphone to the power bank (I've a 2 port power bank), then I found a smaller power-bank with a lower cut-off value that works perfectly also without the phone.

