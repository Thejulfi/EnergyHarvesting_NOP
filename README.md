# EnergyHarvesting_NOP
Projet technique ETN5 SETR Polytech Nante

# Installation

- [Energia IDE](http://energia.nu)
- [LIS3MDL](https://github.com/pololu/lis3mdl-arduino)
- [RunnignMedian](https://github.com/RobTillaart/RunningMedian)

## Libraries

In _Libraries/_.

Add (copy/paste) libraries in your libraries' folders.

For Macos :
- /Applications/Energia.app/Contents/Java/hardware/energia/msp430/libraries
- /Users/**USERNAME**/Library/Energia15/packages/energia/hardware/msp430/1.0.7/libraries


## Current consumption measurement with magnetometers

Using the magnetometer code without Serial communication. Here the jumper placement as well as schematic. I'm using MSP430FR5969 target and BoosterPack.

### Schematic
```
            USB
           +-----+
          ++-----+---------------------------------------+
          |                 J13                          |
          |                +-----+------+-----+          +---Gnd/Gnd
          |                |+++++| . . .|+++++|          |
          |                |+++++| . . .|+++++|          |
          |                +-----+------+-----+          |
          |                                              |
          |            J2   +----+    +----+   +----+    |
          |         +--+--+ |  . |    |++++|   |  . |    |
          |         |. |++| |  . |    |++++|   |  . |    |
          |         +--+--+ |  . |J1  |++++|J10|  . |J12 |
          |                 +----+    +----+   +----+    |
+ amp     |                                              |
     -----+----------+-+   +--+-+   +--+-+     +----+    |
          |  Current |+|   |  |+|   |  |+|     |  . |    |
- amp     |       |  |+|J9 |  |+|J6 |  |+|J11  |  . |J9  |
    ------+-------+--+-+   +--+-+   +--+-+     +----+    |
          |                                              |
          |                                              |
          |MSP430                                        |
          +---------------+----+-------------------------+
                          |    |
                  P3.5/SCL|    |P3.6/SDA
                          |    |
                          |    |
```
