# Demos used at Maker Faire Berlin 2016

This repository contains code, music and 3D-prints used at the Maker Faire in
Berlin 2016.

The demo was based on the Crazyflie and the Loco Positioning system, and the 
idea was to show autonomous flight.

## The stand

The stand was based on a cubic structure of pipes roughly 2x2x2 meters in size, 
with one pipe on each edge. The structure was based on a "party tent" and thus
went under the name "the tent". We attached LED-strips on three of the edges to 
represent the X, Y and Z axises and we used these to show the current position 
of the crazyflie.

## The tent

Files for the bottom joints and stands for the Loco Positioning anchors can be
found in the tent folder.

## Connections

```
Emergency stop --> Arduino --> USB --+
                                     |
                                     +--> ROS <--> Crazyradio <--> Crazyflie <--> LPS
                                     |     |
                                     |     +--> USB Serial port --> Arduino -> LED-strips
Sequencer --> MIDI ------------------+
```   
                        
## Emergency stop

The emergency stop button was handled by an arduino. The arduino was programmed
to be recognized as a USB gampad/joystick that ROS listened to. 

The code is in estop.

## The LED coordinate system

A python script subscribed to the position topic in ROS, converted it to a format
suitable for a serial port. An arduino on the other side of the serial interface
parsed the data and controlled the LED-strips using the 
https://github.com/adafruit/Adafruit_NeoPixel library.

Code in led-coordinates

## The sound and light show

We used a MIDI sequencer (https://lmms.io/) that ran a sequence with
an audio track combined with 4 MIDI tracks that controlled the position of the 
Crazyflie and the color of its LED-ring. The MIDI data was sent to a physical 
MIDI interface that in turn was connected to ROS. A python script converted
the MIDI messages to positions and LED data that was published on suitable
ROS topics that changed the set point of the Crazyflie and color of the LED-ring.

The music is composed by Kristoffer Richardsson on Garage Band and can be found
in the music folder. 

Code and sequence in the sound-light-show directory. 

## License

Music copyright Kristoffer Richardsson 2016, licensed under cc-by 

All code and other files licensed under the MIT license 
(https://opensource.org/licenses/MIT)

