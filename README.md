# Customized Ardupilot for Control Loops Testing

Ardupilot firmware is used to control and manage unmanned vehicles and supports various controller boards. We have customized the firmware so that it can be used for control loops testing for quadrotor drones. The control loops are split into 3 parts.

The customized ardupilot firmware retains all the original ardupilot modes with a new mode called 'Mode New Control' which has its own modeular control loops different. They have been separated to allow easy modification to each one of them.

Following flow is followed:

> (X, Y) ---> Position Controller --- (Roll, Pitch, Yaw) ---> Attitude Controller --- (PID feedbacks) ---> Motor Mixing Matrix\
> (Z) ---> Altitude Controller --- (Throttle) ---> Motor Mixing Matrix

## Motor Mixing Matrix
Following function contains motor mixing matrix. We have included equations for plus frame and cross frames of Quad Copter. Other frames can be added there if needed.

> **Input**: float throttle_mix_custom, pid[roll_i], pid[pitch_i], pid[yaw_i]\
> **Output**: (Writes PWM to motors)


> void ModeNewControl::mix_and_output(){\
> &nbsp;&nbsp;&nbsp;&nbsp;...\
> }

## Attitude Control
The attitude control takes Roll, Pitch and Yaw as input and applies PID on it to get feedbacks which is passed into motor mixing matrix. This can be modified in the following function:

> **Input**: float target_roll, target_pitch, target_yaw (Values in degrees)\
> **Output**: float pid[roll_i], pid[pitch_i], pid[yaw_i]

> void ModeNewControl::PID_motors(){\
> &nbsp;&nbsp;&nbsp;&nbsp;...\
> }

## Altitude Control
It takes target altitude as input and outputs throttle which is passed to motor mixing matrix.

> **Input**: float target_z (Values in cms)\
> **Output**: float throttle_mix_custom

> void ModeNewControl::get_custom_throttle(){\
> &nbsp;&nbsp;&nbsp;&nbsp;...\
> }

## Position Control
It takes target X, Y coordinates from starting position as input and outputs required roll, pitch to attain it.

> **Input**: Vector2f _pos_target (Values in cms)\
> **Output**: float target_roll, target_pitch (Values in degrees)

> void ModeNewControl::run_custom_pos(){\
> &nbsp;&nbsp;&nbsp;&nbsp;...\
> }

# Original Ardupilot Documentation

## The ArduPilot project is made up of: ##

- ArduCopter (or APM:Copter) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane (or APM:Plane) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- ArduRover (or APMrover2) : [code](https://github.com/ArduPilot/ardupilot/tree/master/APMrover2), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub (or APM:Sub) : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums ##

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information ##

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://dev.ardupilot.org>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://gitter.im/ArduPilot/ardupilot>

## Top Contributors ##

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved ##

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers especially for ArduCopter to help us find bugs: [release procedures](https://dev.ardupilot.org/wiki/release-procedures)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Gitter](https://gitter.im/ArduPilot/ardupilot_wiki)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://dev.ardupilot.com/wiki/license-gplv3)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers ##

Ardupilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.  See also the list of developers with [merge rights](https://github.com/orgs/ArduPilot/teams/ardupilot-maintainers/members).

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: APM1, APM2, Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Jacob Walser](https://github.com/jaxxzer):
  - ***Vehicle***: Sub
- [Lucas De Marchi](https://github.com/lucasdemarchi):
  - ***Subsystem***: Linux
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Chris Olson](https://github.com/ChristopherOlson):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Eugene Shamaev](https://github.com/EShamaev):
  - ***Subsystem***: CAN bus
  - ***Subsystem***: UAVCAN
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Pierre Kancir](https://github.com/khancyr):
  - ***Subsystem***: Copter SITL, Rover SITL
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
