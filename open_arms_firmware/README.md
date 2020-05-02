# open_arms_firmware

This package contains libraries and sketches to be loaded to the robot's Arduino Mega.

## Libraries

The following libraries are included in this package:

- Encoder.h: used for reading encoders. This library was developed by Paul Stoffregen and you can find the latest version [here] (https://github.com/PaulStoffregen/Encoder).

- open_arms_common.h: stores pin numbers for each motor or encoder as well as some constants. Change this header file if you want to connect your electronics to a different pin on the Arduino. Note that the encoder pins were selected such that each encoder is connected to 1 interrupt pin.

- OpenArmsFB.h: used to access feedback, i.e. encoders, of the robot. Usually, you interact with this library directly and it communicates with Encoder.h to pass all the necessary information.

- OpenArmsStepper.h: a library used to control stepper motors with software PWM signal
  
- ros.h: a library to enable communication between ROS and Arduino. Users can also install the latest version through the Arduino IDE

## Using the Package

Remeber to set the sketchbook location of your Arduino IDE to this package in order to use the libraries.