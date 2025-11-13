# CRSF PWM bridge

This code will read serial communications from a ELRS or crossfire receiver and parse/demux the channel data and then pipe
out the PWM control signals (via pin 12 and 13 with the code as is).

This is meant as a simple(ish) way to take an existing off the shelf already setup/bound receiver and be able to use it for controlling planes or RC trucks.

The code will run on an ESP32 dev kit board or equivalent (just about any ESP32 variant that has some open UARTs and a open GPIOs to be used for the PWM output)

## References
https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
https://www.basicpi.org/2018/03/07/esp32-servo-programming-example/
https://learn.sparkfun.com/tutorials/pulse-width-modulation/all
https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/
https://community.platformio.org/t/esp32-board-selection/17331# esp32-elrs-crsf-to-pwm


## myref

### esp32 c6
https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
### serial hardware / software
https://wiki.seeedstudio.com/xiao_pin_multiplexing_esp33c6/#serial-communication

### L9110 motor driver howto
https://www.espboards.dev/blog/motor-control-l9110-esp32/
https://www.amazon.de/L9110S-Controller-gleichzeitig-Dual-Gleichstrommotor-4-Draht-2-Phasen-Schrittmotor/