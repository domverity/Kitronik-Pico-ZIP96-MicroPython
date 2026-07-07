# Kitronik Pico ZIP96

Fork of the helper library for accessing the Kitronic zip96 controller hardware, targeting some efficiency and quality-of-life improvements, e.g. interrupt-based button handling.  

*Note* the software assumes an RP2040 MCU (i.e. Pico 1), as the buttons are wired to use the internal pull-down resistors, which don't work properly on RP2350 (Pico 2)(https://forums.raspberrypi.com/viewtopic.php?t=375631).
