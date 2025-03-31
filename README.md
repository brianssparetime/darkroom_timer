# Darkroom Timer project


**Looking back, this code is shit that I'm ashamed I wrote.  Do not use it as a model for anything.  If you want a much better model, look at [PlantWaterBot](https://github.com/brianssparetime/PlantWaterBot), [DarkroomRoller](https://github.com/brianssparetime/darkroom_roller), or the barebones [example FSM for UI on Arduino](https://github.com/brianssparetime/UI_FSM_example)**


This project creates a usable darkroom timer that can:
 - control turning on and off a safelight and enlarger
 - has a settable timer for keeping the enlarger on, with manual override or cancel
 - uses a backlit LED display (2 line, 16 characters), except when the timer is running and the enlarger is on, in which case the time is shown on a red 4 digit 7 segment display (TM1637)
 - uses a rotary encoder to control functions, set timer
 - has a buzzer/beeper for indicating end of timer



# Resources for v2 

## better (hardware?) debounce for rotary encoders
https://arduino.stackexchange.com/questions/61861/will-a-simple-rc-filter-work-with-my-mechanical-rotary-encoder-or-do-i-need-a-s
https://forum.arduino.cc/t/rotary-encoder-debouncing/361438/11
http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html



