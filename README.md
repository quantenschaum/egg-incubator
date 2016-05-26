# Egg Incubator

Is an arduino controlled incubator for chicken's eggs. It's purpose is to keep temperature and humidity at defined values, 
so that the eggs are incubated and the chicks finally hatch after some days.

It might as well be used to incubate other things that chicken's egg like other kinds eggs, 
or cultures of bacteria i.e. to make yoghurt.

## Incubation Conditions

Chicken's eggs are incubated for 21 days under temperature and humidity conditions as shown in the table.

| Day   | Temperature [°C] | Humidity [%] |
|:-----:|:----------------:|:------------:|
|  1-17 |      37.8        |    55-50     |
| 18-19 |      37.5        |    55-60     |
| 20-21 |      37.5        |      70      |

Turn the eggs 3 times per day on days 3-17.

The eggs are checked by "x-raying" them with a flashlight on day 7 and 15 
(german [schieren](https://de.wikipedia.org/wiki/Schieren_(Biologie))). 
Dead or unfertilized eggs are sorted out.

## How it works

There are several components attachted to the aruino.

- temperature and humidity sensor (DHT11 or 22)
- heating element (can be switched on and off, i.e. heating wire)
- fan (to distibute the heat equally in the incubator)
- servo (to open and close an air vent for control the humidity)
- buzzer (to sound an alarm on error conditions)

The arduino constantly measures temperature and humidity. The raw measurements are smoothed using 
[Holt-Winters double exponential smoothing](https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing).
The smoothed values are then fed into a [PID control loop](https://en.wikipedia.org/wiki/PID_controller).

### Temperature control

The temperature is maintained by turning the heating element on and off in a 2 seconds cycle. The duty cycle of the heater 
is determined by the temperature PID loop. Using a low frequency pulse width modulation turns a two point controlled 
heating element into a fully modulated element.

As heating element I recommed kanthal heating wire (used for styrofoam cutting). 10-15 watts are enough!
This is much better than a light bulb. It can be operated with low voltage (saver than 220V bulb), does not emit light
and does not burn out. Use a MOSFET to turn it on and off (no heatsink needed due to PWM and no clicking and wearing of a relay).

### Humidity control

The humidity is maintained by opening and closing an air vent on the incubator using a servo. The servo angle is
determined by the humidity PID loop.Inside the incubator is placed a jar filled with water.
The water warms up and evaporates, the humidity rises, the vent is opened and the humid air can escape letting dryer in.
The air vent is needed also to allow fresh air and oxygen to get into the incubator. You have to experiment with the size 
(water surface) of the jar to reach the humidty setpoint and have the air vent half open.

### Fan monitoring

The fan does not need to be controlled, it is constantly running and distributes heat and humidity equally in the incubator.
I used a 12cm 12V PC fan operated at 5V, so it runs slowly.
The arduino monitors the fan using it's rpm signal and sets of an alarm if it fails.

## Setting it up


