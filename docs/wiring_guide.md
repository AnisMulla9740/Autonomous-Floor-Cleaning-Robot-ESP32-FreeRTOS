Wiring Guide for Autonomous Floor Cleaning Robot Components List ESP32
DOIT DevKit V1

Cytron MD10C Motor Drivers (2x)

HC-SR04 Ultrasonic Sensors (3x)

DC Motors with Encoders (2x)

Cleaning Mechanism Motor (1x)

12V Li-ion Battery Pack

Emergency Stop Button

Status LED (optional)

Voltage Divider Circuit (2x 100kΩ resistors)

Power Connections Battery to Motor Drivers text 12V Battery+ → Cytron
MD10C-1 VCC 12V Battery+ → Cytron MD10C-2 VCC 12V Battery- → Cytron
MD10C-1 GND 12V Battery- → Cytron MD10C-2 GND ESP32 Power text 12V
Battery+ → 5V Regulator → ESP32 5V pin 12V Battery- → ESP32 GND pin
Motor Connections Left Motor to Cytron MD10C-1 text Left Motor+ → Cytron
MD10C-1 M1A Left Motor- → Cytron MD10C-1 M1B Right Motor to Cytron
MD10C-2 text Right Motor+ → Cytron MD10C-2 M1A Right Motor- → Cytron
MD10C-2 M1B Cleaning Motor text Cleaning Motor+ → 12V Battery+ (via
MOSFET) Cleaning Motor- → MOSFET Drain MOSFET Source → 12V Battery-
MOSFET Gate → ESP32 GPIO23 ESP32 to Cytron MD10C Connections Left Motor
Control (Cytron MD10C-1) text ESP32 GPIO16 → Cytron MD10C-1 PWM ESP32
GPIO17 → Cytron MD10C-1 DIR Right Motor Control (Cytron MD10C-2) text
ESP32 GPIO18 → Cytron MD10C-2 PWM ESP32 GPIO19 → Cytron MD10C-2 DIR
Sensor Connections Front Ultrasonic Sensor text ESP32 GPIO26 → HC-SR04
TRIG ESP32 GPIO25 → HC-SR04 ECHO ESP32 5V → HC-SR04 VCC ESP32 GND →
HC-SR04 GND Left Ultrasonic Sensor text ESP32 GPIO13 → HC-SR04 TRIG
ESP32 GPIO12 → HC-SR04 ECHO ESP32 5V → HC-SR04 VCC ESP32 GND → HC-SR04
GND Right Ultrasonic Sensor text ESP32 GPIO14 → HC-SR04 TRIG ESP32
GPIO27 → HC-SR04 ECHO ESP32 5V → HC-SR04 VCC ESP32 GND → HC-SR04 GND
Encoder Connections Left Motor Encoder text Encoder A Phase → ESP32
GPIO34 Encoder B Phase → Not connected (or to ESP32 GPIO35 if using
quadrature) Encoder VCC → ESP32 3.3V Encoder GND → ESP32 GND Right Motor
Encoder text Encoder A Phase → ESP32 GPIO35 Encoder B Phase → Not
connected Encoder VCC → ESP32 3.3V Encoder GND → ESP32 GND Battery
Monitoring Circuit Voltage Divider text 12V Battery+ → 100kΩ Resistor →
ESP32 GPIO36 ESP32 GPIO36 → 100kΩ Resistor → 12V Battery- Emergency Stop
Button text ESP32 GPIO4 → Emergency Button → ESP32 GND (Use internal
pull-up on GPIO4) Status LED text ESP32 GPIO2 → 220Ω Resistor → LED+ →
LED- → ESP32 GND Cleaning Mechanism Control Brush Motor (if separate)
text ESP32 GPIO23 → MOSFET Gate MOSFET Drain → Brush Motor- MOSFET
Source → 12V Battery- Brush Motor+ → 12V Battery+ Vacuum Motor (if
applicable) text ESP32 GPIO5 → Relay Control Relay NC/NO → Vacuum Motor
Complete Wiring Diagram text +---------------+ \| 12V BATTERY \| \|
+-------+ \| \| \| \| \| \| \| \| +-----+ +-----------------+ \|
+-------+ \| \| \| Cytron MD10C \| \| \| \| +---\>\| VCC M1A \|---\>
Left Motor+ \| \| \| \| \| GND M1B \|---\> Left Motor- \| \| \| \| \|
PWM \<--GPIO16 \| \| \| \| \| \| DIR \<--GPIO17 \| \| \| \| \|
+-----------------+ \| \| \| \| \| \| \| \| +-----------------+ \| \| \|
\| \| Cytron MD10C \| \| \| \| +---\>\| VCC M1A \|---\> Right Motor+ \|
\| \| \| \| GND M1B \|---\> Right Motor- \| \| \| \| \| PWM \<--GPIO18
\| \| \| \| \| \| DIR \<--GPIO19 \| \| \| \| \| +-----------------+ \|
\| \| \| \| \| \| +---\> 5V Regulator ---\> ESP32 5V \| \| \| \| \| \|
\| +---\> Ultrasonic Sensors VCC \| \| \| \| \| \| \| +---\> Encoders
VCC \| \| \| \| \| +---------\> ESP32 GND \| \| \| \| \| +---\>
Ultrasonic Sensors GND \| \| \| \| \| +---\> Encoders GND \| \| \| \| \|
+---\> Cytron MD10C GND \| \| \| +-----------------\> Voltage Divider
---\> ESP32 GPIO36 \| +-----------------\> Emergency Stop Button ---\>
ESP32 GPIO4 Important Notes Power Considerations:

The Cytron MD10C can handle up to 30V and 10A per channel

Ensure your battery can supply enough current for all motors

Use appropriate wire gauge for motor connections (18-20 AWG recommended)

Voltage Divider:

The voltage divider reduces the 12V battery voltage to a safe level for
the ESP32 ADC

Calculation: Vout = Vin \* (R2/(R1+R2)) = 12V \* (100k/(100k+100k)) = 6V

Note: This is still above the ESP32's 3.3V maximum. Use a different
resistor ratio:

For 12V to 3.3V: R1 = 270kΩ, R2 = 100kΩ

Vout = 12V \* (100k/(270k+100k)) ≈ 3.24V

Encoder Connections:

GPIO34 and GPIO35 are input-only pins on ESP32, perfect for encoders

Enable internal pull-up resistors in code

Motor Direction:

Test motor directions and swap M1A/M1B if rotation is opposite to
expected

Alternatively, modify the DIR pin logic in code

Ultrasonic Sensors:

Ensure sensors have a clear field of view

Mount sensors at appropriate height for obstacle detection

Emergency Stop:

The button should be easily accessible

Consider adding a software debounce in the ISR

Cleaning Mechanism:

Use a MOSFET or relay capable of handling the motor current

Add a flyback diode across the motor terminals for protection

Testing Procedure Test power supply connections with a multimeter

Test each motor individually without load

Verify ultrasonic sensor readings with known distances

Test encoder counting by manually rotating wheels

Verify emergency stop functionality

Test Bluetooth connectivity and command reception

This wiring guide provides a comprehensive connection diagram for your
autonomous floor cleaning robot. Always double-check connections before
applying power, and consider adding fuses or current limiting devices
for safety.
