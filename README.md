# Tekkai 2.0 — Sumo 2026

Current ESP32 firmware for the 2026 Tekkai sumo robot. This source snapshot was taken from the owner's Sumo 2026 PlatformIO project on 29 October 2026. It is separate from the historical [RoboSumo 2024](https://github.com/ChoteX/RoboSumo-2024) implementation.

## Current implementation

- Three ultrasonic channels track opponents with a 40 cm detection threshold. Missing echoes are not treated as targets.
- Two active-low front edge inputs require two consecutive confirmations.
- Remote ON/OFF inputs control the motor-driver enable output. The robot boots OFF.
- An onboard Wi-Fi access point serves sensor readings, motor PWM telemetry, manual ON/OFF controls, and a motor test.
- Serial input adjusts the speed command from 0 to 255.

The main program is `src/sumoFinal_wifi_debug.ino`. `scripts/sumoFinal.ino` and `scripts/motor_pin_control.ino` are separate reference/test sketches, outside PlatformIO's default `src/` build. Do not combine their entry points with the main program.

## Motion decision order

Every nominal 100 ms cycle reads the three ultrasonic sensors, each with a 25 ms echo timeout. This is a configured interval, not a measured control-loop guarantee.

1. If any valid target is closer than 40 cm, follow the target direction and reset the edge-confirmation counters.
2. Only when no ultrasonic target is detected, read the front edge inputs.
3. Confirming both edges starts a 500 ms reverse, followed by a left search. A single edge triggers the corresponding evasive turn.
4. With no target or edge action, search toward the last detected side (left by default).

**Target pursuit takes priority over the edge inputs. This version does not provide unconditional boundary avoidance while pursuing an opponent.** That behavior is preserved from the supplied source, not silently changed in this publication.

## GPIO configuration

Use the code's numeric constants; some inline ultrasonic pin comments refer to an older layout.

| Function | GPIO |
| --- | --- |
| Ultrasonic triggers, left / center / right | 27 / 25 / 32 |
| Ultrasonic echoes, left / center / right | 14 / 26 / 33 |
| Remote ON / OFF | 23 / 22 |
| Motor driver enable | 18, active HIGH |
| Front-left / front-right edge | 13 / 39, active LOW |
| Right motor forward / reverse | 4 / 16 |
| Left motor forward / reverse | 21 / 19 |

GPIO39 uses `INPUT`; provide the external bias required by the actual sensor. GPIO13 uses `INPUT_PULLUP`. Verify motor direction and voltage compatibility against the actual wiring.

## Build

Install PlatformIO, then run:

```sh
pio run -e esp32dev
```

The supplied configuration selects the ESP32 Dev Module and Arduino framework. Arduino, WiFi, and WebServer are supplied by that framework; no additional library dependencies are declared.

Upload only when connected to the intended board:

```sh
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

## Local debug interface

The source configures the access-point name `SUMO_2026_DEBUG` with an empty password. Connect locally and use the access-point address printed in Serial Monitor. The page refreshes telemetry every 200 ms and exposes `/status`, `/on`, `/off`, and `/motor-test` routes. This is a local commissioning interface; avoid treating its manual motor-test mode as autonomous operation.

Compilation verifies the firmware build, not the robot's physical performance, stop latency, or competition result. No competition ranking or measured reliability is claimed here.
