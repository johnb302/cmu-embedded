# cmu-embedded

Zephyr firmware for a small RC car on an **STM32 Nucleo-F401RE**.

A Raspberry Pi reads a steering wheel + pedals and sends commands over UART.
The Nucleo turns those into motor speed, steering, and blinkers — and stops the car safely if the Pi goes quiet.

```
 Steering wheel / pedals
          │
          ▼
   Raspberry Pi ──UART (13-byte frames)──▶ Nucleo F401RE ──▶ motors, servo, blinkers
          ▲                                     │
          └────────── status frames ────────────┘  (current sensors, error state)
```

## File tree

```
.
├── CMakeLists.txt          # What gets compiled (normal firmware or a calibration build)
├── prj.conf                # Zephyr features to enable (GPIO, PWM, ADC, UART, logging)
├── prj_lab3.conf           # Zephyr features for Lab 3 (CAN)
├── zephyr-env.sh           # Sets paths to the local Zephyr SDK/toolchain in .tools/
├── boards/
│   ├── nucleo_f401re.overlay       # Pin map: motors, encoders, servo, blinkers, ADC, UART
│   └── nucleo_f401re_lab3.overlay  # Lab 3 pin map: MCP2515 CAN over SPI
├── src/
│   ├── config.h            # ★ Every tunable number: timings, PID gains, pedal ranges, priorities
│   ├── main.c              # ★ Start here. Receives Pi frames, runs the link watchdog
│   ├── motor_control.c/h   # PID speed loop: pedals → target RPM → PWM
│   ├── motor_driver.c/h    # Low-level L298N H-bridge (drive / brake / coast)
│   ├── encoder.c/h         # Wheel encoders → position + RPM
│   ├── servo_driver.c/h    # Steering servo: steering value → pulse width
│   ├── blinker.c/h         # Turn signals and hazards
│   ├── wheel_buttons.c/h   # Debounces steering-wheel buttons
│   ├── status_tx.c/h       # Sends status frames back to the Pi
│   └── crc8_checksum.h     # CRC-8 shared with the Pi
├── src_lab3/               # Lab 3 dual CAN-bus firmware (built with -DCAN_LAB=ON)
│   ├── lab3.c              # Entry point: N2 steering / N3 drivetrain, chosen by NODE_ID
│   ├── can_bus.c/h         # MCP2515 dual-bus CAN: filters, TX/RX, stall recovery
│   ├── zone_state.c/h      # This zone's OK/ERROR state machine
│   ├── heartbeat.c/h       # Heartbeat TX + monitoring of other zones
│   ├── watchdog.c/h        # Independent watchdog (IWDG), fed only while tasks make progress
│   ├── control.c/h, blink.c/h, button.c/h, n2_tasks.c/h
│   └── tasks.h             # Per-node thread priorities
├── tools/
│   ├── telemetry/          # Live USB dashboard (Python + Matplotlib), logs CSV
│   ├── motor_calibration/  # Separate firmware + script to sweep motor PWM
│   └── servo_calibration/  # Separate firmware + script to find servo endpoints
├── Lab2_Writeup.pdf        # Lab 2 report
└── tests.yaml              # Leftover from the Zephyr blinky sample
```

★ = read these first.

## How the firmware works

Everything runs as Zephyr threads. Lower number = higher priority.

| Priority | Thread         | Job |
| -------: | -------------- | --- |
| 0 | motor          | Every 10 ms: read encoders, run PID, set PWM |
| 1 | fail_safe      | Wakes only if the Pi stops talking → brake + hazards |
| 2 | cmd_handler    | One run per Pi frame: check CRC, apply steer/throttle/brake |
| 3 | status_tx      | Every 20 ms: send current readings + state to the Pi |
| 4 | blinker        | Flashes turn signals / hazards |

One command's path:

1. **UART interrupt** (`main.c`) assembles a 13-byte frame and queues it (only the newest is kept).
2. **cmd_handler** checks the CRC and value ranges, then hands throttle/brake to `motor_control` and steering to `servo_driver`.
3. **Link watchdog**: each good frame restarts a timer. Three missed 50 ms intervals → **fail-safe** (brake, steering off, hazards on).
4. Bad or out-of-range values also trigger fail-safe. The car powers up in fail-safe until the first valid frame.

Frame layout (both directions): `SOF(0xA5) | ID(2) | DLC | data(8) | CRC`.
Command data = steer, throttle, brake (int16 LE), buttons, sequence number.

## Build and flash

```bash
source .venv/bin/activate          # also loads zephyr-env.sh
west build -b nucleo_f401re -p     # normal firmware
west flash
```

Calibration firmware (pick one):

```bash
west build -b nucleo_f401re -p -- -DMOTOR_CALIBRATION=ON
west build -b nucleo_f401re -p -- -DSERVO_CALIBRATION=ON
```

Lab 3 CAN firmware (`src_lab3/`), one build per node:

```bash
west build -b nucleo_f401re -p -d build-lab3-n2 -- -DCAN_LAB=ON -DNODE_ID=2 \
  -DCONF_FILE=prj_lab3.conf -DDTC_OVERLAY_FILE=boards/nucleo_f401re_lab3.overlay
```

Watch it run:

```bash
pip install -r tools/telemetry/requirements.txt
python -u tools/telemetry/telemetry.py --port /dev/ttyACM0
```

## Where to change things

| I want to…                         | Edit |
| ---------------------------------- | ---- |
| Tune PID, max RPM, timeouts        | `src/config.h` |
| Change which pin does what         | `boards/nucleo_f401re.overlay` |
| Turn a Zephyr driver on/off        | `prj.conf` |
| Change frame format                | `src/config.h` **and** the Pi sender |

Motor measurement notes live in `tools/motor_calibration/MEASUREMENTS.md`.
