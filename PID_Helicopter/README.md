# PID Helicopter

Our Lab 3 for ECE 4760, running on a Raspberry Pi Pico 2 (RP2350). Lab handout: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html

A motor on the end of an arm lifts the arm around a hinge. An IMU on the arm measures its angle, and a PID controller sets the motor's PWM so the arm goes to the angle we type in over serial.

## Folders

| Folder | Week | What's in it |
|---|---|---|
| [`week1_PWM_Demo`](week1_PWM_Demo) | 1 | Hunter Adams' [PWM_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/PWM/PWM_Demo) with two changes: the PWM starts at 0 (motor off), and it replies with the duty it set. Type a duty cycle (0-6000) over serial and it sets the PWM on GPIO 4 |
| [`week2_IMU_Demo`](week2_IMU_Demo) | 2 | Built from Hunter Adams' [MPU6050_IMU_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/Sensors/MPU6050_IMU_Demo). Arm angle from a complementary filter (calibrated for our IMU), a PID controller in the 1 kHz PWM interrupt, the angle / target / smoothed motor command plotted on the VGA, and serial commands for the target and gains (below) |
| [`week3_Helicopter`](week3_Helicopter) | 3 | Our code. Starts as a copy of the IMU demo; the to-do list for week 3 is at the top of `helicopter.c` |

Both demos have a block added at the top of their `CMakeLists.txt` (the same one as in `Digital_Galton_Board`), so the Pico VS Code extension builds them for the `pico2`. The PWM demo also has the two week 1 changes above. The week 2 folder also has the Pico extension's `.vscode` settings, so opening it on its own gives the Compile and Run buttons.

## Week 2 serial commands

Type a letter and press Enter; if it asks for a number, type it and press Enter again.

| Type | Then | Does |
|---|---|---|
| `a` | angle in degrees | sets the target and turns the controller on (0 = level, -90 = hanging) |
| `p` | a number | sets Kp, straight away |
| `i` | a number | sets Ki |
| `d` | a number | sets Kd |
| `m` | 0-6000 | controller off, motor at that duty |
| `s` | (nothing) | stop: controller off, motor off |
| `t` | a whole number | plot one point every that many ms (default 10; 30 fits the week 3 sequence) |

All gains start at 0, so `a` does nothing until `p` is set.

**Bench numbers (23 Oct):**

- Open loop, about **3600 of 6000** holds the arm level; with the motor off it rests at about -76 deg.
- **Our motor supply (barrel jack) can't deliver above about 4500-5000.** At 5000 open loop the arm can't lift at all. So the controller's output is capped at `MOTOR_CAP` = 4500 in `imu_demo.c`; holding level only needs about 3700. The `m` command is not capped.
- **Best gains found: `p` 100, `d` 12, `i` 80.** From hanging to 0 deg: 2.5 deg overshoot, settled in about 4 s, holds within 0.5 deg. Holds 0, +30 and -30 within 1 deg; the step up to +30 overshoots about 17 deg before settling.
- P alone (`p` 80, `d` 6, `i` 0) settles about 31 deg below the target; that's expected.
- After many full-power runs the motor or its supply heats up and gives less push for the same command. Let it cool between long tuning sessions.

## What each week needs (from the handout)

Each week also has to meet everything from the weeks before.

**Week 1**

- The arm, hinge and motor put together
- The motor circuit soldered on the PCB (not on the breadboard)
- Motor speed set open loop from the PWM output

**Week 2**

- The arm's angle and the low-passed motor command shown on the VGA
- Closed-loop control of the motor (just P is fine)
- Serial commands to set the PID gains

**Week 3**

- Serial commands, at any time, for the target angle, Kp, Ki and Kd, taking effect right away
- One set of gains that's stable over the whole range of hover angles
- Angle and low-passed motor command on the VGA, plus the set angle and gains on the VGA or serial console
- A button (not reset): while held the motor is off and the arm hangs down. On release the target is level, then +30 degrees at 5 s, -30 degrees at 10 s, and level again at 15 s
- Never needs a reset during the demo

## Wiring

| Signal | Pin |
|---|---|
| Serial TX / RX (to a USB-serial adapter, crossed: TX to RX) | GP0 / GP1 |
| PWM to the motor board's PWM terminal | GP4 |
| MPU6050 SDA / SCL | GP8 / GP9 |
| MPU6050 VCC | 3V3 |
| Scope test points: PWM interrupt / VGA frame | GP14 / GP15 |
| VGA Hsync / Vsync | GP16 / GP17 |
| VGA Green low / Green high / Blue / Red | GP18 / GP19 / GP20 / GP21 |
| Motor board MCU GND, IMU GND, VGA GND | GND |

The motor board's 12 V, motor supply and power supply ground stay separate from the Pico. The optoisolator on the board is what connects the two sides.

## Things to know

- **The week 1 code starts the PWM at 0 (motor off).** The course demo started at 3000 out of 6000 (half power), which spins the motor as soon as the Pico boots. Still keep the kill switch off until the Pico is running.
- Serial goes over UART0 (GP0 and GP1), not the USB cable, so you need a USB-serial adapter. On a Mac: `screen /dev/tty.usbserial-XXXX 115200`.
- The PWM is 1 kHz: 150 MHz / 25 (clock divider) / 6000 (wrap). The 4N35 optoisolator is too slow for much faster than that.
- The IMU demo reads the accelerometer in g and the gyro in degrees per second, both as fix15 (15.16 fixed point).
- **IMU calibration (measured 23 Oct, arm held still):** x runs along the arm and points toward the hinge, z points up when the arm is level, y is the hinge axis. The accelerometer reads about +0.07 g too high on x and 0.22 g too low on z, so `imu_demo.c` subtracts those before working out the angle. Re-measure if the IMU is moved or swapped. Angle: 0 = level, -90 = hanging, +90 = straight up.

## Building

Open a week's folder with the Raspberry Pi Pico VS Code extension (board `pico2`), or from a terminal in that folder:

```bash
mkdir build && cd build && cmake .. -DPICO_BOARD=pico2 && make
```

To flash, hold BOOTSEL while plugging in the Pico 2 and copy the `.uf2` from `build/` onto it (`PWM_Demo.uf2`, `MPU6050_IMU.uf2` or `Helicopter.uf2`).
