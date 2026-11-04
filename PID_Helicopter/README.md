# PID Helicopter

Our Lab 3 for ECE 4760, running on a Raspberry Pi Pico 2 (RP2350). Lab handout: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html

A motor on the end of an arm lifts the arm around a hinge. An IMU on the arm measures its angle, and a PID controller sets the motor's PWM so the arm goes to the angle we type in over serial.

## Folders

| Folder | Week | What's in it |
|---|---|---|
| [`week1_PWM_Demo`](week1_PWM_Demo) | 1 | Hunter Adams' [PWM_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/PWM/PWM_Demo) with two changes: the PWM starts at 0 (motor off), and it replies with the duty it set. Type a duty cycle (0-6000) over serial and it sets the PWM on GPIO 4 |
| [`week2_IMU_Demo`](week2_IMU_Demo) | 2 | Built from Hunter Adams' [MPU6050_IMU_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/Sensors/MPU6050_IMU_Demo). Arm angle from a complementary filter (calibrated for our IMU), a PID controller in the 1 kHz PWM interrupt, the angle / target / smoothed motor command plotted on the VGA, and serial commands for the target and gains (below) |
| [`week2_hover_test`](week2_hover_test) | 2 | Bench tool, not for the checkoff: type a PWM duty (0-6000) over serial like the week 1 demo, and it prints the arm angle twice a second. Used to find the hover duty. `serial_terminal.py` is a serial terminal for the Debug Probe's UART (`python3 serial_terminal.py`), also run by the VS Code task "Serial Terminal" |
| [`week3_Helicopter`](week3_Helicopter) | 3 | The week 3 demo, built on the week 2 code: button sequence, gains already set at power-up, feedforward and a startup ramp (see Week 3 below). Set up for the IRF520 MOSFET and tested on the board on 4 Nov; **+30 deg isn't reached yet** (see Bench numbers, 4 Nov) |

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
| `x` | 0-6000 | sets the controller's motor cap (starts at `MOTOR_CAP` = 5500) |
| `s` | (nothing) | stop: controller off, motor off |
| `t` | a whole number | plot one point every that many ms (default 10; 30 fits the week 3 sequence) |

All gains start at 0, so `a` does nothing until `p` is set.

**Bench numbers (23 Oct):**

- Open loop, about **3600 of 6000** holds the arm level; with the motor off it rests at about -76 deg.
- **Our motor supply (barrel jack) can't deliver above about 4500-5000.** At 5000 open loop the arm can't lift at all. So the controller's output was capped at 4500; holding level only needed about 3700. The `m` command is not capped.
- **MOSFET swapped (23 Oct, evening).** The motor board's IRLB8721 failed (the fan kept running with the motor off, even with the killswitch off) and was replaced with an **IRF520**. The IRF520 needs about 10 V on its gate (the IRLB8721 needs 4.5 V) and loses more voltage when on, so the motor is weaker: holding level now takes about **4760**, and with a cap of 4500 the arm stops around -17 deg. So `MOTOR_CAP` is now **5500**, and `x` changes it while running. With cap 5500 and the gains below: from hanging to 0 deg, 2.8 deg overshoot, settled in about 6.5 s. The IRF520 gets warm and the push fades over many runs; let it cool between runs.
- **Best gains found: `p` 100, `d` 12, `i` 80.** From hanging to 0 deg: 2.5 deg overshoot, settled in about 4 s, holds within 0.5 deg. Holds 0, +30 and -30 within 1 deg; the step up to +30 overshoots about 17 deg before settling.
- P alone (`p` 80, `d` 6, `i` 0) settles about 31 deg below the target; that's expected.
- After many full-power runs the motor or its supply heats up and gives less push for the same command. Let it cool between long tuning sessions.

## Week 3 demo (`week3_Helicopter`)

Flash it, then hold the button and let go: level, +30 deg at 5 s, -30 deg at 10 s, level at 15 s. Holding the button switches the motor off; letting go starts the sequence again, so it never needs a reset. The VGA shows the angle (white), target (yellow) and smoothed motor command (green) over 21 s, plus the target, gains, feedforward and the sequence clock as text.

Same serial commands as week 2, plus:

| Type | Then | Does |
|---|---|---|
| `f` | a number | feedforward gain: the power that holds the arm level (starts at 4700; `0` = plain PID) |
| `x` | 0-6000 | the controller's motor cap (starts at 5800) |

Typing `a` or `s` cancels a running sequence.

Changes from week 2:

- **Gains start at Kp 150, Ki 200, Kd 30**, so the button works straight after power-up.
- **Feedforward.** Holding the arm takes the most power at level (about 3560-3850) and less above and below (it fits 3850 x cos(angle)^1.8). With plain PID the I term carries that power over from the last target, so a step from 0 to +30 overshot 13-21 deg. The controller now adds `ff * cos(aim)^1.8`, and I only fixes what's left (it may go below 0). The gain is 4700 for the IRF520. Bench test with the IRLB8721 (23 Oct): 0 to +30 overshot 1-15 deg (hinge friction makes it vary) and every step settled in under 3 s. This isn't plain PID; say so in the report (`f 0` shows plain PID).
- **Startup ramp.** From hanging, the aim slides up at 60 deg/s instead of jumping, otherwise the arm flew up to about 50 deg past level. Later steps still jump.
- **Plot speed 40 ms per point**, so the whole 20 s sequence fits on the screen.
- **Motor cap 5800** for the IRF520. A step from level to +30 needs about 5700; at 5600 it stopped at +26.
- **Anti-windup.** I stops growing while the motor is at its limit, and it's held during the startup ramp. That removed the overshoot at takeoff and on the +30 step.
- **IMU low-pass filter on** (`mpu6050.c`, register 0x1A, about 44 Hz). Without it, motor vibration got into the angle (see below).

**Bench numbers (4 Nov, IRF520, arm as built):**

- **Final tune: Kp 150, Ki 200, Kd 30, cap 5800.** Three 20 s runs of the demo sequence in a row, sent over serial, each with the arm starting from rest (about -82):

| Step | Within 5 deg | Within 2 deg | Overshoot | End of step |
|---|---|---|---|---|
| Rest to 0 | 1.6-1.7 s | 1.8 s | 0.2-1.1 deg | 0.1 to 0.4 deg |
| 0 to +30 | 0.8 s | 1.0-1.1 s | 0.1-0.3 deg | 30.0 to 30.2 deg |
| +30 to -30 | 0.7 s | 1.3 s | 0.1-0.2 deg | -30.0 to -30.1 deg |
| -30 to 0 | 0.5 s | 0.5-0.6 s | 0.4-0.7 deg | -0.2 to 0.0 deg |

- **Other angles, same gains:** rest to +45 held 45.0 deg; +45 to -45 overshot 13.5 deg, then held -44.6 deg; -45 to +15 held about 14.6 deg.
- **The IMU filter was the big fix.** Before it, the same motor command ended at +17, +31 or never lifted.
- **Everything below was measured without the filter**, so its in-flight angles aren't reliable, and the hinge and supply explanations were never proven. Readings with the arm still are fine. The arm really does rest anywhere from about -65 to -80 deg; the filter doesn't change that.
- Tested with `week2_IMU_Demo` over serial, each run about 14 s with 25 s off, because the IRF520 heats up.
- **Best gains: `p` 200, `i` 200, `d` 20.** Steps down to -30 deg get within 5 deg in about 0.5 s and settle in about 2.5 s; level settled in 0.5 s and held to about 1 deg.
- **The motor cap has a best value.** At 5200 the arm flew straight from rest to +30 deg and held 29.9 deg, within about 1 deg. At 5500 and 5750 it swung about 15 deg around +30. At 6000 it couldn't reach -30 at all and swung between -35 and -110.
- **+30 from level doesn't work yet.** A step from level stalls near +6 at the cap, and the week 3 button sequence stalled near -3. Dipping first only reached about +20.
- **It looks like hinge friction.** -30 needs about 4100 coming from below and 3450 from above, +30 holds on 4920 but won't climb on 5200, and the arm rests anywhere from -64 to -79 instead of -90. Clean and oil the hinge, then check the wires aren't dragging. A stronger motor supply would also help. Retune `x` and `f` afterwards.
- **Button sequence:** tested by forcing GPIO 11 through the Debug Probe, because the button isn't wired yet. Holding keeps the motor off, releasing starts it, and the targets change on time.

Button: GPIO 11 (physical pin 15) to GND (pin 18). No resistor: the code turns on the Pico's internal pull-up. It's debounced: the reading must hold for 30 ms. Not the reset button, which goes from RUN to GND.

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

Week 3 wiring. `week2_IMU_Demo` is the same, without the button.

| Signal | Pin |
|---|---|
| Serial TX / RX (Debug Probe U port, crossed: TX to RX) | GP0 / GP1 (pins 1, 2) |
| Debug Probe D port | Pico debug connector |
| PWM to the motor board's PWM terminal | GP4 (pin 6) |
| MPU6050 3.3 V (driven by the code, so it can power-cycle the IMU) | GP10 (pin 14) |
| Week 3 button (other leg to GND) | GP11 (pin 15) |
| MPU6050 SDA / SCL | GP12 / GP13 (pins 16, 17) |
| Scope test points: PWM interrupt / VGA frame | GP14 / GP15 |
| VGA Hsync / Vsync | GP16 / GP17 |
| VGA Green low / Green high / Blue / Red | GP18 / GP19 / GP20 / GP21 |
| Motor board MCU GND, IMU GND, VGA GND, button | GND |
| Reset button (other leg to GND) | RUN (pin 30) |

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
