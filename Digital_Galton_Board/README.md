# Digital Galton Board

Our Lab 2 for ECE 4760, running on a Raspberry Pi Pico 2 (RP2350). Lab handout: https://vanhunteradams.com/Pico/Galton/Galton.html

We started from Hunter Adams' [VGA Animation_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics/Animation_Demo). The thunk sound is based on his DMA demo. The core 1 timing and the spinlocks are done the same way as his Mandelbrot and Multicore DDS demos.

## What it does

- 16 rows of pegs (136 pegs) on a VGA screen
- Up to 21,700 balls. Each ball starts at the top with zero y velocity and a small random x velocity, falls with gravity and bounces off the pegs. Once a ball gets past the bottom row of pegs it gets counted in the histogram and starts over at the top
- Runs at 300 MHz (2x overclock) and uses both cores. Each core moves and draws half of the balls
- A "thunk" plays on the MCP4822 DAC (through DMA) when a ball hits a new peg
- Histogram under the board showing where the balls land, scaled to fit the space under the pegs
- Turning the knob changes the selected setting and pushing it switches settings. The settings are the number of balls (100 to 21,700, steps of 100) and bounciness (0.05 to 1.00, steps of 0.05). Changing either one resets the histogram and the fallen count
- The screen shows both settings (a `>` marks the selected one), balls animated, balls fallen since reset, time since boot, frame time, number of late frames, and time on each core
- The LED on the Pico turns on if a frame takes longer than 16,667 us (missed 60 fps)

We used the parameters from Fig. 2 of the handout: gravity 0.37, ball radius 4, peg radius 6, vertical spacing 19, horizontal spacing 38, and bounciness 0.5 at startup. At 0.5 the balls sometimes bounce over whole rows, so the histogram is wider than the ideal binomial. Turning bounciness down to around 0.35 makes it narrower.

All the ball math is fixed point (fix15). We use alpha max plus beta min for the distance to a peg, and each ball only gets checked against the closest peg in the 1 or 2 rows it's next to, not all 136 pegs.

## Changes we made to Hunter's VGA library

To fit more balls in RAM we changed the VGA driver in `VGA/` to 2 colors (1 bit per pixel) instead of 16 colors (4 bits per pixel). That makes each frame buffer 38.4 KB instead of 153.6 KB.

- `rgb.pio` sends 1 bit per pixel to one color pin, using autopull
- `vga16_graphics_v3.c` stores 8 pixels per byte. We changed `drawPixel`, `drawHLine`, the text functions, `clearLowFrame` and `readPixel` to match. Any color except `BLACK` turns the pixel on
- `vga16_graphics_v3.h` has `VGA_COLOR_PIN`, which sets which color pin is used (blue right now)
- `hsync.pio`, `vsync.pio` and `rgb.pio` multiply their clock dividers by `VGA_CLK_MULT` (system clock / 150 MHz), so the pixel clock is still 25 MHz when we overclock to 300 MHz

The rest of the files (fonts, `pt_cornell_rp2040_v1_4.h`, `pico_sdk_import.cmake`) are the same as in Hunter's demo.

## Memory

Each ball takes 20 bytes (x, y, vx, vy, last peg hit), so 21,700 balls use 434 KB. Add the two 38.4 KB frame buffers and that fills up the 512 KB of main RAM. 21,800 balls still builds, but 21,900 gives "region RAM overflowed".

## Wiring

| Signal | Pin |
|---|---|
| Encoder A / B / C (COM) | GP2 / GP3 / GND |
| Encoder switch | GP4 (other side to GND) |
| DAC CS / SCK / SDI | GP5 / GP6 / GP7 |
| VGA Hsync / Vsync | GP16 / GP17 |
| VGA Green low / Green high / Blue / Red | GP18 / GP19 / GP20 / GP21 |
| LED | on the Pico (GP25) |

## Building

Open the folder with the Raspberry Pi Pico VS Code extension (SDK 2.3.1, board `pico2`), or from a terminal:

```bash
mkdir build && cd build && cmake .. -DPICO_BOARD=pico2 && make
```

To flash, hold BOOTSEL while plugging in the Pico 2 and copy `build/VGA_Animation_Demo.uf2` onto it. The LED blinks 10 times (10 seconds) before the board shows up on the screen.
