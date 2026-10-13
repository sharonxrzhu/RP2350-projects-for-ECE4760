# Digital Galton Board

ECE 4760 Lab 2 ([handout](https://vanhunteradams.com/Pico/Galton/Galton.html)) on the RP2350 (Raspberry Pi Pico 2).
Status: final (Week 3).

Started from Hunter Adams' [VGA Animation_Demo](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics/Animation_Demo).
The thunk sound is based on his DMA demo, and the core-to-core timing and the spinlocks on his Mandelbrot and Multicore DDS demos.

## What it does

- 16-row Galton board (136 pegs) drawn on VGA
- Up to 21,700 balls. They drop from the top with zero y-velocity and a small random x-velocity, fall under gravity and bounce off the pegs. Once they clear the bottom row of pegs they're counted and dropped again from the top
- 300 MHz (2x overclock) with both cores: each core moves and draws half of the balls every frame
- A DMA-driven "thunk" plays on the MCP4822 DAC each time a ball hits a new peg
- A histogram under the board counts where balls land, normalized to the space beneath the bottom row of pegs
- Rotary encoder: turning the knob adjusts the selected parameter, and pushing it selects the next one. The parameters are the ball count (100 to 21,700, steps of 100) and bounciness (0.05 to 1.00, steps of 0.05). Changing either one resets the histogram and the fallen count
- On screen: both parameters (the selected one is marked with `>`), balls animated, balls fallen since reset, time since boot, frame compute time, number of late frames, and time per core
- The on-board LED lights when a frame takes longer than 16,667 µs (misses 60 fps)

Physics uses the handout's Fig. 2 parameters (gravity 0.37, ball radius 4, peg radius 6, vertical separation 19, horizontal separation 38), except bounciness starts at 0.35 instead of 0.5. At 0.5 balls bounce over rows and the histogram comes out much wider than the ideal binomial.
All ball math is fixed-point (fix15). The ball-to-peg distance uses the alpha-max-plus-beta-min approximation, and each ball is only checked against the nearest peg in the one or two rows it's next to, not against all 136.

## Changes to Hunter's VGA library

To fit this many balls in RAM, the VGA driver in `VGA/` runs in 2 colours (1 bit a pixel) instead of 16 colours (4 bits a pixel). Each frame buffer shrinks from 153.6 KB to 38.4 KB.

- `rgb.pio` sends 1 bit per pixel to one colour pin, using autopull
- `vga16_graphics_v3.c` packs 8 pixels a byte. `drawPixel`, `drawHLine`, the text functions, `clearLowFrame` and `readPixel` are changed to match. Any colour other than `BLACK` lights the pixel
- `vga16_graphics_v3.h` adds `VGA_COLOR_PIN`, which picks the colour pin (currently blue)
- `hsync.pio`, `vsync.pio` and `rgb.pio` multiply their clock dividers by `VGA_CLK_MULT` (system clock / 150 MHz), so the pixel clock stays at 25 MHz at 300 MHz

The other files (fonts, `pt_cornell_rp2040_v1_4.h`, `pico_sdk_import.cmake`) are unchanged from Hunter's demo.

## Memory

Each ball takes 20 bytes (x, y, vx, vy, last peg hit), so 21,700 balls take 434 KB. Together with the two 38.4 KB frame buffers, that fills the 512 KB of main RAM. 21,800 still links, but 21,900 fails with "region RAM overflowed".

## Wiring

| Signal | Pin |
|---|---|
| Encoder A / B / C (COM) | GP2 / GP3 / GND |
| Encoder switch | GP4 (other side to GND) |
| DAC CS / SCK / SDI | GP5 / GP6 / GP7 |
| VGA Hsync / Vsync | GP16 / GP17 |
| VGA Green low / Green high / Blue / Red | GP18 / GP19 / GP20 / GP21 |
| LED | on-board (GP25) |

## Build

Open the folder with the Raspberry Pi Pico VS Code extension (SDK 2.3.1, board `pico2`), or:

```bash
mkdir build && cd build && cmake .. -DPICO_BOARD=pico2 && make
```

Then hold BOOTSEL while plugging in the Pico 2, and copy `build/VGA_Animation_Demo.uf2` onto it.
After flashing, the LED blinks 10 times (10 seconds) before the board appears on the screen.
