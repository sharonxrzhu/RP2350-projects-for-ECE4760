# Digital Galton Board

ECE 4760 Lab 2 ([handout](https://vanhunteradams.com/Pico/Galton/Galton.html)) on the RP2350 (Pico 2).
Status: Week 2 checkpoint.

## What it does

- 16-row Galton board (136 pegs) drawn on VGA
- Balls are dropped from the top with zero y-velocity and a small random x-velocity, fall under gravity, bounce off pegs, and respawn when they leave the bottom
- The rotary encoder sets how many balls are animated (1–150, starts at 10); new balls are released one every 20 frames
- A DMA-driven "thunk" plays on the MCP4822 DAC each time a ball hits a new peg
- A histogram under the board counts where balls land, normalized to the space beneath the bottom row of pegs
- On screen: balls animated, balls fallen since reset, time since boot, and how long each frame's physics + drawing took (budget: 16667 us at 60 fps)

Physics uses the handout's Fig. 2 parameters (gravity 0.37, ball radius 4, peg radius 6, vertical separation 19, horizontal separation 38), except bounciness is 0.35 instead of 0.5. At 0.5 balls bounce over rows and the histogram comes out much wider than the ideal binomial.
All ball math is fixed-point (fix15). The ball-to-peg distance uses the alpha-max-plus-beta-min approximation, and each ball is only checked against the nearest peg in the one or two rows it is next to, not against all 136.

## Wiring

| Signal | Pin |
|---|---|
| Encoder A / B / COM | GP2 / GP3 / GND |
| DAC CS / SCK / MOSI | GP5 / GP6 / GP7 |
| VGA Hsync / Vsync | GP16 / GP17 |
| VGA Green low / Green high / Blue / Red | GP18 / GP19 / GP20 / GP21 |

## Build

Open the folder with the Raspberry Pi Pico VS Code extension (SDK 2.3.1, board `pico2`), or:

```bash
mkdir build && cd build && cmake .. -DPICO_BOARD=pico2 && make
```

Then copy `VGA_Animation_Demo.uf2` to the Pico 2.

## Not done yet (Week 3)

- Using both cores and finding the maximum ball count at 60 fps
- Encoder push-button to switch between ball count and bounciness
- Resetting the histogram and fallen count whenever a parameter changes
- LED on when the 60 fps deadline is missed
