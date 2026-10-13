/**
 * Digital Galton Board - ECE 4760 Lab 2
 * Raspberry Pi Pico 2 (RP2350)
 *
 * We started from Hunter Adams' VGA Animation_Demo (vha3@cornell.edu).
 * The thunk sound comes from his DMA demo, the core 1 timing through the
 * FIFO is how his Mandelbrot demo does it, and the spinlock is from his
 * Multicore DDS demo.
 *
 * What it does:
 *  - 16 rows of pegs (136 pegs total)
 *  - up to 21700 balls drop from the top. it starts at 21700 and each
 *    click of the knob changes it by 100
 *  - runs at 300 MHz (2x overclock) and uses both cores, each core
 *    moves and draws half of the balls
 *  - balls start with vy = 0 and a small random vx, then gravity pulls
 *    them down
 *  - all the ball math is fixed point (fix15). each ball only gets
 *    checked against the pegs in the rows right next to it
 *  - bounce physics from the handout pseudocode, with the Fig. 2 numbers
 *    (bounciness starts at 0.5 and the knob can change it)
 *  - balls are drawn as 2x2 dots because it's way faster than circles,
 *    but the physics still uses radius 4
 *  - VGA is 2 colors (1 bit per pixel): black, and whatever color pin
 *    VGA_COLOR_PIN is set to in vga16_graphics_v3.h. this makes each frame
 *    buffer 4x smaller, which leaves room in RAM for more balls
 *  - DMA plays a "thunk" when a ball hits a new peg
 *  - a ball gets counted and sent back to the top once it gets past the
 *    bottom row of pegs
 *  - histogram of where the balls land, scaled to fit the space under
 *    the board. bar heights get recalculated 10 times a second
 *  - the knob changes whichever setting is selected (ball count or
 *    bounciness) and the button switches between them. changing either
 *    one resets the histogram and the fallen count
 *  - screen shows both settings (the selected one has a > next to it),
 *    balls animated, balls fallen since reset, time since boot, how long
 *    the frame took, how many frames were late, and time on each core
 *  - the LED on the Pico turns on if a frame takes longer than 16667 us
 *    (that means we missed 60 fps)
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 2  ---> Rotary encoder A
 *  - GPIO 3  ---> Rotary encoder B   (encoder C/COM ---> GND)
 *  - GPIO 4  ---> Encoder switch     (other switch pin ---> GND)
 *  - GPIO 5  ---> DAC CS
 *  - GPIO 6  ---> DAC SCK
 *  - GPIO 7  ---> DAC SDI (MOSI)
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
 *  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
 *  - RP2350 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0 (VGA)
 *  - DMA channels: 2 for VGA, 1 for sound (claim mechanism)
 *  - DMA timer 0 (sound sample rate)
 *  - SPI0 (DAC)
 *  - 2 hardware spinlocks (sound, histogram)
 *  - 2 x 38.4 kBytes of RAM for the VGA buffers, plus 20 bytes per
 *    ball (434 kBytes for 21700 balls)
 */

// Include the VGA grahics library
#include "VGA/vga16_graphics_v3.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/ioqspi.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)((a) << 15))
#define fix2int15(a) ((int)((a) >> 15))
#define divfix(a,b) ((fix15)((((signed long long)(a)) << 15) / (b)))


// ==================================================
// === rotary encoder
// ==================================================
#define ENCODER_A 2
#define ENCODER_B 3

// the interrupt just counts clicks into encoder_delta (+1 for clockwise,
// -1 for counterclockwise). once per frame the animation thread takes
// those clicks and applies them to whatever setting is selected (see
// applyEncoderClicks), so the interrupt doesn't care what mode we're in

// every ball needs 20 bytes of RAM (x, y, vx, vy, last_peg are 4 bytes
// each). 21700 balls is 434 KB, and with the two 38.4 KB VGA buffers
// that's pretty much all of the 512 KB of main RAM. we tried building
// with more: 21800 still fits but 21900 gives "region RAM overflowed"
//
// one click changes the count by BALL_STEP so you don't have to turn
// the knob 21700 times. we start at the max
#define BALL_STEP    100
#define MIN_BALLS    BALL_STEP
#define MAX_BALLS    21700
#define START_BALLS  MAX_BALLS

volatile int encoder_delta = 0 ;
volatile uint32_t last_encoder_time = 0 ;

// runs on the falling edge of A
void encoder_callback(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32() ;

    // debounce: ignore edges less than 3 ms apart
    if ((current_time - last_encoder_time) < 3000) return ;
    last_encoder_time = current_time ;

    // when A falls, B tells us the direction:
    // B high = clockwise, B low = counterclockwise
    if (gpio_get(ENCODER_B)) encoder_delta++ ;
    else encoder_delta-- ;
}


// ==================================================
// === encoder push button + LED
// ==================================================
// the button is the switch built into the encoder. one side goes to
// GP4 and the other to GND. we use the internal pull-up so pressed = LOW
// the LED is the one on the Pico (GP25). it turns on when we miss 60 fps
#define BUTTON_PIN 4
#define LED_PIN    PICO_DEFAULT_LED_PIN

// one late frame only lasts 16 ms which is too fast to see, so we keep
// the LED on for 30 frames (half a second) after a late frame
#define LED_HOLD_FRAMES 30

// how long one frame is at 60 fps, in us
#define FRAME_BUDGET_US 16667


// ==================================================
// === SPI DAC (from Hunter's DMA demo)
// ==================================================
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000


// ==================================================
// === thunk sound
// ==================================================
// Hunter's DMA demo plays one period of a sine wave (256 samples) over
// and over. for the thunk we made a longer wave that fades out, and we
// only play it once each time a ball hits a peg
#define SOUND_SAMPLES 2048

// DAC samples per second. the DMA demo set the timer to 0x17/0xffff of
// the clock, which is 52.6 kHz at 150 MHz. initAudio figures it out from
// the actual clock instead so the pitch stays the same when we overclock
#define SOUND_SAMPLE_RATE 52643

// the values DMA sends straight to the DAC
unsigned short sound_buffer[SOUND_SAMPLES] ;

// the DMA channel for the sound
int audio_dma_chan ;

// spinlock for playThunk, since both cores call it
spin_lock_t *thunk_lock ;

// fills sound_buffer with the thunk
void buildSoundBuffer()
{
    for (int i = 0; i < SOUND_SAMPLES; i++) {
        // envelope goes from 1 down to 0 across the buffer so the
        // sound dies away instead of being a flat beep
        float envelope = 1.0f - ((float)i / (float)SOUND_SAMPLES) ;

        // 18 periods of a sine wave across the whole buffer
        // (more periods = higher pitch)
        float wave = sinf(2.0f * 3.14159265f * 18.0f * ((float)i / (float)SOUND_SAMPLES)) ;

        // swing around the middle of the DAC range (2048)
        int sample = 2048 + (int)(1800.0f * envelope * wave) ;

        // keep it inside 12 bits just in case
        if (sample < 0) sample = 0 ;
        if (sample > 4095) sample = 4095 ;

        // add the MCP4822 config bits on top
        sound_buffer[i] = DAC_config_chan_A | (sample & 0x0fff) ;
    }
}

// sets up SPI and the DMA channel for the sound
void initAudio()
{
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, SPI_MSB_FIRST) ;

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI) ;

    // make the thunk
    buildSoundBuffer() ;

    // Claim and initialize a spinlock
    thunk_lock = spin_lock_init(spin_lock_claim_unused(true)) ;

    // Select DMA channels
    audio_dma_chan = dma_claim_unused_channel(true) ;

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(audio_dma_chan) ;  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16) ;                 // 16-bit txfers
    channel_config_set_read_increment(&c2, true) ;                            // yes read incrementing
    channel_config_set_write_increment(&c2, false) ;                          // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // we use X = 1, Y = sys_clk / SOUND_SAMPLE_RATE
    dma_timer_set_fraction(0, 1, clock_get_hz(clk_sys) / SOUND_SAMPLE_RATE) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b) ;                                      // DREQ paced by timer 0

    // we don't have the control channel from the DMA demo. the thunk
    // should play once and stop, and playThunk restarts it on each hit
    dma_channel_configure(
        audio_dma_chan,             // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        sound_buffer,               // The initial read address
        SOUND_SAMPLES,              // Number of transfers
        false                       // Don't start immediately.
    ) ;
}

// called every time a ball hits a new peg, from either core.
// both cores could hit a peg at the same time, so we take the spinlock
// first. that way one core finishes restarting the DMA before the other
// one touches the DMA registers
void playThunk()
{
    uint32_t irq_state = spin_lock_blocking(thunk_lock) ;

    // with lots of balls the hits come faster than one thunk lasts, so
    // stop whatever is playing and start over so every hit makes a sound
    dma_channel_abort(audio_dma_chan) ;
    // go back to the start of the sound
    dma_channel_set_read_addr(audio_dma_chan, sound_buffer, false) ;
    // reload the sample count and start it
    dma_channel_set_trans_count(audio_dma_chan, SOUND_SAMPLES, true) ;

    spin_unlock(thunk_lock, irq_state) ;
}


// ==================================================
// === Galton board parameters (handout Fig. 2)
// ==================================================
#define BALL_RADIUS 4
#define PEG_RADIUS  6

// row 0 has 1 peg, row 1 has 2, ... row 15 has 16
// so 1 + 2 + ... + 16 = 136 pegs
#define NUM_ROWS 16
#define NUM_PEGS (NUM_ROWS * (NUM_ROWS + 1) / 2)

// where the top peg is
#define PEG_X 320
#define PEG_Y 80

// spacing between pegs
#define PEG_VERTICAL_SEPARATION    19
#define PEG_HORIZONTAL_SEPARATION  38

// when we need more balls we add this many per frame instead of all at
// once. 21700 balls takes 868 frames (about 14.5 s) to fill up. the
// random vx spreads them out as they come in
#define BALLS_PER_FRAME 25

// screen edges
#define LEFT_EDGE      8
#define RIGHT_EDGE     632
#define TOP_EDGE       20
#define BOTTOM_EDGE    475

// gets added to vy every frame (positive y is down on the VGA screen)
#define GRAVITY float2fix15(0.37f)

// a ball is touching a peg if their centers are closer than this
#define COLLISION_DISTANCE (BALL_RADIUS + PEG_RADIUS)

// alpha max plus beta min, so we can get the distance without doing a
// square root:
// distance ~= ALPHA * max(|dx|,|dy|) + BETA * min(|dx|,|dy|)
// with these two numbers the error is less than about 4%
#define ALPHA float2fix15(0.960433870103f)
#define BETA  float2fix15(0.397824734759f)

// bounciness is how much of the ball's speed is kept when it hits a new
// peg (from the handout pseudocode). 1.0 means no energy lost and 0.0
// means the ball stops. the knob changes bounce_percent by BOUNCE_STEP
// and bounciness is the same number in fix15 for the physics
//
// starts at 0.5 like Fig. 2. turning it down to around 0.35 makes the
// histogram narrower, since balls bounce over fewer rows
#define START_BOUNCE_PERCENT 50
#define BOUNCE_STEP           5

int   bounce_percent = START_BOUNCE_PERCENT ;
fix15 bounciness     = float2fix15(START_BOUNCE_PERCENT / 100.0f) ;


// ==================================================
// === peg positions
// ==================================================
short peg_x[NUM_PEGS] ;
short peg_y[NUM_PEGS] ;

// fills in peg_x and peg_y, one row at a time
void buildPegs()
{
    int i = 0 ;
    for (int row = 0; row < NUM_ROWS; row++) {
        for (int col = 0; col <= row; col++) {
            // each row starts half a spacing more to the left than the
            // row above it, so every peg ends up under a gap
            peg_x[i] = PEG_X - row * (PEG_HORIZONTAL_SEPARATION / 2) + col * PEG_HORIZONTAL_SEPARATION ;
            peg_y[i] = PEG_Y + row * PEG_VERTICAL_SEPARATION ;
            i++ ;
        }
    }
}


// ==================================================
// === ball state
// ==================================================
fix15 ball_x[MAX_BALLS] ;
fix15 ball_y[MAX_BALLS] ;
fix15 ball_vx[MAX_BALLS] ;
fix15 ball_vy[MAX_BALLS] ;

// which peg each ball hit last (-1 = none yet). we only play a thunk
// for a NEW peg, otherwise a ball sitting on a peg for a few frames
// would make a bunch of sounds
int last_peg[MAX_BALLS] ;

// how many balls are on the screen right now. this catches up to
// num_balls BALLS_PER_FRAME at a time
int balls_released = 0 ;


// ==================================================
// === histogram
// ==================================================
#define NUM_BINS (NUM_ROWS + 1)     // 16 rows -> 17 bins
int histogram[NUM_BINS] ;           // how many balls landed in each bin
int total_fallen = 0 ;              // balls counted since the last reset

// both cores add to the histogram, so we lock before adding
// (same idea as the counter in Hunter's Multicore DDS demo)
spin_lock_t *hist_lock ;

// we only recalculate the bar heights every 6 frames (60 / 6 = 10 times
// a second) instead of every frame
#define HIST_UPDATE_INTERVAL 6
static int cached_heights[NUM_BINS] ;
static int hist_frame_counter = 0 ;

// the bars go in the space between the bottom row of pegs and the
// bottom of the screen. bottom row of pegs is at y = 80 + 15*19 = 365,
// plus the peg radius and a little gap, so a full bar reaches y = 375.
// bars sit on HIST_BOTTOM (y = 463), which leaves 12 px underneath for
// the count numbers
#define BOTTOM_PEG_Y (PEG_Y + (NUM_ROWS - 1) * PEG_VERTICAL_SEPARATION)
#define HIST_BOTTOM (BOTTOM_EDGE - 12)
#define MAX_BAR_HEIGHT (HIST_BOTTOM - (BOTTOM_PEG_Y + PEG_RADIUS + 4))


// ==================================================
// === spawn / respawn a ball
// ==================================================
void spawnBall(int b)
{
    // start at the top middle
    ball_x[b] = int2fix15(320) ;
    ball_y[b] = int2fix15(30) ;

    // random vx between -0.5 and +0.5 pixels per frame. in fix15 that's
    // -16384 to +16383, so 32768 different values.
    // it needs a lot of different values. balls don't hit each other, so
    // two balls with the same vx take the exact same path and land in
    // the same bin. when we only had 100 values the histogram was jagged
    // and some bins were empty. with 32768 it comes out smooth
    ball_vx[b] = (rand() % 32768) - 16384 ;

    // don't let vx be exactly 0. a ball with vx = 0 lands right on top
    // of the first peg and bounces straight up and down forever
    if (ball_vx[b] == 0) ball_vx[b] = 1 ;

    // the handout says balls start with zero y velocity
    ball_vy[b] = 0 ;

    last_peg[b] = -1 ;
}


// ==================================================
// === move a ball
// ==================================================
void updateBallPosition(int b)
{
    // move by the velocity
    ball_x[b] += ball_vx[b] ;
    ball_y[b] += ball_vy[b] ;
    // gravity
    ball_vy[b] += GRAVITY ;
}


// ==================================================
// === peg collision
// ==================================================
// the handout pseudocode checks every ball against all 136 pegs, which
// is way too slow for this many balls. rows are 19 px apart, so a ball
// can only be touching the 1 or 2 rows that are within
// COLLISION_DISTANCE of it. pegs in a row are 38 px apart, which is more
// than twice COLLISION_DISTANCE, so in each of those rows only the
// closest peg matters. that's 2-3 pegs per ball instead of 136
void checkPegCollision(int b)
{
    int x = fix2int15(ball_x[b]) ;
    int y = fix2int15(ball_y[b]) ;

    // how far down the ball is from the top row of pegs
    int rel_y = y - PEG_Y ;

    // still above the board, nothing to hit
    if (rel_y + COLLISION_DISTANCE + 1 < 0) return ;

    // first and last row to check. the +1 is for the part of a pixel we
    // lose when we turn y into an int. rel_y - COLLISION_DISTANCE can be
    // negative and C division rounds negatives toward 0, so we clamp it
    int first_row = (rel_y - COLLISION_DISTANCE) / PEG_VERTICAL_SEPARATION ;
    int last_row = (rel_y + COLLISION_DISTANCE + 1) / PEG_VERTICAL_SEPARATION ;
    if (rel_y - COLLISION_DISTANCE < 0) first_row = 0 ;
    if (last_row > NUM_ROWS - 1) last_row = NUM_ROWS - 1 ;

    for (int row = first_row; row <= last_row; row++) {
        // x of the leftmost peg in this row
        int row_left = PEG_X - row * (PEG_HORIZONTAL_SEPARATION / 2) ;

        // closest peg in this row (rounded, and kept inside the row)
        int offset = x - row_left + PEG_HORIZONTAL_SEPARATION / 2 ;
        int col = (offset < 0) ? 0 : offset / PEG_HORIZONTAL_SEPARATION ;
        if (col > row) col = row ;

        // pegs are stored row by row, and row r starts at r(r+1)/2
        int p = row * (row + 1) / 2 + col ;

        // distance from the peg center to the ball center
        fix15 dx = ball_x[b] - int2fix15(peg_x[p]) ;
        fix15 dy = ball_y[b] - int2fix15(peg_y[p]) ;

        // quick box check first, like in the pseudocode
        fix15 abs_dx = absfix15(dx) ;
        fix15 abs_dy = absfix15(dy) ;
        if (abs_dx >= int2fix15(COLLISION_DISTANCE) || abs_dy >= int2fix15(COLLISION_DISTANCE)) continue ;

        // distance using alpha max plus beta min
        fix15 distance = (abs_dx > abs_dy)
            ? multfix15(ALPHA, abs_dx) + multfix15(BETA, abs_dy)
            : multfix15(ALPHA, abs_dy) + multfix15(BETA, abs_dx) ;
        if (distance >= int2fix15(COLLISION_DISTANCE)) continue ;

        // === if we get here the ball hit peg p ===

        // don't divide by zero
        if (distance == 0) distance = 1 ;

        // normal vector, points from the peg to the ball
        fix15 normal_x = divfix(dx, distance) ;
        fix15 normal_y = divfix(dy, distance) ;

        // from the pseudocode: intermediate = -2 * (n . v)
        // n . v is negative when the ball is moving toward the peg.
        // adding intermediate * n flips the part of the velocity that
        // points into the peg. energy loss happens further down, and only
        // for a new peg
        fix15 normal_dot_v = multfix15(normal_x, ball_vx[b]) + multfix15(normal_y, ball_vy[b]) ;
        fix15 intermediate = -2 * normal_dot_v ;

        // move the ball just outside the peg first. otherwise it can stay
        // inside the peg and keep colliding every frame
        ball_x[b] = int2fix15(peg_x[p]) + multfix15(normal_x, int2fix15(COLLISION_DISTANCE + 1)) ;
        ball_y[b] = int2fix15(peg_y[p]) + multfix15(normal_y, int2fix15(COLLISION_DISTANCE + 1)) ;

        // if the ball is already moving away from the peg (it can still be
        // overlapping for a frame), don't bounce it. bouncing it would send
        // it right back into the peg
        if (normal_dot_v >= 0) return ;

        // bounce: flip the part of the velocity going into the peg
        ball_vx[b] += multfix15(normal_x, intermediate) ;
        ball_vy[b] += multfix15(normal_y, intermediate) ;

        // new peg: lose some energy and play a thunk. we scale both vx
        // and vy, otherwise balls keep speeding up sideways as they go
        // down the board
        if (last_peg[b] != p) {
            ball_vx[b] = multfix15(bounciness, ball_vx[b]) ;
            ball_vy[b] = multfix15(bounciness, ball_vy[b]) ;
            playThunk() ;
            last_peg[b] = p ;
        }

        // a ball can only touch one peg at a time so we can stop here
        return ;
    }
}


// ==================================================
// === walls, counting and respawning
// ==================================================
void checkScreenEdges(int b)
{
    int x = fix2int15(ball_x[b]) ;
    int y = fix2int15(ball_y[b]) ;

    // left wall
    if (x < LEFT_EDGE + BALL_RADIUS) {
        ball_x[b] = int2fix15(LEFT_EDGE + BALL_RADIUS) ;
        ball_vx[b] = -ball_vx[b] ;
    }
    // right wall
    if (x > RIGHT_EDGE - BALL_RADIUS) {
        ball_x[b] = int2fix15(RIGHT_EDGE - BALL_RADIUS) ;
        ball_vx[b] = -ball_vx[b] ;
    }
    // top wall (a ball can bounce up off the top peg)
    if (y < TOP_EDGE + BALL_RADIUS && ball_vy[b] < 0) {
        ball_y[b] = int2fix15(TOP_EDGE + BALL_RADIUS) ;
        ball_vy[b] = -ball_vy[b] ;
    }

    // once a ball is past the bottom row of pegs we count it and send it
    // back to the top. we used to wait until it hit the bottom of the
    // screen, but then it drifts sideways another ~100 px which messes up
    // the histogram, and it wastes time on a ball that can't hit anything
    if (y > BOTTOM_PEG_Y + COLLISION_DISTANCE) {
        // which bin? bin k is centered at x = 16 + 38*k, halfway between
        // two pegs in the bottom row
        int bin = (x + 3) / 38 ;
        if (bin < 0) bin = 0 ;
        if (bin > NUM_BINS - 1) bin = NUM_BINS - 1 ;

        // both cores count balls and could add to the same bin at the
        // same time, which could lose a count, so we lock first
        // Lock spinlock (without disabling interrupts)
        spin_lock_unsafe_blocking(hist_lock) ;
        histogram[bin]++ ;
        total_fallen++ ;
        // Unlock spinlock
        spin_unlock_unsafe(hist_lock) ;

        spawnBall(b) ;
    }
}


// ==================================================
// === draw pegs and histogram
// ==================================================
// (the balls get drawn by both cores in animateBalls before this runs)
void drawGaltonScene()
{
    // draw all 136 pegs
    for (int p = 0; p < NUM_PEGS; p++) {
        fillCircle(peg_x[p], peg_y[p], PEG_RADIUS, BLUE) ;
    }

    // recalculate the bar heights every 6 frames
    if (++hist_frame_counter >= HIST_UPDATE_INTERVAL) {
        hist_frame_counter = 0 ;

        // find the tallest bin
        int tallest = 0 ;
        for (int k = 0; k < NUM_BINS; k++) {
            if (histogram[k] > tallest) tallest = histogram[k] ;
        }

        // scale everything so the tallest bin is MAX_BAR_HEIGHT tall
        for (int k = 0; k < NUM_BINS; k++) {
            cached_heights[k] = (tallest > 0) ? (int)((int64_t)histogram[k] * MAX_BAR_HEIGHT / tallest) : 0 ;
        }
    }

    // draw the bars as outlines
    for (int k = 0; k < NUM_BINS; k++) {
        int height = cached_heights[k] ;
        if (height <= 0) continue ;

        int left = -3 + 38 * k ;
        int width = 36 ;

        // the first bar would start off the left side of the screen
        if (left < 0) {
            width += left ;
            left = 0 ;
        }

        int top = HIST_BOTTOM - height ;
        drawHLine(left, top, width, GREEN) ;                // top
        drawVLine(left, top, height, GREEN) ;               // left side
        drawVLine(left + width - 1, top, height, GREEN) ;   // right side
    }

    // write the count under each bin
    for (int k = 0; k < NUM_BINS; k++) {
        char count_text[12] ;
        int len = sprintf(count_text, "%d", histogram[k]) ;

        // center the number under the bin (each character is 6 px wide)
        int x = 16 + 38 * k - 3 * len ;
        if (x < 0) x = 0 ;

        drawTextAscii(x, HIST_BOTTOM + 3, count_text, WHITE, BLACK) ;
    }
}


// ==================================================
// === user interface: modes, button, encoder clicks
// ==================================================
// what the knob is changing right now. pressing the button goes to the
// next one and wraps back around to the first
enum { MODE_BALLS, MODE_BOUNCINESS, NUM_MODES } ;

int mode = MODE_BALLS ;

// how many balls the knob is asking for
int num_balls = START_BALLS ;

// clears the histogram and the fallen count
void resetCounts()
{
    for (int k = 0; k < NUM_BINS; k++) histogram[k] = 0 ;
    total_fallen = 0 ;
}

// checked once per frame. a press is when the pin goes from HIGH to LOW.
// presses less than 200 ms apart are just the switch bouncing so we
// ignore them
void checkButton()
{
    static bool was_pressed = false ;
    static uint32_t last_press_time = 0 ;

    bool pressed = !gpio_get(BUTTON_PIN) ;
    uint32_t now = time_us_32() ;

    if (pressed && !was_pressed && (now - last_press_time) > 200000) {
        mode = (mode + 1) % NUM_MODES ;
        last_press_time = now ;
    }
    was_pressed = pressed ;
}

// takes the clicks the encoder interrupt counted since the last frame
// and applies them to the selected setting. if the setting actually
// changed we reset the histogram and fallen count (handout requirement)
void applyEncoderClicks()
{
    // read and clear with interrupts off, so if a click comes in right
    // in the middle we don't lose it
    uint32_t irq_state = save_and_disable_interrupts() ;
    int clicks = encoder_delta ;
    encoder_delta = 0 ;
    restore_interrupts(irq_state) ;

    if (clicks == 0) return ;

    bool changed = false ;

    if (mode == MODE_BALLS) {
        int n = num_balls + clicks * BALL_STEP ;
        if (n > MAX_BALLS) n = MAX_BALLS ;
        if (n < MIN_BALLS) n = MIN_BALLS ;
        changed = (n != num_balls) ;
        num_balls = n ;
    }
    else if (mode == MODE_BOUNCINESS) {
        int pct = bounce_percent + clicks * BOUNCE_STEP ;
        if (pct > 100) pct = 100 ;
        if (pct < BOUNCE_STEP) pct = BOUNCE_STEP ;
        changed = (pct != bounce_percent) ;
        bounce_percent = pct ;
        bounciness = int2fix15(pct) / 100 ;
    }

    if (changed) resetCounts() ;
}


// ==================================================
// === ball drawing
// ==================================================
// we draw each ball as a 2x2 dot even though the physics treats it as
// radius 4. a radius 4 fillCircle is a lot more drawing per ball
void drawBall(int x, int y)
{
    // (drawPixel skips pixels that are off the screen)
    drawPixel(x,     y,     WHITE) ;
    drawPixel(x + 1, y,     WHITE) ;
    drawPixel(x,     y + 1, WHITE) ;
    drawPixel(x + 1, y + 1, WHITE) ;
}


// ==================================================
// === both cores: each one does half of the balls
// ==================================================
// what happens every frame:
//   core 0: clear the screen, read the knob, add new balls
//   core 0: tells core 1 to go (through the FIFO)
//   core 0 does balls [0, split) while core 1 does balls [split, n)
//   core 0: waits for core 1 to say it's done. core 1 sends back how
//           long it took, like core 1 does in Hunter's Mandelbrot demo
//   core 0: draws the pegs, bars and text
//
// things both cores share: the histogram (hist_lock), the thunk DMA
// (thunk_lock), and the frame buffer. if two balls from different cores
// land in the same byte of the frame buffer at the same time, one pixel
// can get lost for one frame, but you can't see that so we left it

// which balls core 1 does this frame (core 0 sets these)
volatile int core1_first = 0 ;
volatile int core1_last  = 0 ;

// move, collide, count and draw balls first to last-1
void animateBalls(int first, int last)
{
    for (int b = first; b < last; b++) {
        updateBallPosition(b) ;
        checkPegCollision(b) ;
        checkScreenEdges(b) ;
        // if the ball just got past the bottom row it was respawned,
        // so this draws it back at the top
        drawBall(fix2int15(ball_x[b]), fix2int15(ball_y[b])) ;
    }
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
    while (1) {
        // wait for core 0 to say go
        multicore_fifo_pop_blocking() ;

        uint32_t begin_time = time_us_32() ;
        animateBalls(core1_first, core1_last) ;

        // tell core 0 we're done and how long it took (us)
        multicore_fifo_push_blocking(time_us_32() - begin_time) ;
    }
}


// ==================================================
// === animation thread on core 0
// ==================================================
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt) ;

    static char encoder_text[40] ;
    static char bounce_text[40] ;
    static char time_text[40] ;
    static char hist_text[80] ;
    static char frame_text[40] ;
    static char core_text[40] ;

    // how many frames missed 60 fps since boot, and how many more frames
    // the LED should stay on
    static int late_frames = 0 ;
    static int led_frames_left = 0 ;

    // how long the last frame took (physics + drawing), in us
    static uint32_t frame_time_us = 0 ;

    // how long each core spent on its balls, in us
    static uint32_t core0_us = 0 ;
    static uint32_t core1_us = 0 ;

    while (1) {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;

        // start timing this frame
        uint32_t frame_start = time_us_32() ;

        // Clear the buffer
        clearLowFrame(0, BLACK) ;

        // check the button and the knob
        checkButton() ;
        applyEncoderClicks() ;

        // make the number of balls match the knob.
        // fewer balls: the highest numbered balls just go away
        if (balls_released > num_balls) balls_released = num_balls ;

        // more balls: add BALLS_PER_FRAME this frame until we have enough
        for (int i = 0; i < BALLS_PER_FRAME && balls_released < num_balls; i++) {
            spawnBall(balls_released) ;
            balls_released++ ;
        }

        // physics, collisions, walls, respawning and drawing the balls,
        // split between the two cores
        int split = balls_released / 2 ;
        core1_first = split ;
        core1_last  = balls_released ;

        // tell core 1 to go
        multicore_fifo_push_blocking(1) ;

        uint32_t core0_start = time_us_32() ;
        animateBalls(0, split) ;
        core0_us = time_us_32() - core0_start ;

        // wait for core 1 to finish (it sends back its time)
        core1_us = multicore_fifo_pop_blocking() ;

        // draw the pegs and the histogram
        drawGaltonScene() ;

        // the two settings. the one the knob is changing has a > in front.
        // it's also drawn in ORANGE, but with only 2 colors every color
        // except BLACK looks the same, so the > is what you actually see
        sprintf(encoder_text, "%c Balls: %d (animated %d)",
                (mode == MODE_BALLS) ? '>' : ' ', num_balls, balls_released) ;
        drawTextAscii(20, 20, encoder_text, (mode == MODE_BALLS) ? ORANGE : WHITE, BLACK) ;

        sprintf(bounce_text, "%c Bounciness: %d.%02d",
                (mode == MODE_BOUNCINESS) ? '>' : ' ', bounce_percent / 100, bounce_percent % 100) ;
        drawTextAscii(20, 35, bounce_text, (mode == MODE_BOUNCINESS) ? ORANGE : WHITE, BLACK) ;

        drawTextAscii(20, 50, "Knob: adjust  Button: next", CYAN, BLACK) ;

        // time since boot in seconds
        sprintf(time_text, "Time since boot: %d s", (int)(time_us_64() / 1000000)) ;
        drawTextAscii(20, 70, time_text, MAGENTA, BLACK) ;

        sprintf(hist_text, "Fallen since reset: %d", total_fallen) ;
        drawTextAscii(20, 85, hist_text, GREEN, BLACK) ;

        // frame time compared to the 60 fps budget (16667 us). if it gets
        // close to that the animation starts slowing down. this shows the
        // previous frame's time since this frame isn't done yet
        sprintf(frame_text, "Frame: %lu us  Late: %d", (unsigned long)frame_time_us, late_frames) ;
        drawTextAscii(20, 100, frame_text, YELLOW, BLACK) ;

        // time on each core, these should be about the same
        sprintf(core_text, "Core0: %lu us  Core1: %lu us", (unsigned long)core0_us, (unsigned long)core1_us) ;
        drawTextAscii(20, 115, core_text, YELLOW, BLACK) ;

        // how long this whole frame took
        frame_time_us = time_us_32() - frame_start ;

        // missed 60 fps? turn the LED on and keep it on for
        // LED_HOLD_FRAMES frames so we can actually see it
        if (frame_time_us > FRAME_BUDGET_US) {
            late_frames++ ;
            led_frames_left = LED_HOLD_FRAMES ;
        }

        if (led_frames_left > 0) {
            gpio_put(LED_PIN, 1) ;
            led_frames_left-- ;
        }
        else {
            gpio_put(LED_PIN, 0) ;
        }
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt) ;
} // animation thread


// ==================================================
// === overclocking: 150 MHz -> 300 MHz
// ==================================================
// we picked 300 MHz because it's exactly 2x the normal 150 MHz. it has
// to be a multiple of 150 MHz because the VGA PIO programs count clock
// cycles. we multiply their clock dividers by VGA_CLK_MULT (see the .pio
// files) so the monitor still gets a 25 MHz pixel clock
//
// two other things have to happen before we speed up:
//  1. raise the core voltage from 1.10 V to 1.30 V so the chip is stable
//     at 2x speed
//  2. slow down the flash. the flash chip (where the program is stored)
//     is rated for 133 MHz. its clock is sys_clk / CLKDIV, and with the
//     default divider it would end up at 150 MHz, so we set CLKDIV = 4
//     (75 MHz) first
// these are the same settings from our ballmax test (ninaa26/ballmax),
// which ran at 300 MHz on a Pico 2
#define SYS_CLOCK_KHZ 300000

// this has to run from RAM, because the chip can't read its program out
// of flash while we're changing the flash timing
static void __no_inline_not_in_flash_func(slowFlashClock)(void)
{
    // wait until the flash isn't selected (chip select is high)
    while ((ioqspi_hw->io[1].status & IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS)
           != IO_QSPI_GPIO_QSPI_SS_STATUS_OUTTOPAD_BITS) {
        tight_loop_contents() ;
    }

    qmi_hw->m[0].timing =
          (1u << QMI_M0_TIMING_COOLDOWN_LSB)
        | (2u << QMI_M0_TIMING_PAGEBREAK_LSB)
        | (7u << QMI_M0_TIMING_MIN_DESELECT_LSB)
        | (2u << QMI_M0_TIMING_RXDELAY_LSB)
        | (4u << QMI_M0_TIMING_CLKDIV_LSB) ;

    // reading from flash once makes the new timing kick in
    (void)*(volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE ;
}

void overclock()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30) ;
    busy_wait_us(10000) ;        // give the voltage time to settle
    slowFlashClock() ;
    set_sys_clock_khz(SYS_CLOCK_KHZ, true) ;
}


// ========================================
// === main
// ========================================
int main()
{
    // overclock to 300 MHz
    overclock() ;

    // initialize stio
    stdio_init_all() ;

    // encoder A and B are inputs with pull-ups
    gpio_init(ENCODER_A) ;
    gpio_set_dir(ENCODER_A, GPIO_IN) ;
    gpio_pull_up(ENCODER_A) ;

    gpio_init(ENCODER_B) ;
    gpio_set_dir(ENCODER_B, GPIO_IN) ;
    gpio_pull_up(ENCODER_B) ;

    // interrupt when A goes low
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &encoder_callback) ;

    // encoder button (pressed = LOW)
    gpio_init(BUTTON_PIN) ;
    gpio_set_dir(BUTTON_PIN, GPIO_IN) ;
    gpio_pull_up(BUTTON_PIN) ;

    // LED on the Pico
    gpio_init(LED_PIN) ;
    gpio_set_dir(LED_PIN, GPIO_OUT) ;

    // blink 10 times at startup (takes 10 seconds)
    for (int i = 0; i < 10; i++) {
        gpio_put(LED_PIN, 1) ;
        sleep_ms(500) ;
        gpio_put(LED_PIN, 0) ;
        sleep_ms(500) ;
    }

    // initialize VGA
    initVGA() ;

    // sound
    initAudio() ;

    // Claim and initialize a spinlock
    hist_lock = spin_lock_init(spin_lock_claim_unused(true)) ;

    // where all the pegs go
    buildPegs() ;

    // start core 1
    multicore_reset_core1() ;
    multicore_launch_core1(&core1_main) ;

    // add threads
    pt_add_thread(protothread_anim) ;

    // start scheduler
    pt_schedule_start ;
}
