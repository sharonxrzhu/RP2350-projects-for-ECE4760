/**
 * Digital Galton Board -- ECE 4760 Lab 2 (final)
 * Raspberry Pi Pico 2 (RP2350)
 *
 * Started from Hunter Adams' VGA Animation_Demo (vha3@cornell.edu).
 * The thunk sound is based on his DMA demo, the core-to-core
 * timing on his Mandelbrot demo, and the spinlock on his
 * Multicore DDS demo.
 *
 * Features:
 *  - 16-row board of pegs (136 pegs)
 *  - Up to 21700 balls dropped from the top (starts at 21700;
 *    the knob changes it 100 at a time)
 *  - 300 MHz (overclocked 2x) and both cores: each core moves
 *    and draws half the balls
 *  - Initial vy = 0, small random vx, gravity
 *  - Fixed-point (fix15) physics; each ball is only tested
 *    against the pegs in the rows it's near
 *  - Bounce physics with the Fig. 2 parameters, except
 *    bounciness starts at 0.35 instead of 0.5
 *  - Balls are drawn as 2x2 dots to save drawing time;
 *    the physics still uses radius 4
 *  - 2-colour VGA (1 bit a pixel): black, and the colour of
 *    one VGA pin (VGA_COLOR_PIN in vga16_graphics_v3.h). The
 *    frame buffers are a quarter of their 16-colour size, so
 *    the RAM goes to balls instead
 *  - DMA-driven "thunk" when a ball hits a new peg
 *  - Balls are counted and respawned at the top as soon as
 *    they clear the bottom row of pegs
 *  - Histogram of where balls land, normalized to the space
 *    under the board; bar heights recomputed 10 times a second
 *  - Knob adjusts the selected parameter (ball count or
 *    bounciness); the button selects the next one. Changing
 *    either resets the histogram and the fallen count.
 *  - Display: both parameters (selected one marked with >),
 *    balls animated, balls fallen since reset, time since
 *    boot, frame compute time, late frames, time per core
 *  - On-board LED lights when a frame takes longer than
 *    16667 us (misses 60 fps)
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
 *  - 2 x 38.4 kBytes of RAM for the VGA buffers, and
 *    20 bytes of RAM per ball (434 kBytes for 21700 balls)
 */

// Include the VGA graphics library
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

// The interrupt only counts clicks (+1 clockwise, -1 counter-
// clockwise) into encoder_delta. Once a frame, the display thread
// applies them to whichever parameter is selected (see
// applyEncoderClicks), so the interrupt doesn't need to know
// about modes.

// Each ball takes 20 bytes of RAM (x, y, vx, vy, last_peg).
// 21700 balls = 434 KB, which fits next to the two 38.4 KB VGA
// buffers in the 512 KB of main RAM. 21800 would also fit, but
// 21900 doesn't link ("region RAM overflowed").
//
// One click changes the count by BALL_STEP, so the full range is
// 217 clicks instead of 21700. The board starts at the maximum.
#define BALL_STEP    100
#define MIN_BALLS    BALL_STEP
#define MAX_BALLS    21700
#define START_BALLS  MAX_BALLS

volatile int encoder_delta = 0 ;
volatile uint32_t last_encoder_time = 0 ;

// GPIO interrupt on the falling edge of A
void encoder_callback(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32() ;

    // Debounce: ignore edges closer than 3 ms
    if ((current_time - last_encoder_time) < 3000) return ;
    last_encoder_time = current_time ;

    // At the falling edge of A:
    // B HIGH -> clockwise, B LOW -> counterclockwise
    if (gpio_get(ENCODER_B)) encoder_delta++ ;
    else encoder_delta-- ;
}


// ==================================================
// === encoder push button + LED
// ==================================================
// The switch built into the encoder: one side to GP4, the other
// to GND (internal pull-up, so pressed = LOW).
// The LED is the Pico's on-board LED (GP25). It turns on when a
// frame misses the 60 fps deadline.
#define BUTTON_PIN 4
#define LED_PIN    PICO_DEFAULT_LED_PIN

// After a missed deadline, keep the LED on this many frames
// (30 = half a second) so you can see it
#define LED_HOLD_FRAMES 30

// Time for one frame at 60 fps (us)
#define FRAME_BUDGET_US 16667


// ==================================================
// === SPI DAC (from Hunter Adams' DMA demo)
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
// Hunter's DMA demo plays one period of a sine wave (256
// samples) forever. For a thunk we build a longer wave whose
// amplitude dies away, and play it once per hit.
#define SOUND_SAMPLES 2048

// DAC samples per second. The DMA demo used 0x17/0xffff of the
// 150 MHz clock (52.6 kHz); initAudio works it out from the real
// clock instead, so overclocking doesn't change the pitch.
#define SOUND_SAMPLE_RATE 52643

// Values DMA will send directly to the DAC
unsigned short sound_buffer[SOUND_SAMPLES] ;

// DMA channel used for sound
int audio_dma_chan ;

// Spinlock for playThunk, which both cores call
spin_lock_t *thunk_lock ;

// Build the thunk waveform
void buildSoundBuffer()
{
    for (int i = 0; i < SOUND_SAMPLES; i++) {
        // Envelope falls from 1 to 0, so the sound
        // decays instead of being a constant beep
        float envelope = 1.0f - ((float)i / (float)SOUND_SAMPLES) ;

        // 18 sine periods across the whole buffer
        // (more periods = higher pitch)
        float wave = sinf(2.0f * 3.14159265f * 18.0f * ((float)i / (float)SOUND_SAMPLES)) ;

        // Oscillate around the DAC midpoint (2048)
        int sample = 2048 + (int)(1800.0f * envelope * wave) ;

        // Clamp to 12 bits
        if (sample < 0) sample = 0 ;
        if (sample > 4095) sample = 4095 ;

        // Add the MCP4822 command bits
        sound_buffer[i] = DAC_config_chan_A | (sample & 0x0fff) ;
    }
}

// Set up the SPI channel and the sound DMA channel
void initAudio()
{
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, SPI_MSB_FIRST) ;

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI) ;

    // Build the thunk
    buildSoundBuffer() ;

    // Claim and initialize a spinlock (both cores call playThunk)
    thunk_lock = spin_lock_init(spin_lock_claim_unused(true)) ;

    // Select a DMA channel
    audio_dma_chan = dma_claim_unused_channel(true) ;

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(audio_dma_chan) ;  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16) ;                 // 16-bit txfers
    channel_config_set_read_increment(&c2, true) ;                            // yes read incrementing
    channel_config_set_write_increment(&c2, false) ;                          // no write incrementing
    // (X/Y)*sys_clk: one sample every sys_clk / SOUND_SAMPLE_RATE cycles
    dma_timer_set_fraction(0, 1, clock_get_hz(clk_sys) / SOUND_SAMPLE_RATE) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b) ;                                      // DREQ paced by timer 0

    // Unlike the DMA demo there is no control channel: the thunk
    // plays once, and playThunk restarts it on every new hit.
    dma_channel_configure(
        audio_dma_chan,             // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        sound_buffer,               // The initial read address
        SOUND_SAMPLES,              // Number of transfers
        false                       // Don't start immediately.
    ) ;
}

// Called on every new-peg hit, from either core. Both cores can
// hit a peg at the same moment, so the restart is done under the
// spinlock: one core finishes before the other starts, and they
// never mix up the DMA registers.
void playThunk()
{
    uint32_t irq_state = spin_lock_blocking(thunk_lock) ;

    // Hits are often less than one thunk apart: stop any thunk
    // still playing and start again from the beginning, so every
    // new hit is heard
    dma_channel_abort(audio_dma_chan) ;
    // Back to the start of the sound
    dma_channel_set_read_addr(audio_dma_chan, sound_buffer, false) ;
    // Reload the number of samples and start
    dma_channel_set_trans_count(audio_dma_chan, SOUND_SAMPLES, true) ;

    spin_unlock(thunk_lock, irq_state) ;
}


// ==================================================
// === Galton board parameters (handout Fig. 2)
// ==================================================
#define BALL_RADIUS 4
#define PEG_RADIUS  6

// Row 0 has 1 peg, row 1 has 2 pegs, ... row 15 has 16 pegs.
// Total = 1 + 2 + ... + 16 = 136 pegs
#define NUM_ROWS 16
#define NUM_PEGS (NUM_ROWS * (NUM_ROWS + 1) / 2)

// Top peg
#define PEG_X 320
#define PEG_Y 80

// Peg grid spacing
#define PEG_VERTICAL_SEPARATION    19
#define PEG_HORIZONTAL_SEPARATION  38

// When balls are added, release this many new balls each frame:
// 21700 balls pour in over 868 frames (about 14.5 s). Their
// random vx spreads them out.
#define BALLS_PER_FRAME 25

// Screen boundaries
#define LEFT_EDGE      8
#define RIGHT_EDGE     632
#define TOP_EDGE       20
#define BOTTOM_EDGE    475

// Added to the vertical velocity every frame
// (positive y points down on VGA)
#define GRAVITY float2fix15(0.37f)

// A ball touches a peg when their centres are closer than this
#define COLLISION_DISTANCE (BALL_RADIUS + PEG_RADIUS)

// Alpha max plus beta min: approximates sqrt(dx^2 + dy^2)
// without a square root:
// distance ~= ALPHA * max(|dx|,|dy|) + BETA * min(|dx|,|dy|)
// These constants keep the error under about 4%.
#define ALPHA float2fix15(0.960433870103f)
#define BETA  float2fix15(0.397824734759f)

// Bounciness: fraction of the ball's speed kept each time it hits
// a NEW peg (handout pseudocode). 1.0 = no energy lost, 0.0 = ball
// stops dead. The knob changes bounce_percent in steps of
// BOUNCE_STEP; bounciness is the same value in fix15 for the
// physics.
#define START_BOUNCE_PERCENT 35
#define BOUNCE_STEP           5

int   bounce_percent = START_BOUNCE_PERCENT ;
fix15 bounciness     = float2fix15(START_BOUNCE_PERCENT / 100.0f) ;


// ==================================================
// === peg positions
// ==================================================
short peg_x[NUM_PEGS] ;
short peg_y[NUM_PEGS] ;

void buildPegs()
{
    int i = 0 ;
    for (int row = 0; row < NUM_ROWS; row++) {
        for (int col = 0; col <= row; col++) {
            // Each row starts half a spacing further left than the
            // row above, so every peg sits under the gap between
            // two pegs of the row above
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

// Index of the last peg each ball hit (-1 = none yet).
// A thunk plays only when a ball hits a NEW peg, so one peg
// doesn't make several sounds over consecutive frames.
int last_peg[MAX_BALLS] ;

// How many balls are on screen right now.
// This follows num_balls, BALLS_PER_FRAME at a time.
int balls_released = 0 ;


// ==================================================
// === histogram
// ==================================================
#define NUM_BINS (NUM_ROWS + 1)     // 16 rows -> 17 bins
int histogram[NUM_BINS] ;           // balls that landed in each bin
int total_fallen = 0 ;              // balls counted since reset

// Both cores add to the histogram, so the adds are done under a
// spinlock (like the counter in Hunter's Multicore DDS demo)
spin_lock_t *hist_lock ;

// Bar heights are recomputed every 6 frames (60 fps / 6 = 10 fps)
#define HIST_UPDATE_INTERVAL 6
static int cached_heights[NUM_BINS] ;
static int hist_frame_counter = 0 ;

// The bars use the space between the bottom row of pegs and the
// bottom of the screen. Bottom row of pegs: y = 80 + 15*19 = 365;
// plus the peg radius and a small gap, bars can reach up to
// y = 375. They stop at HIST_BOTTOM (y = 463), leaving a 12 px
// strip under them for each bin's count.
#define BOTTOM_PEG_Y (PEG_Y + (NUM_ROWS - 1) * PEG_VERTICAL_SEPARATION)
#define HIST_BOTTOM (BOTTOM_EDGE - 12)
#define MAX_BAR_HEIGHT (HIST_BOTTOM - (BOTTOM_PEG_Y + PEG_RADIUS + 4))


// ==================================================
// === spawn / respawn a ball
// ==================================================
void spawnBall(int b)
{
    // Start near the centre-top
    ball_x[b] = int2fix15(320) ;
    ball_y[b] = int2fix15(30) ;

    // Random x velocity in fix15: one of 32768 values from
    // -16384 to +16383, i.e. -0.5 ... +0.5 pixels/frame.
    // It needs MANY possible values: balls don't hit each other,
    // so two balls with the same vx follow exactly the same path
    // into the same bin. With only 100 values the histogram was
    // jagged, with empty bins; with 32768 it's a smooth bell.
    ball_vx[b] = (rand() % 32768) - 16384 ;

    // Never exactly 0: a ball with vx = 0 lands dead centre on
    // the top peg and bounces straight up and down forever
    if (ball_vx[b] == 0) ball_vx[b] = 1 ;

    // The checkpoint requires zero starting y velocity
    ball_vy[b] = 0 ;

    last_peg[b] = -1 ;
}


// ==================================================
// === move a ball
// ==================================================
void updateBallPosition(int b)
{
    // Update position using velocity
    ball_x[b] += ball_vx[b] ;
    ball_y[b] += ball_vy[b] ;
    // Gravity
    ball_vy[b] += GRAVITY ;
}


// ==================================================
// === peg collision
// ==================================================
// Only check the pegs this ball could be touching. Rows are 19 px
// apart, so only the one or two rows within COLLISION_DISTANCE of
// the ball can matter. Pegs in a row are 38 px apart (more than
// twice COLLISION_DISTANCE), so in each of those rows only the
// nearest peg can matter: at most 2-3 pegs per ball, not 136.
void checkPegCollision(int b)
{
    int x = fix2int15(ball_x[b]) ;
    int y = fix2int15(ball_y[b]) ;

    // Ball height measured from the top row of pegs
    int rel_y = y - PEG_Y ;

    // Entirely above the board: nothing to hit
    if (rel_y + COLLISION_DISTANCE + 1 < 0) return ;

    // First and last rows to check. The +1 covers the fraction of
    // a pixel lost converting y to an int. (rel_y - COLLISION_DISTANCE
    // can be negative, and C division rounds those toward 0, so
    // clamp it.)
    int first_row = (rel_y - COLLISION_DISTANCE) / PEG_VERTICAL_SEPARATION ;
    int last_row = (rel_y + COLLISION_DISTANCE + 1) / PEG_VERTICAL_SEPARATION ;
    if (rel_y - COLLISION_DISTANCE < 0) first_row = 0 ;
    if (last_row > NUM_ROWS - 1) last_row = NUM_ROWS - 1 ;

    for (int row = first_row; row <= last_row; row++) {
        // x of the leftmost peg in this row
        int row_left = PEG_X - row * (PEG_HORIZONTAL_SEPARATION / 2) ;

        // Nearest peg in this row (rounded, clamped to the row)
        int offset = x - row_left + PEG_HORIZONTAL_SEPARATION / 2 ;
        int col = (offset < 0) ? 0 : offset / PEG_HORIZONTAL_SEPARATION ;
        if (col > row) col = row ;

        // Pegs are stored row by row: row r starts at r(r+1)/2
        int p = row * (row + 1) / 2 + col ;

        // Displacement from peg centre to ball centre
        fix15 dx = ball_x[b] - int2fix15(peg_x[p]) ;
        fix15 dy = ball_y[b] - int2fix15(peg_y[p]) ;

        // Quick bounding-box rejection
        fix15 abs_dx = absfix15(dx) ;
        fix15 abs_dy = absfix15(dy) ;
        if (abs_dx >= int2fix15(COLLISION_DISTANCE) || abs_dy >= int2fix15(COLLISION_DISTANCE)) continue ;

        // Approximate distance (alpha max plus beta min)
        fix15 distance = (abs_dx > abs_dy)
            ? multfix15(ALPHA, abs_dx) + multfix15(BETA, abs_dy)
            : multfix15(ALPHA, abs_dy) + multfix15(BETA, abs_dx) ;
        if (distance >= int2fix15(COLLISION_DISTANCE)) continue ;

        // === we collided with peg p ===

        // Avoid divide by zero
        if (distance == 0) distance = 1 ;

        // Normal vector, pointing from the peg to the ball
        fix15 normal_x = divfix(dx, distance) ;
        fix15 normal_y = divfix(dy, distance) ;

        // Handout pseudocode: intermediate = -2 * (n . v)
        // n . v is negative when the ball moves toward the peg.
        // Adding intermediate * n flips the part of the velocity
        // pointing into the peg (a perfect bounce). Energy is lost
        // below, on a new peg only.
        fix15 normal_dot_v = multfix15(normal_x, ball_vx[b]) + multfix15(normal_y, ball_vy[b]) ;
        fix15 intermediate = -2 * normal_dot_v ;

        // Move the ball outside the peg first, or it can stay
        // overlapping and collide again and again
        ball_x[b] = int2fix15(peg_x[p]) + multfix15(normal_x, int2fix15(COLLISION_DISTANCE + 1)) ;
        ball_y[b] = int2fix15(peg_y[p]) + multfix15(normal_y, int2fix15(COLLISION_DISTANCE + 1)) ;

        // Already moving away from the peg (it can still overlap
        // for a frame): don't bounce it, or we'd send it straight
        // back into the peg
        if (normal_dot_v >= 0) return ;

        // Bounce: flip the part of the velocity into the peg
        ball_vx[b] += multfix15(normal_x, intermediate) ;
        ball_vy[b] += multfix15(normal_y, intermediate) ;

        // NEW peg -> lose energy and thunk. Bounciness shrinks the
        // WHOLE velocity, sideways part included, so balls can't
        // build up speed across the board.
        if (last_peg[b] != p) {
            ball_vx[b] = multfix15(bounciness, ball_vx[b]) ;
            ball_vy[b] = multfix15(bounciness, ball_vy[b]) ;
            playThunk() ;
            last_peg[b] = p ;
        }

        // A ball can only touch one peg at a time, so stop looking
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

    // Left wall
    if (x < LEFT_EDGE + BALL_RADIUS) {
        ball_x[b] = int2fix15(LEFT_EDGE + BALL_RADIUS) ;
        ball_vx[b] = -ball_vx[b] ;
    }
    // Right wall
    if (x > RIGHT_EDGE - BALL_RADIUS) {
        ball_x[b] = int2fix15(RIGHT_EDGE - BALL_RADIUS) ;
        ball_vx[b] = -ball_vx[b] ;
    }
    // Top wall (a ball can bounce up off the top peg)
    if (y < TOP_EDGE + BALL_RADIUS && ball_vy[b] < 0) {
        ball_y[b] = int2fix15(TOP_EDGE + BALL_RADIUS) ;
        ball_vy[b] = -ball_vy[b] ;
    }

    // Count the ball as soon as it has cleared the bottom row of
    // pegs, then drop it again from the top. Waiting until the
    // bottom of the screen would let it drift sideways another
    // ~100 px (blurring the histogram) and spend physics on a ball
    // that can't hit anything.
    if (y > BOTTOM_PEG_Y + COLLISION_DISTANCE) {
        // Which bin? Bin k is centred at x = 16 + 38*k,
        // halfway between two bottom-row pegs
        int bin = (x + 3) / 38 ;
        if (bin < 0) bin = 0 ;
        if (bin > NUM_BINS - 1) bin = NUM_BINS - 1 ;

        // Both cores count balls, maybe into the same bin at the
        // same moment, so lock first or a count could be lost
        spin_lock_unsafe_blocking(hist_lock) ;
        histogram[bin]++ ;
        total_fallen++ ;
        spin_unlock_unsafe(hist_lock) ;

        spawnBall(b) ;
    }
}


// ==================================================
// === draw pegs and histogram
// ==================================================
// (Balls are drawn by both cores in animateBalls, before this runs)
void drawGaltonScene()
{
    // Draw all 136 pegs
    for (int p = 0; p < NUM_PEGS; p++) {
        fillCircle(peg_x[p], peg_y[p], PEG_RADIUS, BLUE) ;
    }

    // Recompute the bar heights every 6 frames (10 fps)
    if (++hist_frame_counter >= HIST_UPDATE_INTERVAL) {
        hist_frame_counter = 0 ;

        // Find the tallest bin
        int tallest = 0 ;
        for (int k = 0; k < NUM_BINS; k++) {
            if (histogram[k] > tallest) tallest = histogram[k] ;
        }

        // Scale so the tallest bin fills MAX_BAR_HEIGHT
        for (int k = 0; k < NUM_BINS; k++) {
            cached_heights[k] = (tallest > 0) ? (int)((int64_t)histogram[k] * MAX_BAR_HEIGHT / tallest) : 0 ;
        }
    }

    // Draw the bars as outlines every frame
    for (int k = 0; k < NUM_BINS; k++) {
        int height = cached_heights[k] ;
        if (height <= 0) continue ;

        int left = -3 + 38 * k ;
        int width = 36 ;

        // Keep the first bar on screen
        if (left < 0) {
            width += left ;
            left = 0 ;
        }

        int top = HIST_BOTTOM - height ;
        drawHLine(left, top, width, GREEN) ;                // top
        drawVLine(left, top, height, GREEN) ;               // left side
        drawVLine(left + width - 1, top, height, GREEN) ;   // right side
    }

    // Draw the count under each bin
    for (int k = 0; k < NUM_BINS; k++) {
        char count_text[12] ;
        int len = sprintf(count_text, "%d", histogram[k]) ;

        // Centre the text under the bin
        int x = 16 + 38 * k - 3 * len ;
        if (x < 0) x = 0 ;

        drawTextAscii(x, HIST_BOTTOM + 3, count_text, WHITE, BLACK) ;
    }
}


// ==================================================
// === user interface: modes, button, encoder clicks
// ==================================================
// What the encoder adjusts. Each button press moves to the next
// mode, wrapping back to the first.
enum { MODE_BALLS, MODE_BOUNCINESS, NUM_MODES } ;

int mode = MODE_BALLS ;

// How many balls the encoder asks for
int num_balls = START_BALLS ;

// Clear the histogram and the fallen-through count
void resetCounts()
{
    for (int k = 0; k < NUM_BINS; k++) histogram[k] = 0 ;
    total_fallen = 0 ;
}

// Called once a frame. A press is a HIGH -> LOW change on the
// button pin; presses closer together than 200 ms are switch
// bounce and are ignored.
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

// Apply the clicks the encoder interrupt has counted since last
// frame to the selected parameter. If the parameter actually
// changed, reset the histogram and the fallen count.
void applyEncoderClicks()
{
    // Read and clear together, so a click that arrives in between
    // isn't lost
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
// Each ball is DRAWN as a 2x2 dot, though the physics still
// treats it as radius BALL_RADIUS. A small dot is much less
// drawing than a radius-4 circle.
void drawBall(int x, int y)
{
    // (drawPixel skips any pixel that is off the screen)
    drawPixel(x,     y,     WHITE) ;
    drawPixel(x + 1, y,     WHITE) ;
    drawPixel(x,     y + 1, WHITE) ;
    drawPixel(x + 1, y + 1, WHITE) ;
}


// ==================================================
// === both cores: each animates half of the balls
// ==================================================
// Every frame:
//   core 0: clear screen, read knob, release balls
//   core 0: tell core 1 "go" (through the FIFO)
//   core 0: balls [0, split)      core 1: balls [split, n)
//   core 0: wait for core 1's "done" (it sends its time,
//           like core 1 in Hunter's Mandelbrot demo)
//   core 0: draw pegs, bars, text
//
// Shared by both cores: the histogram (hist_lock), the thunk DMA
// (thunk_lock), and the frame buffer. Two balls from different
// cores touching the same byte can, rarely, lose one ball pixel
// for one frame; that can't be seen.

// The range core 1 animates this frame (set by core 0)
volatile int core1_first = 0 ;
volatile int core1_last  = 0 ;

// Move, collide, count and draw balls first .. last-1
void animateBalls(int first, int last)
{
    for (int b = first; b < last; b++) {
        updateBallPosition(b) ;
        checkPegCollision(b) ;
        checkScreenEdges(b) ;
        // (A ball that just cleared the bottom row was respawned,
        // so this draws it at the top)
        drawBall(fix2int15(ball_x[b]), fix2int15(ball_y[b])) ;
    }
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
    while (1) {
        // Wait for core 0's "go"
        multicore_fifo_pop_blocking() ;

        uint32_t begin_time = time_us_32() ;
        animateBalls(core1_first, core1_last) ;

        // "done", and how long it took (us)
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

    // Missed 60 fps deadlines since boot, and how many more
    // frames the LED stays on after the last one
    static int late_frames = 0 ;
    static int led_frames_left = 0 ;

    // How long the last frame's physics + drawing took (us)
    static uint32_t frame_time_us = 0 ;

    // How long each core spent on its balls (us)
    static uint32_t core0_us = 0 ;
    static uint32_t core1_us = 0 ;

    while (1) {
        // Wait for the signal that the buffer's changed
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;

        // Start timing this frame's work
        uint32_t frame_start = time_us_32() ;

        // Clear the buffer
        clearLowFrame(0, BLACK) ;

        // Button + encoder
        checkButton() ;
        applyEncoderClicks() ;

        // Match the ball count to the encoder.
        // Fewer balls wanted: the highest-numbered balls just disappear.
        if (balls_released > num_balls) balls_released = num_balls ;

        // More balls wanted: release BALLS_PER_FRAME new balls this
        // frame, until there are enough
        for (int i = 0; i < BALLS_PER_FRAME && balls_released < num_balls; i++) {
            spawnBall(balls_released) ;
            balls_released++ ;
        }

        // Physics, collisions, walls, respawn and draw balls,
        // split between the two cores
        int split = balls_released / 2 ;
        core1_first = split ;
        core1_last  = balls_released ;

        // "go" to core 1
        multicore_fifo_push_blocking(1) ;

        uint32_t core0_start = time_us_32() ;
        animateBalls(0, split) ;
        core0_us = time_us_32() - core0_start ;

        // Wait for core 1's "done" (it sends its time)
        core1_us = multicore_fifo_pop_blocking() ;

        // Draw the pegs and the histogram
        drawGaltonScene() ;

        // Tunable parameters. The one the knob is adjusting right
        // now has a ">" in front. (It's also drawn in ORANGE, but in
        // 2-colour mode every colour except BLACK looks the same.)
        sprintf(encoder_text, "%c Balls: %d (animated %d)",
                (mode == MODE_BALLS) ? '>' : ' ', num_balls, balls_released) ;
        drawTextAscii(20, 20, encoder_text, (mode == MODE_BALLS) ? ORANGE : WHITE, BLACK) ;

        sprintf(bounce_text, "%c Bounciness: %d.%02d",
                (mode == MODE_BOUNCINESS) ? '>' : ' ', bounce_percent / 100, bounce_percent % 100) ;
        drawTextAscii(20, 35, bounce_text, (mode == MODE_BOUNCINESS) ? ORANGE : WHITE, BLACK) ;

        drawTextAscii(20, 50, "Knob: adjust  Button: next", CYAN, BLACK) ;

        // Time since boot, in whole seconds
        sprintf(time_text, "Time since boot: %d s", (int)(time_us_64() / 1000000)) ;
        drawTextAscii(20, 70, time_text, MAGENTA, BLACK) ;

        sprintf(hist_text, "Fallen since reset: %d", total_fallen) ;
        drawTextAscii(20, 85, hist_text, GREEN, BLACK) ;

        // Frame compute time vs. the 60 fps budget (16667 us).
        // If this gets close to the budget, the animation starts
        // to slow down. (Shows the previous frame's time.)
        sprintf(frame_text, "Frame: %lu us  Late: %d", (unsigned long)frame_time_us, late_frames) ;
        drawTextAscii(20, 100, frame_text, YELLOW, BLACK) ;

        // Each core's ball time: should be about equal
        sprintf(core_text, "Core0: %lu us  Core1: %lu us", (unsigned long)core0_us, (unsigned long)core1_us) ;
        drawTextAscii(20, 115, core_text, YELLOW, BLACK) ;

        // Time spent computing and drawing this frame
        frame_time_us = time_us_32() - frame_start ;

        // Missed the 60 fps deadline? Keep the LED on for
        // LED_HOLD_FRAMES frames so it can be seen
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
// 300 MHz is exactly 2x the rated 150 MHz. It has to be a
// multiple of 150 MHz because the VGA PIO programs count cycles:
// their clock dividers are multiplied by VGA_CLK_MULT (see
// VGA/*.pio), so the monitor still gets a 25 MHz pixel clock.
//
// Two other things have to change first:
//  1. Core voltage 1.10 V -> 1.30 V, so the CPU is stable at
//     2x speed.
//  2. The flash chip (where the program lives) is rated for
//     133 MHz. Its clock is sys_clk / CLKDIV; at the default
//     divider it would be pushed to 150 MHz, so set CLKDIV = 4
//     (75 MHz) before speeding up.
// Same settings as our ballmax test (ninaa26/ballmax), which ran
// at 300 MHz on a Pico 2.
#define SYS_CLOCK_KHZ 300000

// Runs from RAM: the chip can't read its own program from flash
// while the flash timing is being changed
static void __no_inline_not_in_flash_func(slowFlashClock)(void)
{
    // Wait until the flash is deselected (chip select high)
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

    // One read from flash makes the new timing take effect
    (void)*(volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE ;
}

void overclock()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30) ;
    busy_wait_us(10000) ;        // let the voltage settle
    slowFlashClock() ;
    set_sys_clock_khz(SYS_CLOCK_KHZ, true) ;
}


// ========================================
// === main
// ========================================
int main()
{
    // Overclock to 300 MHz
    overclock() ;

    // initialize stdio
    stdio_init_all() ;

    // Encoder A and B: inputs with pull-ups
    gpio_init(ENCODER_A) ;
    gpio_set_dir(ENCODER_A, GPIO_IN) ;
    gpio_pull_up(ENCODER_A) ;

    gpio_init(ENCODER_B) ;
    gpio_set_dir(ENCODER_B, GPIO_IN) ;
    gpio_pull_up(ENCODER_B) ;

    // Interrupt when A falls
    gpio_set_irq_enabled_with_callback(ENCODER_A, GPIO_IRQ_EDGE_FALL, true, &encoder_callback) ;

    // Encoder push button (pressed = LOW)
    gpio_init(BUTTON_PIN) ;
    gpio_set_dir(BUTTON_PIN, GPIO_IN) ;
    gpio_pull_up(BUTTON_PIN) ;

    // On-board LED
    gpio_init(LED_PIN) ;
    gpio_set_dir(LED_PIN, GPIO_OUT) ;

    // Blink 10 times during startup (10 seconds)
    for (int i = 0; i < 10; i++) {
        gpio_put(LED_PIN, 1) ;
        sleep_ms(500) ;
        gpio_put(LED_PIN, 0) ;
        sleep_ms(500) ;
    }

    // initialize VGA
    initVGA() ;

    // DMA audio
    initAudio() ;

    // Claim and initialize a spinlock for the histogram
    hist_lock = spin_lock_init(spin_lock_claim_unused(true)) ;

    // Peg positions
    buildPegs() ;

    // start core 1 (animates half the balls each frame)
    multicore_reset_core1() ;
    multicore_launch_core1(&core1_main) ;

    // add threads
    pt_add_thread(protothread_anim) ;

    // start scheduler
    pt_schedule_start ;
}
