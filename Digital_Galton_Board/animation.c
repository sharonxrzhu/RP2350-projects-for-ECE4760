/**
 * WEEK 2 CHECKPOINT
 *
 * Digital Galton Board
 *
 * Features:
 *  - 16-row board of pegs (136 pegs)
 *  - 1 to 150 balls dropped from top; the rotary
 *    encoder sets how many (starts at 10)
 *  - Initial vy = 0, small randomized vx
 *  - Gravity
 *  - Fixed-point (fix15) physics; each ball is only
 *    tested against the pegs in the rows it's near
 *  - Bounce physics, Fig. 2 parameters except
 *    bounciness (0.35 instead of 0.5)
 *  - DMA-generated sound when a ball hits a new peg
 *  - Balls automatically respawn after leaving bottom
 *  - Histogram of where balls land, normalized to the
 *    space under the board
 *  - Display: balls animated, balls fallen since reset,
 *    time since boot, frame compute time
 *
 *
 * ROTARY ENCODER:
 * A   ---> GP2
 * B   ---> GP3
 * COM ---> GND
 *
 * DAC:
 * CS   ---> GP5
 * SCK  ---> GP6
 * MOSI ---> GP7
 *
 * VGA:
 * GP16 ---> Hsync
 * GP17 ---> Vsync
 * GP18 ---> Green low
 * GP19 ---> Green high
 * GP20 ---> Blue
 * GP21 ---> Red
 */


// ============================================================
// Includes
// ============================================================

#include "VGA/vga16_graphics_v3.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#include "pico/rand.h"

#include "pt_cornell_rp2040_v1_4.h"


// ============================================================
// Fixed-point arithmetic
// ============================================================

typedef signed int fix15;

#define multfix15(a,b) \
    ((fix15)((((signed long long)(a)) * \
    ((signed long long)(b))) >> 15))

#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)

#define int2fix15(a) ((fix15)((a) << 15))
#define fix2int15(a) ((int)((a) >> 15))

#define divfix(a,b) \
    ((fix15)((((signed long long)(a)) << 15) / (b)))

#define absfix15(a) abs(a)


// ============================================================
// Rotary encoder
// ============================================================

#define ENCODER_A 2
#define ENCODER_B 3

/*
 * The encoder sets how many balls are falling.
 *
 * Each click adds or removes one ball, between
 * MIN_BALLS and MAX_BALLS.
 */

#define MIN_BALLS    1
#define MAX_BALLS    150
#define START_BALLS  10

volatile int encoder_count = START_BALLS;
volatile uint32_t last_encoder_time = 0;


void encoder_callback(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();


    // Debounce
    if ((current_time - last_encoder_time) < 3000)
    {
        return;
    }

    last_encoder_time = current_time;


    // At falling edge of A:
    //
    // B HIGH -> clockwise
    // B LOW  -> counterclockwise

    if (gpio_get(ENCODER_B))
    {
        encoder_count++;
    }
    else
    {
        encoder_count--;
    }


    // Keep the ball count in range

    if (encoder_count > MAX_BALLS)
    {
        encoder_count = MAX_BALLS;
    }

    if (encoder_count < MIN_BALLS)
    {
        encoder_count = MIN_BALLS;
    }
}


// ============================================================
// SPI DAC configuration
// ============================================================
//
// Taken from Hunter Adams DMA audio demo
// ============================================================

#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0


// MCP4822:
// A channel
// 1x gain
// active

#define DAC_CONFIG_CHAN_A 0b0011000000000000


// ============================================================
// Sound configuration
// ============================================================

// The original example used 256 samples for one sine period.
//
// For a thunk, we build a somewhat longer waveform with
// decreasing amplitude.

#define SOUND_SAMPLES 2048


// Values DMA will send directly to DAC
unsigned short sound_buffer[SOUND_SAMPLES];


// DMA channel used for sound
int audio_dma_chan;


// ============================================================
// Build thunk waveform
// ============================================================

void buildSoundBuffer()
{
    for (int i = 0; i < SOUND_SAMPLES; i++)
    {
        /*
         * Envelope goes:
         *
         * 1 ----------------\
         *                    \
         *                     \
         *                      0
         *
         * This makes the sound decay rather than produce
         * a constant beep.
         */

        float envelope =
            1.0f -
            ((float)i / (float)SOUND_SAMPLES);


        /*
         * Frequency is controlled by divisor below.
         *
         * Larger frequency value -> more cycles
         * in the same buffer.
         */

        float wave =
            sinf(
                2.0f *
                3.14159265f *
                18.0f *
                ((float)i / (float)SOUND_SAMPLES)
            );


        /*
         * DAC midpoint = 2048
         *
         * Oscillate around midpoint.
         */

        int sample =
            2048 +
            (int)(
                1800.0f *
                envelope *
                wave
            );


        // Safety clamp for 12-bit DAC

        if (sample < 0)
            sample = 0;

        if (sample > 4095)
            sample = 4095;


        // Add MCP4822 command bits

        sound_buffer[i] =
            DAC_CONFIG_CHAN_A |
            (sample & 0x0fff);
    }
}


// ============================================================
// Initialize audio DMA
// ============================================================

void initAudio()
{
    // --------------------------------------------------------
    // SPI
    // --------------------------------------------------------

    spi_init(SPI_PORT, 20000000);


    // 16 bits / transfer
    // CPOL = 0
    // CPHA = 0
    // MSB first

    spi_set_format(
        SPI_PORT,
        16,
        0,
        0,
        SPI_MSB_FIRST
    );


    // Connect SPI hardware to GPIO

    gpio_set_function(
        PIN_CS,
        GPIO_FUNC_SPI
    );

    gpio_set_function(
        PIN_SCK,
        GPIO_FUNC_SPI
    );

    gpio_set_function(
        PIN_MOSI,
        GPIO_FUNC_SPI
    );


    // --------------------------------------------------------
    // Generate our thunk
    // --------------------------------------------------------

    buildSoundBuffer();


    // --------------------------------------------------------
    // Claim DMA channel
    // --------------------------------------------------------

    audio_dma_chan =
        dma_claim_unused_channel(true);


    // --------------------------------------------------------
    // DMA configuration
    // --------------------------------------------------------

    dma_channel_config config =
        dma_channel_get_default_config(
            audio_dma_chan
        );


    // DAC expects 16-bit packets

    channel_config_set_transfer_data_size(
        &config,
        DMA_SIZE_16
    );


    // Walk through sound_buffer

    channel_config_set_read_increment(
        &config,
        true
    );


    // Always write to same SPI hardware register

    channel_config_set_write_increment(
        &config,
        false
    );


    /*
     * Same timing concept as Hunter Adams'
     * audio DMA example.
     *
     * DMA timer 0 controls sample rate.
     */

    dma_timer_set_fraction(
        0,
        0x0017,
        0xffff
    );


    // DMA timer 0 = DREQ 0x3b

    channel_config_set_dreq(
        &config,
        0x3b
    );


    /*
     * Configure ONCE.
     *
     * We don't start yet.
     *
     * Unlike the original sine-wave example, we're
     * deliberately NOT chaining back to another
     * channel forever.
     */

    dma_channel_configure(
        audio_dma_chan,

        &config,

        // Destination:
        // SPI data register

        &spi_get_hw(SPI_PORT)->dr,

        // Source:
        sound_buffer,

        SOUND_SAMPLES,

        false
    );
}


// ============================================================
// Trigger sound
// ============================================================

void playThunk()
{
    /*
     * Many balls hit pegs, often less than one thunk
     * apart. Stop any thunk still playing and start
     * again from the beginning, so every new hit
     * is heard.
     */

    dma_channel_abort(audio_dma_chan);


    /*
     * Reset DMA source address to start of sound.
     */

    dma_channel_set_read_addr(
        audio_dma_chan,
        sound_buffer,
        false
    );


    /*
     * Reload number of samples and START.
     */

    dma_channel_set_trans_count(
        audio_dma_chan,
        SOUND_SAMPLES,
        true
    );
}


// ============================================================
// Galton board parameters (handout Fig. 2)
// ============================================================

#define BALL_RADIUS 4
#define PEG_RADIUS  6


/*
 * Board shape:
 *
 * row 0  has 1 peg
 * row 1  has 2 pegs
 * ...
 * row 15 has 16 pegs
 *
 * Total = 1 + 2 + ... + 16 = 136 pegs
 */

#define NUM_ROWS 16
#define NUM_PEGS (NUM_ROWS * (NUM_ROWS + 1) / 2)


// Top peg
#define PEG_X 320
#define PEG_Y 80


// Peg grid spacing
#define PEG_VERTICAL_SEPARATION    19
#define PEG_HORIZONTAL_SEPARATION  38


/*
 * When balls are added, release one every
 * RELEASE_GAP_FRAMES frames so they don't all
 * sit on top of each other.
 */

#define RELEASE_GAP_FRAMES 20


// Screen boundaries
#define LEFT_EDGE      8
#define RIGHT_EDGE     632
#define TOP_EDGE       20
#define BOTTOM_EDGE    475


/*
 * Gravity:
 *
 * A small amount is added to vertical velocity
 * each VGA frame.
 */

#define GRAVITY float2fix15(0.37f)


// A ball touches a peg when their centres are closer than this
#define COLLISION_DISTANCE (BALL_RADIUS + PEG_RADIUS)


/*
 * Alpha max plus beta min: a quick approximation of
 * sqrt(dx^2 + dy^2) with no square root.
 *
 *   distance ~= ALPHA * max(|dx|,|dy|) + BETA * min(|dx|,|dy|)
 *
 * These constants keep the error under about 4%.
 */

#define ALPHA float2fix15(0.960433870103f)
#define BETA  float2fix15(0.397824734759f)


/*
 * Bounciness: fraction of the ball's speed kept
 * each time it hits a NEW peg (handout pseudocode).
 *
 * 1.0 = no energy lost
 * 0.0 = ball stops dead
 *
 * The handout default (Fig. 2) is 0.5, but at 0.5 balls
 * bounce over rows and the histogram comes out much
 * wider than the ideal binomial. 0.35 gives a more
 * bell-shaped histogram.
 */

#define BOUNCINESS float2fix15(0.35f)


// ============================================================
// Peg positions
// ============================================================

short peg_x[NUM_PEGS];
short peg_y[NUM_PEGS];


void buildPegs()
{
    int i = 0;

    for (int row = 0; row < NUM_ROWS; row++)
    {
        for (int col = 0; col <= row; col++)
        {
            /*
             * Each row starts half a spacing further
             * left than the row above, so every peg
             * sits under the gap between two pegs
             * of the row above.
             */

            peg_x[i] =
                PEG_X
                - row * (PEG_HORIZONTAL_SEPARATION / 2)
                + col * PEG_HORIZONTAL_SEPARATION;

            peg_y[i] =
                PEG_Y
                + row * PEG_VERTICAL_SEPARATION;

            i++;
        }
    }
}


// ============================================================
// Ball state
// ============================================================

fix15 ball_x[MAX_BALLS];
fix15 ball_y[MAX_BALLS];

fix15 ball_vx[MAX_BALLS];
fix15 ball_vy[MAX_BALLS];


// Index of the last peg each ball hit (-1 = none yet).
//
// A thunk plays only when a ball hits a NEW peg, so one
// peg doesn't make several sounds over consecutive frames.

int last_peg[MAX_BALLS];


// Has this ball been counted in the histogram yet?
// Set once it passes the bottom row, cleared on respawn.
bool counted[MAX_BALLS];


// How many balls are on screen right now.
// This follows encoder_count, one ball at a time.
int balls_released = 0;
//histogram how many bins
#define NUM_BINS (NUM_ROWS + 1)     // 16 rows -> 17 cups
int histogram[NUM_BINS];//global array start at 0
int total_fallen = 0;// balls counted since reset


/*
 * Histogram bars use the space between the bottom row
 * of pegs and the bottom of the screen.
 *
 * Bottom row of pegs: y = PEG_Y + 15 * 19 = 365,
 * plus the peg radius and a small gap -> bars can
 * reach up to y = 375, i.e. 100 px tall.
 */

#define BOTTOM_PEG_Y \
    (PEG_Y + (NUM_ROWS - 1) * PEG_VERTICAL_SEPARATION)

#define MAX_BAR_HEIGHT \
    (BOTTOM_EDGE - (BOTTOM_PEG_Y + PEG_RADIUS + 4))

// ============================================================
// Spawn / respawn ball
// ============================================================

void spawnBall(int b)
{
    // Start near center-top

    ball_x[b] = int2fix15(320);
    ball_y[b] = int2fix15(30);


    /*
     * Random x velocity, in fix15.
     *
     * (rand() % 100) * 2 - 99:
     *
     * -99, -97, ... +97, +99   (always odd)
     *
     * times 164 (= 0.005 in fix15):
     *
     * about -0.495 ... +0.495 pixels/frame
     *
     * Never exactly 0: a ball with vx = 0 lands
     * dead centre on the top peg and bounces
     * straight up and down on it forever.
     */

    ball_vx[b] =
        ((rand() % 100) * 2 - 99) * 164;


    /*
     * Checkpoint specifically requires
     * zero starting y velocity.
     */

    ball_vy[b] = 0;


    last_peg[b] = -1;

    counted[b] = false;
}


// ============================================================
// Update basic ball motion
// ============================================================

void updateBallPosition(int b)
{
    /*
     * Position changes according to current velocity.
     */

    ball_x[b] += ball_vx[b];

    ball_y[b] += ball_vy[b];


    /*
     * Gravity changes vertical velocity.
     *
     * Positive Y points downward on VGA.
     */

    ball_vy[b] += GRAVITY;
}


// ============================================================
// Peg collision
// ============================================================

void checkPegCollision(int b)
{
    /*
     * Only check the pegs this ball could be touching.
     *
     * Rows are PEG_VERTICAL_SEPARATION apart, so only the
     * one or two rows within COLLISION_DISTANCE of the
     * ball can matter. Pegs in a row are
     * PEG_HORIZONTAL_SEPARATION apart (much more than
     * twice COLLISION_DISTANCE), so in each of those rows
     * only the nearest peg can matter.
     *
     * That's at most 2-3 pegs per ball instead of all 136.
     */

    int x = fix2int15(ball_x[b]);
    int y = fix2int15(ball_y[b]);


    // Ball height measured from the top row of pegs
    int rel_y = y - PEG_Y;


    // Entirely above the board: nothing to hit
    if (rel_y + COLLISION_DISTANCE + 1 < 0)
    {
        return;
    }


    /*
     * First and last rows to check. The +1 covers the
     * fraction of a pixel lost converting y to an int.
     * (rel_y - COLLISION_DISTANCE can be negative, and
     * C division rounds those toward 0, so clamp it.)
     */

    int first_row =
        (rel_y - COLLISION_DISTANCE) / PEG_VERTICAL_SEPARATION;

    int last_row =
        (rel_y + COLLISION_DISTANCE + 1) / PEG_VERTICAL_SEPARATION;

    if (rel_y - COLLISION_DISTANCE < 0) first_row = 0;
    if (last_row > NUM_ROWS - 1)        last_row  = NUM_ROWS - 1;


    for (int row = first_row; row <= last_row; row++)
    {
        // x of the leftmost peg in this row
        int row_left =
            PEG_X - row * (PEG_HORIZONTAL_SEPARATION / 2);


        // Nearest peg in this row (rounded, clamped to the row)
        int offset =
            x - row_left + PEG_HORIZONTAL_SEPARATION / 2;

        int col =
            (offset < 0) ? 0 : offset / PEG_HORIZONTAL_SEPARATION;

        if (col > row) col = row;


        // Pegs are stored row by row: row r starts at r(r+1)/2
        int p = row * (row + 1) / 2 + col;


        /*
         * Find displacement from peg center
         * to ball center.
         */

        fix15 dx =
            ball_x[b] - int2fix15(peg_x[p]);

        fix15 dy =
            ball_y[b] - int2fix15(peg_y[p]);


        /*
         * Quick bounding-box rejection.
         */

        fix15 abs_dx = absfix15(dx);
        fix15 abs_dy = absfix15(dy);

        if (
            abs_dx >= int2fix15(COLLISION_DISTANCE) ||
            abs_dy >= int2fix15(COLLISION_DISTANCE)
        )
        {
            continue;
        }


        // Approximate distance (alpha max plus beta min)

        fix15 distance =
            (abs_dx > abs_dy)
            ? multfix15(ALPHA, abs_dx) + multfix15(BETA, abs_dy)
            : multfix15(ALPHA, abs_dy) + multfix15(BETA, abs_dx);


        if (distance >= int2fix15(COLLISION_DISTANCE))
        {
            continue;
        }


        // -----------------------------------------------
        // We collided with peg p
        // -----------------------------------------------


        // Avoid divide by zero
        if (distance == 0)
        {
            distance = 1;
        }


        /*
         * Normal vector pointing:
         *
         * PEG -----> BALL
         */

        fix15 normal_x =
            divfix(dx, distance);

        fix15 normal_y =
            divfix(dy, distance);


        /*
         * Handout pseudocode:
         *
         * intermediate = -2 * (n . v)
         *
         * Adding intermediate * n flips the part of the
         * velocity pointing into the peg (a perfect bounce).
         * Energy is lost below, on a new peg only.
         */

        // n . v : negative when the ball moves toward the peg
        fix15 normal_dot_v =
            multfix15(normal_x, ball_vx[b]) +
            multfix15(normal_y, ball_vy[b]);

        fix15 intermediate =
            -2 * normal_dot_v;


        /*
         * Move ball outside peg first.
         *
         * Otherwise it can remain physically
         * overlapping and collide repeatedly.
         */

        ball_x[b] =
            int2fix15(peg_x[p]) +
            multfix15(
                normal_x,
                int2fix15(COLLISION_DISTANCE + 1)
            );

        ball_y[b] =
            int2fix15(peg_y[p]) +
            multfix15(
                normal_y,
                int2fix15(COLLISION_DISTANCE + 1)
            );


        /*
         * Already moving away from the peg (it can still
         * overlap for a frame): don't bounce it, or we'd
         * send it straight back into the peg.
         */

        if (normal_dot_v >= 0)
        {
            return;
        }


        // Bounce: flip the part of the velocity into the peg

        ball_vx[b] +=
            multfix15(normal_x, intermediate);

        ball_vy[b] +=
            multfix15(normal_y, intermediate);


        // -----------------------------------------------
        // NEW peg -> lose energy and make thunk
        //
        // BOUNCINESS shrinks the WHOLE velocity, sideways
        // part included, so balls can't build up speed
        // across the board.
        // -----------------------------------------------

        if (last_peg[b] != p)
        {
            ball_vx[b] =
                multfix15(BOUNCINESS, ball_vx[b]);

            ball_vy[b] =
                multfix15(BOUNCINESS, ball_vy[b]);

            playThunk();

            last_peg[b] = p;
        }


        /*
         * A ball can only touch one peg at a time,
         * so stop looking.
         */

        return;
    }
}


// ============================================================
// Screen boundaries + respawning
// ============================================================

void checkScreenEdges(int b)
{
    int x =
        fix2int15(ball_x[b]);

    int y =
        fix2int15(ball_y[b]);


    // --------------------------------------------------------
    // Left wall
    // --------------------------------------------------------

    if (x < LEFT_EDGE + BALL_RADIUS)
    {
        ball_x[b] =
            int2fix15(
                LEFT_EDGE +
                BALL_RADIUS
            );

        ball_vx[b] =
            -ball_vx[b];
    }


    // --------------------------------------------------------
    // Right wall
    // --------------------------------------------------------

    if (x > RIGHT_EDGE - BALL_RADIUS)
    {
        ball_x[b] =
            int2fix15(
                RIGHT_EDGE -
                BALL_RADIUS
            );

        ball_vx[b] =
            -ball_vx[b];
    }


    // --------------------------------------------------------
    // Top wall (a ball can bounce up off the top peg)
    // --------------------------------------------------------

    if (y < TOP_EDGE + BALL_RADIUS && ball_vy[b] < 0)
    {
        ball_y[b] =
            int2fix15(
                TOP_EDGE +
                BALL_RADIUS
            );

        ball_vy[b] =
            -ball_vy[b];
    }


    // --------------------------------------------------------
    // Count the ball in the histogram as soon as it has
    // cleared the bottom row of pegs.
    //
    // Waiting until the bottom of the screen would let the
    // ball drift sideways for another ~100 px and blur the
    // histogram.
    // --------------------------------------------------------

    if (!counted[b] && y > BOTTOM_PEG_Y + COLLISION_DISTANCE)
    {
        // Which cup? Cup k is centred at x = 16 + 38*k,
        // halfway between two bottom-row pegs
        int bin = (x + 3) / 38;

        if (bin < 0)             bin = 0;
        if (bin > NUM_BINS - 1)  bin = NUM_BINS - 1;

        histogram[bin]++;
        total_fallen++;

        counted[b] = true;
    }


    // --------------------------------------------------------
    // Ball exits bottom
    //
    // Checkpoint requirement:
    // automatically drop again from the top.
    // --------------------------------------------------------

    if (y > BOTTOM_EDGE + BALL_RADIUS)
    {
        spawnBall(b);
    }

}


// ============================================================
// Draw pegs and balls
// ============================================================

void drawGaltonScene()
{
    // Pegs

    for (int p = 0; p < NUM_PEGS; p++)
    {
        fillCircle(
            peg_x[p],
            peg_y[p],
            PEG_RADIUS,
            BLUE
        );
    }


    // Histogram bars, scaled so the tallest one fills the space

    int tallest = 0;

    for (int k = 0; k < NUM_BINS; k++)
    {
        if (histogram[k] > tallest)
        {
            tallest = histogram[k];
        }
    }


    if (tallest > 0)
    {
        for (int k = 0; k < NUM_BINS; k++)
        {
            int height = histogram[k] * MAX_BAR_HEIGHT / tallest;

            int left  = -3 + 38 * k;     // left edge of cup k
            int width = 36;              // 38 minus a 2 px gap

            /*
             * Cup 0 starts 3 px off the left of the screen.
             * fillRect doesn't check for x < 0, so trim the
             * bar to start at x = 0.
             */

            if (left < 0)
            {
                width += left;
                left   = 0;
            }

            fillRect(
                left,
                BOTTOM_EDGE - height,
                width,
                height,
                GREEN
            );
        }
    }

    // Balls

    for (int b = 0; b < balls_released; b++)
    {
        fillCircle(
            fix2int15(ball_x[b]),
            fix2int15(ball_y[b]),
            BALL_RADIUS,
            LIGHT_PINK
        );
    }
}



// ============================================================
// Main VGA / physics protothread
// ============================================================

static PT_THREAD(
    protothread_display(
        struct pt *pt
    )
)
{
    PT_BEGIN(pt);


    static char encoder_text[40];
    static char time_text[40];
    static char hist_text[80];
    static char frame_text[40];


    // How long the last frame's physics + drawing took (us)
    static uint32_t frame_time_us = 0;


    // Counts VGA frames, for releasing balls
    static int frame_count = 0;


    while (1)
    {
        // ----------------------------------------------------
        // VGA synchronization
        // ----------------------------------------------------

        PT_YIELD_UNTIL(
            pt,
            draw_start_signal()
        );


        // Start timing this frame's work
        uint32_t frame_start = time_us_32();


        // ----------------------------------------------------
        // Clear previous frame
        // ----------------------------------------------------

        clearLowFrame(
            0,
            BLACK
        );


        // ====================================================
        // MATCH THE BALL COUNT TO THE ENCODER
        // ====================================================

        // Read once: the encoder interrupt can change it
        int target_balls = encoder_count;


        /*
         * Fewer balls wanted: the highest-numbered
         * balls just disappear.
         */

        if (balls_released > target_balls)
        {
            balls_released = target_balls;
        }


        /*
         * More balls wanted: release one new ball every
         * RELEASE_GAP_FRAMES frames until there are enough.
         */

        if (
            balls_released < target_balls &&
            frame_count % RELEASE_GAP_FRAMES == 0
        )
        {
            spawnBall(balls_released);

            balls_released++;
        }

        frame_count++;


        // ====================================================
        // PHYSICS, COLLISIONS, WALLS + RESPAWN
        // for every ball
        // ====================================================

        for (int b = 0; b < balls_released; b++)
        {
            updateBallPosition(b);

            checkPegCollision(b);

            checkScreenEdges(b);
        }


        // ====================================================
        // DRAW
        // ====================================================

        drawGaltonScene();


        // ----------------------------------------------------
        // Encoder display
        //
        // Keeping checkpoint 1 alive because checkpoints
        // are cumulative.
        // ----------------------------------------------------

        sprintf(
            encoder_text,
            "Balls animated: %d (target %d)",
            balls_released,
            encoder_count
        );


        drawTextAscii(
            20,
            20,
            encoder_text,
            CYAN,
            BLACK
        );


        drawTextAscii(
            20,
            40,
            "Turn the knob to change the ball count",
            WHITE,
            BLACK
        );


        // Time since boot, in whole seconds
        sprintf(
            time_text,
            "Time since boot: %d s",
            (int)(time_us_64() / 1000000)
        );


        drawTextAscii(
            20,
            60,
            time_text,
            MAGENTA,
            BLACK
        );
        sprintf(
            hist_text,
            "Fallen since reset: %d",
            total_fallen
        );

        drawTextAscii(20, 80, hist_text, GREEN, BLACK);


        /*
         * Frame compute time vs. the 60 fps budget
         * (16667 us). If this gets close to the budget,
         * the animation will start to slow down.
         */

        sprintf(
            frame_text,
            "Frame: %lu us of 16667",
            (unsigned long)frame_time_us
        );

        drawTextAscii(20, 100, frame_text, YELLOW, BLACK);


        // Shown on the next frame
        frame_time_us = time_us_32() - frame_start;

    }


    PT_END(pt);
}


// ============================================================
// Main
// ============================================================

int main()
{
    // --------------------------------------------------------
    // System clock
    // --------------------------------------------------------

    set_sys_clock_khz(
        150000,
        true
    );


    // --------------------------------------------------------
    // Standard IO
    // --------------------------------------------------------

    stdio_init_all();


    // ========================================================
    // Encoder
    // ========================================================

    gpio_init(ENCODER_A);

    gpio_set_dir(
        ENCODER_A,
        GPIO_IN
    );

    gpio_pull_up(
        ENCODER_A
    );


    gpio_init(ENCODER_B);

    gpio_set_dir(
        ENCODER_B,
        GPIO_IN
    );

    gpio_pull_up(
        ENCODER_B
    );


    // Interrupt when A falls

    gpio_set_irq_enabled_with_callback(
        ENCODER_A,
        GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );


    // ========================================================
    // VGA
    // ========================================================

    initVGA();


    // ========================================================
    // DMA audio
    // ========================================================

    initAudio();


    // ========================================================
    // Random number seed
    // ========================================================
    //
    // time_us_32() is almost the same at every boot, so the
    // "random" drops repeated after each power-up.
    // get_rand_32() mixes in hardware noise, so each boot
    // gives a different sequence.
    // ========================================================

    srand(
        get_rand_32()
    );


    // ========================================================
    // Add VGA / physics thread
    // ========================================================

    // ========================================================
    // Peg positions
    // ========================================================

    buildPegs();


    pt_add_thread(
        protothread_display
    );


    // ========================================================
    // Scheduler
    // ========================================================

    pt_schedule_start;


    return 0;
}
