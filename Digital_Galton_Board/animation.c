/**
 * WEEK 1 CHECKPOINT 3
 *
 * Digital Galton Board
 *
 * Features:
 *  - Rotary encoder from checkpoint 1
 *  - 16-row board of pegs (136 pegs)
 *  - 1 to 100 balls dropped from top; the rotary
 *    encoder sets how many (starts at 10)
 *  - Initial vy = 0
 *  - Small randomized vx
 *  - Gravity
 *  - Collision with every peg
 *  - Bounce physics
 *  - DMA-generated sound when a ball hits a new peg
 *  - Balls automatically respawn after leaving bottom
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


/*
 * Bounciness: fraction of the ball's speed kept
 * each time it hits a NEW peg (handout pseudocode).
 *
 * 1.0 = no energy lost
 * 0.0 = ball stops dead
 */

#define BOUNCINESS float2fix15(0.3f)


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


// How many balls are on screen right now.
// This follows encoder_count, one ball at a time.
int balls_released = 0;
//histogram how many bins
#define NUM_BINS (NUM_ROWS + 1)     // 16 rows -> 17 cups
int histogram[NUM_BINS];//global array start at 0
int total_fallen = 0;// balls counted since eset

// ============================================================
// Spawn / respawn ball
// ============================================================

void spawnBall(int b)
{
    // Start near center-top

    ball_x[b] = int2fix15(320);
    ball_y[b] = int2fix15(30);


    /*
     * Random x velocity.
     *
     * rand() % 100:
     *
     * 0 ... 99
     *
     * subtract 49.5:
     *
     * -49.5 ... +49.5
     *
     * divide by 100:
     *
     * -0.495 ... +0.495
     *
     * Never exactly 0: a ball with vx = 0 lands
     * dead centre on the top peg and bounces
     * straight up and down on it forever.
     */

    float random_vx =
        ((float)(rand() % 100) - 49.5f)
        / 100.0f;


    ball_vx[b] =
        float2fix15(random_vx);


    /*
     * Checkpoint specifically requires
     * zero starting y velocity.
     */

    ball_vy[b] = 0;


    last_peg[b] = -1;
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
    // Ball position, converted once instead of per peg

    float bx =
        fix2float15(ball_x[b]);

    float by =
        fix2float15(ball_y[b]);


    float collision_distance =
        BALL_RADIUS + PEG_RADIUS;


    for (int p = 0; p < NUM_PEGS; p++)
    {
        /*
         * Find displacement from peg center
         * to ball center.
         */

        float dx =
            bx - (float)peg_x[p];

        float dy =
            by - (float)peg_y[p];


        /*
         * Quick bounding-box rejection.
         *
         * Don't bother with sqrt() unless we're
         * actually close to this peg.
         */

        if (
            fabsf(dx) >= collision_distance ||
            fabsf(dy) >= collision_distance
        )
        {
            continue;
        }


        // Actual distance

        float distance =
            sqrtf(
                dx * dx +
                dy * dy
            );


        if (distance >= collision_distance)
        {
            continue;
        }


        // -----------------------------------------------
        // We collided with peg p
        // -----------------------------------------------


        /*
         * Normal vector pointing:
         *
         * PEG -----> BALL
         */

        if (distance < 0.001f)
        {
            // Avoid divide by zero
            distance = 0.001f;
        }


        float normal_x =
            dx / distance;

        float normal_y =
            dy / distance;


        // Current velocity

        float vx =
            fix2float15(ball_vx[b]);

        float vy =
            fix2float15(ball_vy[b]);


        /*
         * Handout pseudocode:
         *
         * intermediate = -2 * (n . v)
         *
         * Adding intermediate * n flips the part of the
         * velocity pointing into the peg (a perfect bounce).
         * Energy is lost below, on a new peg only.
         */

        float intermediate =
            -2.0f *
            (
                normal_x * vx +
                normal_y * vy
            );


        /*
         * Move ball outside peg first.
         *
         * Otherwise it can remain physically
         * overlapping and collide repeatedly.
         */

        float new_x =
            peg_x[p] +
            normal_x *
            (
                collision_distance +
                1.0f
            );

        float new_y =
            peg_y[p] +
            normal_y *
            (
                collision_distance +
                1.0f
            );


        ball_x[b] =
            float2fix15(new_x);

        ball_y[b] =
            float2fix15(new_y);


        // Bounce: flip the part of the velocity into the peg

        vx +=
            normal_x *
            intermediate;

        vy +=
            normal_y *
            intermediate;


        // -----------------------------------------------
        // NEW peg -> lose energy and make thunk
        //
        // BOUNCINESS shrinks the WHOLE velocity, sideways
        // part included, so balls can't build up speed
        // across the board.
        // -----------------------------------------------

        if (last_peg[b] != p)
        {
            vx *=
                fix2float15(BOUNCINESS);

            vy *=
                fix2float15(BOUNCINESS);

            playThunk();

            last_peg[b] = p;
        }


        ball_vx[b] =
            float2fix15(vx);

        ball_vy[b] =
            float2fix15(vy);


        /*
         * Pegs are far enough apart that a ball can
         * only touch one at a time, so stop looking.
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
    // Ball exits bottom
    //
    // Checkpoint requirement:
    // automatically drop again from the top.
    // --------------------------------------------------------
    if (y > BOTTOM_EDGE + BALL_RADIUS)
    {
        // Which cup? Cup k is centred at x = 16 + 38*k
        int bin = (x + 3) / 38;

        if (bin < 0)             bin = 0;
        if (bin > NUM_BINS - 1)  bin = NUM_BINS - 1;

        histogram[bin]++;
        total_fallen++;

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

    #define MAX_BAR_HEIGHT 95

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
            "Balls: %d",
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
            "Time: %d s",
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
            "Fallen: %d  middle cups: %d %d %d",
            total_fallen,
            histogram[7],
            histogram[8],
            histogram[9]
        );

        drawTextAscii(20, 80, hist_text, GREEN, BLACK);

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
