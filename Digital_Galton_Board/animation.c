/**
 * Week 1 Digital Galton Board
 *
 * Encoder:
 *   A   -> GP2
 *   B   -> GP3
 *   COM -> GND
 *
 * DAC:
 *   CS   -> GP5
 *   SCK  -> GP6
 *   MOSI -> GP7
 *
 * VGA:
 *   GP16 -> Hsync
 *   GP17 -> Vsync
 *   GP18 -> Green low
 *   GP19 -> Green high
 *   GP20 -> Blue
 *   GP21 -> Red
 */

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

#include "pt_cornell_rp2040_v1_4.h"


// ============================================================
// Fixed point
// ============================================================

typedef signed int fix15;

#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define int2fix15(a)   ((fix15)((a) << 15))
#define fix2int15(a)   ((int)((a) >> 15))


// ============================================================
// Rotary encoder
// ============================================================

#define ENCODER_A 2
#define ENCODER_B 3

volatile int encoder_count = 0;
volatile uint32_t last_encoder_time = 0;

void encoder_callback(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    // 3 ms debounce
    if ((current_time - last_encoder_time) < 3000)
        return;

    last_encoder_time = current_time;

    if (gpio_get(ENCODER_B))
        encoder_count++;
    else
        encoder_count--;
}


// ============================================================
// DAC + DMA audio
// ============================================================

#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7

#define SPI_PORT spi0

#define DAC_CONFIG_CHAN_A 0b0011000000000000
#define SOUND_SAMPLES 2048

unsigned short sound_buffer[SOUND_SAMPLES];
int audio_dma_chan;


void buildSoundBuffer()
{
    for (int i = 0; i < SOUND_SAMPLES; i++)
    {
        float t = (float)i / SOUND_SAMPLES;

        // Decreasing amplitude
        float envelope = 1.0f - t;

        // 18-cycle sine wave
        float wave = sinf(2.0f * 3.14159265f * 18.0f * t);

        int sample =
            2048 +
            (int)(1800.0f * envelope * wave);

        if (sample < 0) sample = 0;
        if (sample > 4095) sample = 4095;

        sound_buffer[i] =
            DAC_CONFIG_CHAN_A |
            (sample & 0x0fff);
    }
}


void initAudio()
{
    spi_init(SPI_PORT, 20000000);

    spi_set_format(
        SPI_PORT,
        16,
        0,
        0,
        SPI_MSB_FIRST
    );

    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    buildSoundBuffer();

    audio_dma_chan =
        dma_claim_unused_channel(true);

    dma_channel_config config =
        dma_channel_get_default_config(audio_dma_chan);

    channel_config_set_transfer_data_size(
        &config,
        DMA_SIZE_16
    );

    channel_config_set_read_increment(
        &config,
        true
    );

    channel_config_set_write_increment(
        &config,
        false
    );

    // DMA timer controls audio sample rate
    dma_timer_set_fraction(
        0,
        0x0017,
        0xffff
    );

    channel_config_set_dreq(
        &config,
        0x3b
    );

    dma_channel_configure(
        audio_dma_chan,
        &config,
        &spi_get_hw(SPI_PORT)->dr,
        sound_buffer,
        SOUND_SAMPLES,
        false
    );
}


void playThunk()
{
    if (dma_channel_is_busy(audio_dma_chan))
        return;

    dma_channel_set_read_addr(
        audio_dma_chan,
        sound_buffer,
        false
    );

    dma_channel_set_trans_count(
        audio_dma_chan,
        SOUND_SAMPLES,
        true
    );
}


// ============================================================
// Galton board
// ============================================================

#define BALL_RADIUS 7
#define PEG_RADIUS  8

#define PEG_X 320
#define PEG_Y 250

#define LEFT_EDGE   8
#define RIGHT_EDGE  632
#define BOTTOM_EDGE 475

#define GRAVITY    float2fix15(0.12f)
#define BOUNCINESS 0.75f

fix15 ball_x;
fix15 ball_y;
fix15 ball_vx;
fix15 ball_vy;

bool touching_peg = false;


// ============================================================
// Ball spawn
// ============================================================

void spawnBall()
{
    ball_x = int2fix15(320);
    ball_y = int2fix15(30);

    // Random vx from -0.50 to +0.50
    float random_vx =
        ((float)(rand() % 101) - 50.0f) / 100.0f;

    ball_vx = float2fix15(random_vx);

    // Starts with no vertical velocity
    ball_vy = 0;

    touching_peg = false;
}


// ============================================================
// Ball motion
// ============================================================

void updateBallPosition()
{
    ball_x += ball_vx;
    ball_y += ball_vy;

    // Positive y points downward
    ball_vy += GRAVITY;
}


// ============================================================
// Peg collision
// ============================================================

void checkPegCollision()
{
    float dx =
        fix2float15(ball_x) - PEG_X;

    float dy =
        fix2float15(ball_y) - PEG_Y;

    float collision_distance =
        BALL_RADIUS + PEG_RADIUS;

    // Cheap check before sqrt()
    if (fabsf(dx) >= collision_distance ||
        fabsf(dy) >= collision_distance)
    {
        touching_peg = false;
        return;
    }

    float distance =
        sqrtf(dx * dx + dy * dy);

    if (distance >= collision_distance)
    {
        touching_peg = false;
        return;
    }

    if (distance < 0.001f)
        distance = 0.001f;

    // Unit normal from peg -> ball
    float nx = dx / distance;
    float ny = dy / distance;

    float vx = fix2float15(ball_vx);
    float vy = fix2float15(ball_vy);

    // Reflection equation
    float intermediate =
        -2.0f * (nx * vx + ny * vy);

    // Move ball outside peg
    ball_x = float2fix15(
        PEG_X + nx * (collision_distance + 1.0f)
    );

    ball_y = float2fix15(
        PEG_Y + ny * (collision_distance + 1.0f)
    );

    // Only reflect if moving into peg
    if (intermediate > 0.0f)
    {
        vx += nx * intermediate;
        vy += ny * intermediate;

        vx *= BOUNCINESS;
        vy *= BOUNCINESS;

        ball_vx = float2fix15(vx);
        ball_vy = float2fix15(vy);

        if (!touching_peg)
            playThunk();
    }

    touching_peg = true;
}


// ============================================================
// Screen edges
// ============================================================

void checkScreenEdges()
{
    int x = fix2int15(ball_x);
    int y = fix2int15(ball_y);

    if (x < LEFT_EDGE + BALL_RADIUS)
    {
        ball_x =
            int2fix15(LEFT_EDGE + BALL_RADIUS);

        ball_vx = -ball_vx;
    }

    if (x > RIGHT_EDGE - BALL_RADIUS)
    {
        ball_x =
            int2fix15(RIGHT_EDGE - BALL_RADIUS);

        ball_vx = -ball_vx;
    }

    // Respawn after leaving bottom
    if (y > BOTTOM_EDGE + BALL_RADIUS)
        spawnBall();
}


// ============================================================
// Drawing
// ============================================================

void drawGaltonScene()
{
    fillCircle(
        PEG_X,
        PEG_Y,
        PEG_RADIUS,
        RED
    );

    fillCircle(
        fix2int15(ball_x),
        fix2int15(ball_y),
        BALL_RADIUS,
        WHITE
    );
}


// ============================================================
// VGA / physics thread
// ============================================================

static PT_THREAD(protothread_display(struct pt *pt))
{
    PT_BEGIN(pt);

    static char encoder_text[40];

    spawnBall();

    while (1)
    {
        PT_YIELD_UNTIL(
            pt,
            draw_start_signal()
        );

        clearLowFrame(0, BLACK);

        updateBallPosition();
        checkPegCollision();
        checkScreenEdges();

        drawGaltonScene();

        sprintf(
            encoder_text,
            "Encoder: %d",
            encoder_count
        );

        drawTextAscii(
            20,
            20,
            encoder_text,
            CYAN,
            BLACK
        );

        // drawTextAscii(
        //     20,
        //     40,
        //     "Week 1 Checkpoint 2",
        //     WHITE,
        //     BLACK
        // );
    }

    PT_END(pt);
}


// ============================================================
// Main
// ============================================================

int main()
{
    set_sys_clock_khz(150000, true);

    stdio_init_all();

    // Encoder
    gpio_init(ENCODER_A);
    gpio_set_dir(ENCODER_A, GPIO_IN);
    gpio_pull_up(ENCODER_A);

    gpio_init(ENCODER_B);
    gpio_set_dir(ENCODER_B, GPIO_IN);
    gpio_pull_up(ENCODER_B);

    gpio_set_irq_enabled_with_callback(
        ENCODER_A,
        GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    // VGA
    initVGA();

    // Audio
    initAudio();

    // Different random vx each boot
    srand(time_us_32());

    pt_add_thread(protothread_display);

    pt_schedule_start;

    return 0;
}