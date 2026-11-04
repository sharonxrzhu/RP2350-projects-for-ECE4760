/**
 * 1D Helicopter (PID control) - ECE 4760 Lab 3, week 3
 * Raspberry Pi Pico 2 (RP2350)
 *
 * Built on our week 2 code (week2_IMU_Demo/imu_demo.c), which started from
 * Hunter Adams' MPU6050_IMU_Demo (vha3@cornell.edu).
 * Lab handout: https://vanhunteradams.com/Pico/Helicopter/Helicopter.html
 *
 * WEEK 3 (from the handout; weeks 1 and 2 must still work)
 *  [x] Angle of the arm from a complementary filter of accel + gyro,
 *      shown on the VGA (-90 deg = hanging straight down)
 *  [x] Low-passed motor command shown on the VGA (not the raw PWM)
 *  [x] Set angle and PID gains shown on the VGA (text, top left)
 *  [x] Serial commands, any time, taking effect right away:
 *      target angle, Kp, Ki, Kd (see the serial thread for all of them)
 *  [x] PID sets the PWM duty of the motor; one set of gains for the whole
 *      range (Kp 200, Ki 200, Kd 20; they start set, see below)
 *  [x] Button (not reset) demo sequence with the current gains:
 *        held     -> motor off, arm hanging down
 *        released -> t = 0 s level, 5 s +30 deg, 10 s -30 deg, 15 s level
 *  [x] Never needs a reset: the button restarts the sequence from hanging
 *
 * Not plain PID (say so in the report): a feedforward term adds the power
 * that holding the arm at the aim angle needs (FF_GAIN * cos^1.8), so the I
 * term only fixes what's left. Without it, upward steps overshot 13-21 deg.
 * And after a fresh start the aim slides up at START_RAMP deg/s, so the arm
 * doesn't fly past the first target. Serial 'f 0' turns feedforward off.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 0  ---> UART TX  (to the Debug Probe / USB-serial adapter's RX)
 *  - GPIO 1  ---> UART RX  (to its TX)
 *  - GPIO 4  ---> PWM to the motor board (PWM terminal; MCU GND to Pico GND)
 *  - GPIO 10 ---> MPU6050 3.3V (the code power-cycles the IMU at boot)
 *  - GPIO 11 ---> button ---> GND (physical pins 15 and 18; no resistor,
 *                 the Pico's internal pull-up is used)
 *  - GPIO 12 ---> MPU6050 SDA
 *  - GPIO 13 ---> MPU6050 SCL
 *  - GPIO 14 ---> toggles in the PWM interrupt (scope: should be 500 Hz)
 *  - GPIO 15 ---> toggles once per VGA frame
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2350 GND ---> VGA GND, MPU6050 GND, motor board MCU GND, button
 *  Motor board: IRF520 MOSFET (fitted 23 Oct in place of the IRLB8721).
 */



// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// We will start drawing at column 81
#define LEFT_EDGE 81

// The IMU's power wire is on this pin, so main() can power-cycle the IMU at boot
#define IMU_POWER_PIN 10

// Rescale the measurements for display
#define OLD_RANGE 500.  // (+/- 250)
#define NEW_RANGE 150.  // (looks nice on VGA)
#define OLD_MIN -250. 
#define OLD_MAX 250. 
#define ACCEL_SCALE 120.0

// === arm angle (complementary filter) ===
// Which IMU axis is which on our arm (measured 23 Oct). 0 = x, 1 = y, 2 = z.
#define ARM_AXIS  0      // x: along the arm
#define UP_AXIS   2      // z: up when the arm is level
#define GYRO_AXIS 1      // y: along the hinge
// +1.0f or -1.0f, chosen so that each reading goes UP as you lift the arm
#define ARM_SIGN  -1.0f   // x points toward the hinge (+1 g hanging, -1 g straight up)
#define UP_SIGN    1.0f   // z reads about +1 g when level
#define GYRO_SIGN  1.0f   // checked: y reads positive while the arm rises
// Calibration: our accelerometer reads a little off on x and z, in g.
// Measured with the arm hanging, level and straight up (23 Oct).
#define ACCEL_X_OFFSET  0.07f
#define ACCEL_Z_OFFSET -0.22f

#define DT           0.001f    // seconds between samples (the interrupt runs at 1 kHz)
#define ACCEL_SMOOTH 0.05f     // low-pass on the accelerometer: smaller = smoother, slower
#define ACCEL_WEIGHT 0.01f     // how much to trust the accelerometer each sample
#define RAD_TO_DEG   57.29578f // 180 / pi

// Smoothed accelerometer readings, in g
float arm_g = 0.0f ;
float up_g = 0.0f ;
// Best guess of the arm angle, degrees. 0 = level, -90 = hanging straight down
volatile float angle = 0.0f ;
// How fast the arm is turning, degrees per second (straight from the gyro)
volatile float angle_rate = 0.0f ;
// 0 until the first sample has set the starting angle
int angle_started = 0 ;

// Place the plots
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 530
// Screen row for an angle in degrees: +90 at row 80, 0 at row 155, -90 at row 230
#define ANGLE_Y(a) (TOP_PLOT_BOTTOM - 75 - (int)((a) * 75.0f / 90.0f))
// Screen row for a motor command: 0 at row 430, 6000 at row 280
#define MOTOR_Y(m) (BOTTOM_PLOT_BOTTOM - (int)((m) * 150 / 6000))

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// Arrays for circular display buffer: the last PLOT_WIDTH samples
float angle_hist[PLOT_WIDTH] ;   // arm angle, degrees
int motor_hist[PLOT_WIDTH] ;     // low-passed motor command, 0-6000
float target_hist[PLOT_WIDTH] ;  // target angle, degrees

// Pointer to head of buffer
volatile int head = 0 ;

// character array
char screentext[80];

// draw speed
// one point every 40 ms: 530 points = 21 s, so the whole 20 s sequence fits
volatile int threshold = 40 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 6000
#define CLKDIV  25.0
uint slice_num ;

// === motor ===
// The motor's PWM is on GPIO 4 (physical pin 6): PWM slice 2, channel A
// 0 = off, WRAPVAL (6000) = full power
// Highest command the PID controller may give the motor. Holding +30 takes about
// 5200, and the climb there peaks near 5460 (IRF520, bench test, 4 Nov).
// Change it over serial with x.
#define MOTOR_CAP 5600
volatile int motor_cap = MOTOR_CAP ;
volatile int motor_cmd = 0 ;
// Smoothed copy of motor_cmd, only for the screen
volatile int motor_disp = 0 ;

// === PID controller ===
// 0 = motor set by hand ('m'), 1 = the controller sets it
volatile int pid_on = 0 ;
// Where we want the arm, degrees (0 = level)
volatile float target = 0.0f ;
// The three gains. They start at the values we tuned (23 Oct), so the button
// demo works straight after power-up. Change them over serial with p, i and d
volatile float Kp = 200.0f ;
volatile float Ki = 200.0f ;
volatile float Kd = 20.0f ;
// Feedforward: about the power needed to hold the arm level (4900-5200 with
// the IRF520; hinge friction makes it depend on the direction). The controller
// adds FF_GAIN * cos(aim)^1.8, which matches how the holding power falls off
// above and below level (measured 23 Oct, IRLB8721).
// 0 = off (plain PID). Change it over serial with f
#define FF_GAIN 4700.0f
volatile float ff_gain = FF_GAIN ;
// After a fresh start the aim point slides toward the target at this many
// degrees per second, instead of jumping (otherwise the arm flies up to
// ~50 deg past level). Later target changes jump straight there.
#define START_RAMP 60.0f
// Where the controller is aiming right now, degrees (slides to target)
volatile float setpoint = 0.0f ;
// 1 from a fresh start until the aim point reaches the target
volatile int starting = 0 ;
// Button demo sequence (see protothread_button)
// 1 while the button is held down (after debouncing)
volatile int button_held = 0 ;
// 1 while the sequence is running; seconds since the button was let go
volatile int seq_running = 0 ;
volatile float seq_time = 0.0f ;
// The I term so far, already in motor units (0-6000)
volatile float i_term = 0.0f ;


void drawAxes() {
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "3000") ;
    setCursor(45, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "6000") ;
    setCursor(45, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;
    sprintf(screentext, "motor command (low-passed)") ;
    setCursor(85, 265) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+90") ;
    setCursor(50, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-90") ;
    setCursor(50, 225) ;
    writeString(screentext) ;
    sprintf(screentext, "arm angle (deg)") ;
    setCursor(85, 65) ;
    writeString(screentext) ;
}

volatile int throttle = 10 ;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
    gpio_put(14, !gpio_get(14)) ;

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // --- Estimate the arm angle ---
    // 1. This sample's accelerometer readings along the arm and "up", in g,
    //    with our accelerometer's offsets taken off
    float arm_now = ARM_SIGN * (fix2float15(acceleration[ARM_AXIS]) - ACCEL_X_OFFSET) ;
    float up_now  = UP_SIGN  * (fix2float15(acceleration[UP_AXIS])  - ACCEL_Z_OFFSET) ;
    if (!angle_started) {
        arm_g = arm_now ;
        up_g  = up_now ;
    }
    // 2. Smooth them: the motor shakes the accelerometer
    arm_g += ACCEL_SMOOTH * (arm_now - arm_g) ;
    up_g  += ACCEL_SMOOTH * (up_now  - up_g) ;
    // 3. Angle from gravity alone: noisy, but it never drifts
    float accel_angle = atan2f(arm_g, up_g) * RAD_TO_DEG ;
    // 4. Turn rate from the gyro, degrees per second: smooth, but it drifts
    angle_rate = GYRO_SIGN * fix2float15(gyro[GYRO_AXIS]) ;
    // 5. Blend: last angle + what the gyro says we turned, nudged toward gravity
    if (!angle_started) {
        angle = accel_angle ;
        angle_started = 1 ;
    }
    angle = (1.0f - ACCEL_WEIGHT) * (angle + angle_rate * DT) + ACCEL_WEIGHT * accel_angle ;

    // --- PID: work out the motor command from the angle ---
    if (pid_on) {
        // Aim point: slides up after a fresh start, otherwise = target
        if (starting) {
            float step = START_RAMP * DT ;
            if (setpoint < target - step) setpoint += step ;
            else if (setpoint > target + step) setpoint -= step ;
            else { setpoint = target ; starting = 0 ; }
        }
        else setpoint = target ;
        // How far off are we? Positive = the arm is too low
        float error = setpoint - angle ;
        // I: add up the error over time (stored already multiplied by Ki)
        i_term += Ki * error * DT ;
        if (i_term > WRAPVAL) i_term = WRAPVAL ;   // don't let it grow forever
        // With feedforward, I may go below 0 to trim a guess that's too high
        if (i_term < -ff_gain) i_term = -ff_gain ;
        // P + I + D. For D, the gyro already measures how fast the angle changes
        float out = Kp * error + i_term - Kd * angle_rate ;
        // Feedforward from the aim point (not the measured angle, so it
        // can't feed back on itself). cos is negative past +/-90 deg: skip
        float c = cosf(setpoint / RAD_TO_DEG) ;
        if (ff_gain > 0.0f && c > 0.0f) out += ff_gain * powf(c, 1.8f) ;
        // The motor can only push up, from off to motor_cap
        if (out < 0) out = 0 ;
        if (out > motor_cap) out = motor_cap ;   // see MOTOR_CAP
        motor_cmd = (int)out ;
    }

    // --- Drive the motor ---
    pwm_set_chan_level(slice_num, PWM_CHAN_A, motor_cmd) ;
    // Smooth the command for the screen (the handout's low-pass, about 16 samples)
    motor_disp = motor_disp + ((motor_cmd - motor_disp) >> 4) ;

    // Increment drawspeed controller
    throttle += 1 ;
    // Maintain a circular buffer for display
    if ((throttle >= threshold)) { 
        // Zero drawspeed controller
        throttle = 0 ;

        // Replace the oldest sample with the newest
        angle_hist[head] = angle ;
        motor_hist[head] = motor_disp ;
        target_hist[head] = target ;

        // Move the pointer to the oldest data
        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1) ;

    }
}

static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int start ;
    static int i ;

    // Draw axes
    drawAxes() ;
    // Copy to other buffer
    copy_buffer_to_other() ;

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        gpio_put(15, !gpio_get(15)) ;
        
        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        // Show the angle as a number, top left
        setTextSize(2) ;
        setCursor(10, 10) ;
        sprintf(screentext, "angle %6.1f deg", angle) ;
        writeString(screentext) ;
        // Second line: target, gains and who is driving the motor
        setTextSize(1) ;
        setCursor(10, 34) ;
        sprintf(screentext, "%s  target %.1f  Kp %.2f  Ki %.2f  Kd %.2f  ff %.0f  motor %d",
                pid_on ? "PID" : "manual", target, Kp, Ki, Kd, ff_gain, motor_cmd) ;
        writeString(screentext) ;
        // Third line: the button sequence
        setCursor(10, 46) ;
        if (button_held) sprintf(screentext, "button held: motor off") ;
        else if (seq_running) sprintf(screentext, "sequence %4.1f s", seq_time) ;
        else sprintf(screentext, "                ") ;
        writeString(screentext) ;

        // Plot data oldest-->newest
        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            // Target angle, yellow (drawn first so the white angle trace is on top)
            drawPixel(LEFT_EDGE + i, ANGLE_Y(target_hist[start]), YELLOW) ;
            // Top plot: arm angle, white
            drawPixel(LEFT_EDGE + i, ANGLE_Y(angle_hist[start]), WHITE) ;
            // Bottom plot: low-passed motor command, green
            drawPixel(LEFT_EDGE + i, MOTOR_Y(motor_hist[start]), GREEN) ;
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }
    }

    PT_END(pt) ;
}

// === button demo sequence ===
#define BUTTON_PIN 11      // GPIO 11 (physical pin 15) to GND when pressed

// Turn the controller on at a new target. From off, it's a fresh start:
// I starts at 0 and the aim slides up from where the arm is now
void pid_go(float new_target) {
    target = new_target ;
    if (!pid_on) {
        i_term = 0.0f ;
        setpoint = angle ;
        starting = 1 ;
    }
    pid_on = 1 ;
}

// Controller off, motor off
void pid_stop() {
    pid_on = 0 ;
    motor_cmd = 0 ;
}

// Button thread: reads the button every 10 ms. The reading has to stay the
// same 3 times in a row (30 ms) before we believe it: a button's contacts
// bounce for a few ms when pressed (debouncing). Like a doorbell that only
// counts a ring if you hold it, not a brush past it.
static PT_THREAD (protothread_button(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static int same = 0 ;
    static int last_raw = 0 ;
    static uint32_t seq_start = 0 ;
    while(1) {
        PT_YIELD_usec(10000) ;
        // Pressed connects the pin to GND: reads 0
        int raw = !gpio_get(BUTTON_PIN) ;
        if (raw == last_raw) same++ ;
        else same = 0 ;
        last_raw = raw ;
        if (same == 3) {
            if (raw && !button_held) {
                // Pressed: motor off, arm hangs, sequence cancelled
                button_held = 1 ;
                seq_running = 0 ;
                pid_stop() ;
            }
            else if (!raw && button_held) {
                // Released: start the sequence at level
                button_held = 0 ;
                seq_start = time_us_32() ;
                seq_running = 1 ;
                pid_go(0.0f) ;
            }
        }
        // Held: keep the motor off, whatever else asked for
        if (button_held) pid_stop() ;
        // The sequence: level, +30 at 5 s, -30 at 10 s, level at 15 s
        if (seq_running) {
            seq_time = (time_us_32() - seq_start) / 1000000.0f ;
            if (seq_time >= 15.0f) { target = 0.0f ; seq_running = 0 ; }
            else if (seq_time >= 10.0f) target = -30.0f ;
            else if (seq_time >= 5.0f) target = 30.0f ;
            else target = 0.0f ;
        }
    }
    PT_END(pt) ;
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a command:\r\n");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        // num_independents = test_in ;
        if (classifier=='t') {
            sprintf(pt_serial_out_buffer, "timestep:\r\n");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in > 0) {
                threshold = test_in ;
            }
        }
        // m: set the motor power directly (open loop), 0-6000
        else if (classifier=='m') {
            sprintf(pt_serial_out_buffer, "motor (0-6000):\r\n");
            serial_write ;
            serial_read ;
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in >= 0 && test_in <= WRAPVAL) {
                pid_on = 0 ;
                motor_cmd = test_in ;
            }
        }
        // a: set the target angle (degrees) and switch the controller on
        else if (classifier=='a') {
            sprintf(pt_serial_out_buffer, "target angle (deg):\r\n");
            serial_write ;
            serial_read ;
            if (sscanf(pt_serial_in_buffer,"%f", &float_in) == 1) {
                seq_running = 0 ;           // typing a target ends the sequence
                pid_go(float_in) ;
            }
        }
        // p, i, d: set a gain. It takes effect on the next sample
        else if (classifier=='p' || classifier=='i' || classifier=='d') {
            sprintf(pt_serial_out_buffer, "K%c:\r\n", classifier);
            serial_write ;
            serial_read ;
            if (sscanf(pt_serial_in_buffer,"%f", &float_in) == 1) {
                if (classifier=='p') Kp = float_in ;
                if (classifier=='i') Ki = float_in ;
                if (classifier=='d') Kd = float_in ;
            }
        }
        // x: set the controller's motor cap (0-6000)
        else if (classifier=='x') {
            sprintf(pt_serial_out_buffer, "motor cap (0-6000):\r\n");
            serial_write ;
            serial_read ;
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in >= 0 && test_in <= WRAPVAL) motor_cap = test_in ;
        }
        // f: set the feedforward gain (0 = off: plain PID)
        else if (classifier=='f') {
            sprintf(pt_serial_out_buffer, "feedforward (0 = off):\r\n");
            serial_write ;
            serial_read ;
            if (sscanf(pt_serial_in_buffer,"%f", &float_in) == 1 && float_in >= 0.0f)
                ff_gain = float_in ;
        }
        // s: stop. Controller off, motor off, sequence cancelled
        else if (classifier=='s') {
            seq_running = 0 ;
            pid_stop() ;
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    gpio_init(15) ;
    gpio_set_dir(15, GPIO_OUT) ;
    gpio_put(15, 0) ;
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    // Power-cycle the IMU so it always starts fresh, even if the Pico
    // reset in the middle of a read
    gpio_init(IMU_POWER_PIN) ;
    gpio_set_dir(IMU_POWER_PIN, GPIO_OUT) ;
    gpio_set_drive_strength(IMU_POWER_PIN, GPIO_DRIVE_STRENGTH_12MA) ;
    // Hold SDA and SCL low while it's off, or it can stay half-powered through them
    gpio_init(SDA_PIN) ; gpio_set_dir(SDA_PIN, GPIO_OUT) ; gpio_put(SDA_PIN, 0) ;
    gpio_init(SCL_PIN) ; gpio_set_dir(SCL_PIN, GPIO_OUT) ; gpio_put(SCL_PIN, 0) ;
    gpio_put(IMU_POWER_PIN, 0) ;   // IMU off
    sleep_ms(500) ;
    gpio_put(IMU_POWER_PIN, 1) ;   // IMU on
    sleep_ms(100) ;                // give it time to start up

    // If the Pico restarted in the middle of reading the IMU, the IMU can be
    // left holding SDA low and the first I2C write would wait forever.
    // Clocking SCL 9 times lets it finish and let go of the bus.
    gpio_init(SDA_PIN) ;
    gpio_set_dir(SDA_PIN, GPIO_IN) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_init(SCL_PIN) ;
    gpio_set_dir(SCL_PIN, GPIO_OUT) ;
    for (int i = 0; i < 9; i++) {
        gpio_put(SCL_PIN, 0) ; sleep_us(5) ;
        gpio_put(SCL_PIN, 1) ; sleep_us(5) ;
    }
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

   
    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    gpio_init(14) ;
    gpio_set_dir(14, GPIO_OUT) ;
    gpio_put(14, 0) ;
    // Button: input with the internal pull-up (reads 1 until pressed)
    gpio_init(BUTTON_PIN) ;
    gpio_set_dir(BUTTON_PIN, GPIO_IN) ;
    gpio_pull_up(BUTTON_PIN) ;
    pt_add_thread(protothread_button) ;
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
