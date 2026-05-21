/*
 * gpio.c
 *
 * Initial version:  EEE 158 AY2025-26 Sem 1 Handlers
 * Modifications by: DE VILLA, Alberto
 * 
 * Additional Modifications by: SIBUG, Tristan Josh E., 2021-60143, WYZ
 *
 * Pushbutton and LED logic
 */

#include "include.h"

#include "../init.h"

#include <xc.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>

// forward declarations for the new functions
static void platform_LED_pwm_set(int brightness);
static void platform_LED_pwm_clear(void);

static volatile uint8_t motor_pwm_counter = 0;
static volatile uint8_t motor_pwma_duty = 0;
static volatile uint8_t motor_pwmb_duty = 0;

// ==========================================================================

// Initialize GPIO here
void platform_gpio_init(void)
{
    // Motor Driver Pins Setup
    // PA04=PWMA, PA05=AIN1, PA12=AIN2 (Left Motor)
    // PA13=PWMB, PA20=BIN1, PA14=BIN2 (Right Motor)
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 4) | (1 << 5) | (1 << 12) | (1 << 13) | (1 << 20) | (1 << 14) | (1 << 16);

    // Initialize all LOW (safe stop at boot)
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 4) | (1 << 5) | (1 << 12) | (1 << 13) | (1 << 20) | (1 << 14) | (1 << 16);
    
	return;
}

// ==========================================================================

void platform_ADC_init(void)
{
    uint8_t pmux_adc;
    
    /*
	 * =============================================================
	 * Configure MCLK to enable the ADC
	 *
	 * Q: Is this necessary?
	 * A: Not necessary; leave it alone. (IC datasheet, § 18.5.2.6)
	 * =============================================================
	 */

    
    
	/*
	 * =============================================================
	 * Configure GCLK
	 *
	 * Unlike EIC where GCLK is only needed for filtering + debouncing
	 * functionality, GCLK is unconditionally needed for ADC as it is of a
	 * successive-approximation type.
	 *
	 * This assignment selects GCLK2 (4 MHz), then enables GCLK for ADC.
	 * =============================================================
	 */
    
	GCLK_REGS->GCLK_PCHCTRL[28] = 0x00000042;
	while ((GCLK_REGS->GCLK_PCHCTRL[28] & (1 << 6)) == 0) {
		// Wait for synchronization
        asm("nop");
	}

	/*
	 * =============================================================
	 * Configure the ADC itself. First task should be a soft-reset via
	 * the SWRST bit.
	 *
	 * ENABLE should be set last.
	 * =============================================================
	 */
    
	ADC_REGS->ADC_CTRLA = 0x01;
	while ((ADC_REGS->ADC_SYNCBUSY & (1 << 0)) != 0) {
		// Wait for synchronization
		asm("nop");
	}
    
    /*
	 * The problem specifications indicate that the positive reference be
	 * AVDD. On this platform, the negative reference is internally tied to
	 * AVSS.
	 */
	
    ADC_REGS->ADC_REFCTRL = 0x05;
	
    /* 
	 * Let's say the maximum signal frequency is 100 Hz, and we want at
	 * least 32 10-bit samples per cycle. The theoretical minimum
	 * ADC clock frequency is
	 * 
     * f_{samp,min} = 200 Hz * 32 samples/cycle * 10 bits / sample = 64000 Hz
	 * 
	 * The corresponding maximum allowed sampling time is
	 * 
	 * T_{samp,max} = 1 / 64000 Hz = 15.625 μs
	 * 
	 * Electrically, the ADC's sample/hold circuit and the external source
	 * impedance together form a low-pass RC filter; as a result, there is
	 * also a required minimum sampling time. Allowing for 6 time constants
	 * and 10% resistance tolerance, said minimum is
	 * 
	 * T_{samp,min} = 6τ = 6*(10k + 4.7k + 1.715k)*1.1 * 3.2 pF = 346.6848 ns
     *  
	 * 
	 * If T_{samp,min} >= T_{samp,max}, then the current configuration is
	 * not viable (will not work correctly). Either the source impedance
	 * must be lowered, and/or the sampling rate decreased.
	 * 
	 * Because the incoming ADC clock could be unnecessarily too fast, we
	 * can prescale it to slow things down. Though, said prescaler must
	 * satisfy
	 * 
	 * PSC <= f_{gclk,adc} / f_{samp,min}
     * PSC <= 4 MHz / 64000 Hz
	 * 
	 * Plugging in values yields an upper bound of 62.5; the closest
	 * available value is PSC=32. The corresponding ADC clock period is
	 * then
	 * 
     * f_{AD} = f_{gclk,adc} / PSC = 4 MHz / 32 = 125 KHz
	 * T_{AD} = 1 / f_{AD} = 1 / 125 KHz = 8μs
	 * 
	 * The minimum number of ADC clock periods required for sampling is
	 * 
	 * n_{TAD,samp} = floor(T_{samp,min}/T_{AD}) + 1
	 *              = floor(346.6848 ns / 8μs) + 1 
     *              = 20 * 10^-15 + 1
     *              = 1
	 * 
	 * Let's verify the calculations using the formula provided in the
	 * datasheet:
	 * 
	 * T_{samp,ds} = (((1.715k + 10k + 4.7k) * 3.2 pF * (10 + 2) * ln(2)) / 8μs) + 1
     *             = 1
	 * 
	 * In the event of mismatch, the worse of the two (ie. longer sampling
	 * time) must be used. Again, if the resulting sampling time is equal
	 * to or greater than the allowed maximum, the configuration is not
	 * viable and the process must be repeated with a lowered source
	 * impedance and/or sampling rate.
	 * 
	 * ====================================================================
	 * 
	 * In addition to the above, the ADC should exhibit the following
	 * behavior:
	 *	- On-demand sampling
	 *	- At least 10-bit resolution
	 *	- Right-adjusted result (for compatibility with classic
	 *	  programmingEEE 111)
	 */

	ADC_REGS->ADC_CTRLA = 0x80; // Turn on ON-DEMAND
	ADC_REGS->ADC_CTRLB = 0x4; // DIV32
    ADC_REGS->ADC_SAMPCTRL = 0x01;
	ADC_REGS->ADC_CTRLC |= (1 << 5) | (1 << 2); // applies the 3 things stated above

    // We are routing PA02, PA03, PA06, PA07, PA08, PA09, PA10, PA11 to Function B
    
    // Enable PMUX for Port A pins
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[2]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[3]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[6]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[7]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[8]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[9]  |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[10] |= (1 << 0); 
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[11] |= (1 << 0);

    // Write to PMUX (Grouping is [Pin / 2])
    // 0x11 sets both the Even and Odd pins in that register to Function B
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] = 0x11; // PA02 & PA03
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[3] = 0x11; // PA06 & PA07
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[4] = 0x11; // PA08 & PA09
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[5] = 0x11; // PA10 & PA11

    // Write the calibration value to the ADC, prior to enabling it.
    uint32_t t1 = SW_CALIB_FUSES_REGS->FUSES_SW_CALIB_WORD_0;
    uint16_t t2 = 0x0000;
    
    t2  = ((t1 & 0x00000007) << 8); // BIASREFBUF
    t2 |= ((t1 & 0x00000038) >> 3); // BIASCOMP
    ADC_REGS->ADC_CALIB = t2;
    
    // Finally, enable the ADC.
    ADC_REGS->ADC_CTRLA |= (1 << 1);
    while ((ADC_REGS->ADC_SYNCBUSY & (1 << 1)) != 0) {
        // Wait for synchronization
        asm("nop");
    }
}

uint16_t platform_adc_read(uint8_t ain_channel) 
{
    // 1. Point the ADC to the requested channel
    // 0x18 in MUXNEG connects the negative input to Internal Ground
    ADC_REGS->ADC_INPUTCTRL = (ain_channel << 0) | (0x18 << 8); 
    
    // Wait for the multiplexer to lock in
    while ((ADC_REGS->ADC_SYNCBUSY & 0x08) != 0) {
        asm("nop");
    }

    // 2. Start the conversion (Software Trigger)
    ADC_REGS->ADC_SWTRIG = (1 << 1); 
    
    // 3. Wait for the result to be ready
    
     while ((ADC_REGS->ADC_INTFLAG & (1 << 0)) == 0) {
        asm("nop");
    }
    
    // if (ADC_REGS->ADC_INTFLAG & (1 << 0) != 0) {
    
    // 4. Clear the interrupt flag and return the 10-bit result
    ADC_REGS->ADC_INTFLAG = (1 << 0);
    return (ADC_REGS->ADC_RESULT & 0x03FF); 
}

// ==========================================================================
// Motor PWM Functions

void platform_motor_set_pwm(uint8_t duty_a, uint8_t duty_b)
{
    if (duty_a > 100) duty_a = 100;
    if (duty_b > 100) duty_b = 100;

    motor_pwma_duty = duty_a;
    motor_pwmb_duty = duty_b;

    // If duty is 0, force enable pins LOW immediately.
    if (duty_a == 0) {
        PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 4);
    }

    if (duty_b == 0) {
        PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 13);
    }
}

void platform_internal_motor_pwm_systick_handler(void)
{
    uint32_t set_mask = 0;      // pins to be set HIGH
    uint32_t clear_mask = 0;    // pins to be set LOW

    motor_pwm_counter++;

    if (motor_pwm_counter >= 100) { // counter reset
        motor_pwm_counter = 0;
    }

    // PWMA
    if (motor_pwma_duty == 0) {
        clear_mask |= (1 << 4);
    } else if (motor_pwma_duty >= 100 || motor_pwm_counter < motor_pwma_duty) {
        set_mask |= (1 << 4);
    } else {
        clear_mask |= (1 << 4);
    }

    // PWMB
    if (motor_pwmb_duty == 0) {
        clear_mask |= (1 << 13);
    } else if (motor_pwmb_duty >= 100 || motor_pwm_counter < motor_pwmb_duty) {
        set_mask |= (1 << 13);
    } else {
        clear_mask |= (1 << 13);
    }

    if (set_mask != 0) { // turn on all pins in set_mask
        PORT_SEC_REGS->GROUP[0].PORT_OUTSET = set_mask;
    }

    if (clear_mask != 0) { // turn off all pins in clear_mask
        PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = clear_mask;
    }
}

// ==========================================================================
// HC-SR04 Ultrasonic Sensor Functions

// Timeout chosen for around 2 meters max.
// For wall-following, we usually only care about close walls anyway.
#define US_TIMEOUT_US       12000
#define US_NO_READING_CM    999

static void platform_delay_us(uint32_t us)
{
    /*
     * Approximate delay.
     * CPU is around 24 MHz.
     * This does not need to be perfect for wall-following.
     */
    while (us--) {
        for (volatile uint32_t i = 0; i < 12; i++) {
            asm("nop");
        }
    }
}

void platform_ultrasonic_init(void)
{
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[0] = 0; // right trig pin
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[18]  = 0; // front trig pin
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[19] = 0; // left trig pin

    // Echo pins as GPIO input, input enable ON, pull-up OFF.
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[1] = (1 << 1); // right echo pin
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[7]  = (1 << 1); // left echo pin
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[3] = (1 << 1); // front echo pin

    // Trigger pins as outputs
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 0) | (1 << 18) | (1 << 19);

    // Echo pins as inputs
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 1) | (1 << 7);
    PORT_SEC_REGS->GROUP[1].PORT_DIRCLR = (1 << 3);

    // Keep trigger pins LOW by default
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 0) | (1 << 18) | (1 << 19);
}

static uint16_t platform_ultrasonic_read_cm(
    uint32_t trig_mask,
    uint8_t echo_group,
    uint32_t echo_mask)
{
    const uint32_t TIMEOUT_LIMIT = 50000;

    uint32_t timeout_counter = 0;
    uint32_t pulse_us = 0;

    // Trigger LOW first
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = trig_mask;
    platform_delay_us(2);

    // 10 us trigger pulse
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = trig_mask;
    platform_delay_us(10);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = trig_mask;

    // Wait for echo HIGH
    timeout_counter = 0;

    while (((PORT_SEC_REGS->GROUP[echo_group].PORT_IN & echo_mask) == 0) &&
           timeout_counter < TIMEOUT_LIMIT)
    {
        timeout_counter++;
        platform_delay_us(1);
    }

    if (timeout_counter >= TIMEOUT_LIMIT) {
        return US_NO_READING_CM;
    }

    // Measure pulse width
    pulse_us = 0;

    while (((PORT_SEC_REGS->GROUP[echo_group].PORT_IN & echo_mask) != 0) &&
           pulse_us < 50000)
    {
        platform_delay_us(1);
        pulse_us++;
    }

    if (pulse_us >= 50000) {
        return US_NO_READING_CM;
    }

    return (uint16_t)(pulse_us / 4.5);
}

uint16_t platform_ultrasonic_read_right_cm(void)
{
    uint16_t cm = platform_ultrasonic_read_cm((1 << 0), 0, (1 << 1));
    platform_delay_us(60000); // small gap to reduce sensor cross-talk
    return cm;
}

uint16_t platform_ultrasonic_read_left_cm(void)
{
    uint16_t cm = platform_ultrasonic_read_cm((1 << 19), 0, (1 << 7));
    platform_delay_us(60000);
    return cm;
}

uint16_t platform_ultrasonic_read_front_cm(void)
{
    uint16_t cm = platform_ultrasonic_read_cm((1 << 18), 1, (1 << 3));
    platform_delay_us(60000); // 2000
    return cm;
}

// ==========================================================================
// Motor Control Functions

void platform_motor_forward(void)
{
    // Left motor reverse, Right motor forward = turn left
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 12);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 5);
    
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 20);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 14);
}
void platform_motor_reverse(void)
{
    // Left motor forward, Right motor reverse = turn right
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 5);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 12);

    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 14);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 20);
}

void platform_motor_stop(void)
{
    // Disable PWM
    platform_motor_set_pwm(0, 0);
    
    // All pins LOW — both motors full stop
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 4) | (1 << 5) | (1 << 12) | (1 << 13) | (1 << 20) | (1 << 14);
}

void platform_motor_left(void)
{
    // Left motor forward: AIN1 HIGH, AIN2 HIGH
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 5);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 12);
    
    // Right motor forward: BIN1 HIGH, BIN2 LOW
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 20);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 14);
    
    platform_motor_set_pwm(80, 80);
}

void platform_motor_right(void)
{
    // Left motor reverse: AIN2 HIGH, AIN1 LOW
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 12);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 5);
    
    // Right motor reverse: BIN2 HIGH, BIN1 LOW
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1 << 14);
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 20);
}
// ==========================================================================