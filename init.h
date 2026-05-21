/**
 * @file platform.h
 *
 * Initial version:  EEE 158 AY2025-26 Sem 1 Handlers
 * Modifications by: SIBUG, Tristan Josh E., 2021-60143, WYZ
 *
 * Platform-support functionality, core
 */

/*
 * The outermost #if-#endif block makes sure that if this header file is
 * included multiple times (virtually always the case in production code), we
 * don't get multiple-definition errors from the compiler.
 *
 * (C/C++ Standard, "One-definition Rule" [ODR])
 */
#ifndef EEE158_PLATFORM_INIT_H
#define EEE158_PLATFORM_INIT_H

#include <stdint.h>
#include <limits.h>

// C99 compatibility with C++/C23's ``bool`` datatype
#if !defined(__cplusplus) && __STDC_VERSION__ < 202311l
#define bool _Bool
#define true (1)
#define false (0)
#endif

/*
 * The "extern 'C'" block makes sure that, if this file gets included from
 * C++, name mangling is not performed.
 *
 * (You can search the C++ standard to find out about name mangling.)
 */
#ifdef __cplusplus
extern "C" {
#endif

	/**
	 * Perform early platform initialization
	 * 
	 * @details
	 * The initialization phase is split into "early" and "late" parts.
	 * Once the full initialization process is complete, interrupts are
	 * enabled and the code must be ready to process them anytime. For this
	 * reason, the application must initialize all it needs between this
	 * function and platform_init_late().
	 * 
	 * @remarks
	 * Upon return from this function, the platform will have the following
	 * characteristics:
	 *	- CPU clock boosted to 24 MHz;
	 *	- GCLK_GEN2 configured and enabled for 4 MHz from OSC16M;
	 *	- EIC & EVSYS reset; and
	 *	- EIC's generic-clock input enabled.
	 */
	extern void platform_init_early(void);
	
	/**
	 * Perform late platform initialization
	 * 
	 * @note
	 * See platform_init_early() for the rationale behind this function.
         */
	extern void platform_init_late(void);

    /**
     * Trigger a system reset
     * 
     * @note
     * This function never returns.
     */
    extern void __attribute__((noreturn)) platform_do_sys_reset(void);
    
	// ==================================================================

	/// Simple descriptor for a constant-content buffer
	struct platform_ro_buf_desc {
		/// First byte in the buffer
		const char *buf;

		/// Number of bytes in the buffer
		unsigned int len;
	};

	/**
	 * Declare a buffer containing the given constant C string
	 *
	 * @param v_name	Variable name
	 * @param str		String contents
	 */
#define DECLARE_PLATFORM_RO_STR_DESC(v_name, str)	\
	struct platform_ro_buf_desc ## v_name ## = {	\
		.buf = (str),				\
		.len = sizeof(str)-1			\
	}

	// ==================================================================

	/**
	 * Add platform-event flags for retrieval via platform_evt_get()
	 * 
	 * This function is intended to be called from IRQ handlers.
	 *
	 * @note
	 * The set of event flags is completely application-defined. When
	 * defining flags, make sure no two flags use the same bit.
         */
	extern void platform_evt_add(int evt);
	
	/**
	 * Get available platform-event flags
	 *
	 * This function is intended to be called from within the main infinite
	 * loop.
	 *
	 * @note
	 * The set of event flags is completely application-defined. In
	 * addition, this function clears all added event flags so that they
	 * will not appear again until re-added via a call to
	 * platform_evt_add().
	 */
	extern int platform_evt_get(void);

	/// If set, the on-board button was pressed
#define PLATFORM_EVT_PB_PRESS	(0x00000001)
    
	/// Initialize the GPIO
	extern void platform_gpio_init(void);

	// ==================================================================

	/// Get the current number of SysTick occurrences, as a raw value
	extern uint32_t platform_systick_count(void);

	/// Get the difference between two ticks, considering overflow
	extern uint32_t platform_tick_delta(uint32_t lhs, uint32_t rhs);
	
	/// Number of milliseconds corresponding to one SysTick occurrence
#define PLATFORM_MS_PER_SYSTICK	(10)

    // Motor Control Functions
    extern void platform_motor_forward(void);
    extern void platform_motor_reverse(void);
    extern void platform_motor_stop(void);
    extern void platform_motor_left(void);
    extern void platform_motor_right(void);
    extern void platform_motor_set_pwm(uint8_t duty_a, uint8_t duty_b);
    
    extern void platform_ADC_init(void);
    extern uint16_t platform_adc_read(uint8_t ain_channel);

    
    // Ultrasonic Functions
    extern void platform_ultrasonic_init(void);
    extern uint16_t platform_ultrasonic_read_right_cm(void);
    extern uint16_t platform_ultrasonic_read_left_cm(void);
    extern uint16_t platform_ultrasonic_read_front_cm(void);
	// ==================================================================
#ifdef __cplusplus
}
#endif
#endif
