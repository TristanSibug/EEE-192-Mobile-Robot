/**
 * @file platform.h
 *
 * Initial version:  EEE 158 AY2025-26 Sem 1 Handlers
 *
 * Function prototypes utilized across the implementation files; not to be
 * used outside the source files in this directory
 *
 * NOTE: If a function is only used in a single file, it is not declared
 *       here. Rather, it is declared `static` there.
 */

#ifndef EEE158_PLATFORM_IMPL_H
#define EEE158_PLATFORM_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

	// usart.c
	extern void platform_internal_usart_cdc_systick_handler(void);
    
    // gpio.c
    extern void platform_internal_motor_pwm_systick_handler(void);
    
#ifdef __cplusplus
}
#endif
#endif
