/**
 * @file platform.h
 *
 * Initial version:  EEE 158 AY2025-26 Sem 1 Handlers
 * Modifications by: SIBUG, Tristan Josh E., 2021-60143, WYZ
 *
 * Platform-support functionality, USART
 */

/*
 * The outermost #if-#endif block makes sure that if this header file is
 * included multiple times (virtually always the case in production code), we
 * don't get multiple-definition errors from the compiler.
 *
 * (C/C++ Standard, "One-definition Rule" [ODR])
 */
#ifndef EEE158_PLATFORM_USART_H
#define EEE158_PLATFORM_USART_H

#include "init.h"

#ifdef __cplusplus
extern "C" {
#endif

	/// Initialize the USART
	extern void platform_usart_cdc_init(void);
	
	/**
	 * Transfer a "vector" of strings via the USART
	 *
	 * @param	vec	Address of first buffer descriptor
	 * @param	nr	Number of buffer descriptors
	 *
	 * @return
	 * @c true if the descriptors are successfully enqueued,
	 * @c false otherwise. The latter is returned when an existing
	 * transaction has not yet completed.
	 *
	 * @warning
	 * While the descriptors themselves may be reused after this call, the
	 * underlying buffers must remain valid and unmodified while transmission
	 * is on-going!
	 */
	extern bool platform_usart_cdc_send_async(
		const struct platform_ro_buf_desc *vec, unsigned int nr
	);

	/// Maximum number of vectors allowed for platform_usart_cdc_send_async()
#define PLATFORM_USART_NR_TX_VEC_MAX    (16)
    
	/// Abort a transfer initiated via platform_usart_cdc_send_async()
	extern void platform_usart_cdc_send_abort(void);
    
    /// Is the transmitter busy?
    extern bool platform_usart_cdc_tx_busy(void);

	/**
	 * Initiate reception of some buffer via USART
	 *
	 * @param	buf	Address of first byte in buffer to hold the
	 * 			received data
	 * @param	len	Maximum anticipated length of the buffer, in
	 * 			bytes
	 *
	 * @return
	 * @c true if the transaction is successfully enqueued,
	 * @c false otherwise. The latter is returned when an existing
	 * transaction has not yet completed.
	 *
	 * @warning
	 * All underlying buffers + descriptors must remain valid while
	 * transmission is on-going!
	 */
	extern bool platform_usart_cdc_recv_async(char *buf, unsigned int len);

	/// Abort a transfer initiated via platform_usart_cdc_recv_async()
	extern void platform_usart_cdc_recv_abort(void);

	/// Is the receiver busy?
	extern bool platform_usart_cdc_rx_busy(void);
    
	/// Information about a completed receive-transaction
	struct platform_usart_recv_data_type {
		/**
		 * First byte in the buffer
		 *
		 * @note
		 * This is the same buffer passed in the call to
		 * platform_usart_cdc_recv_async().
		 */
		union {
			const char *buf;
			char *w_buf;
		};

		/// Number of bytes in the buffer
		unsigned int len;

		/// If set, a parity error occurred
		int err_parity:1;

		/// If set, a framing error occurred
		int err_framing:1;

		/// If set, an overrun error occurred
		int err_overflow:1;

		/// If set, a collision error occurred
		int err_collision:1;
        
		/// If set a break condition occurred
		int err_break:1;

		// Padding
		int _err_pad:27;
	};

	/**
	 * Check if there is data to be read from the USART
	 *
	 * @param	desc	Descriptor for the read data
	 *
	 * @return
	 * @c true if the recent transaction has just completed with at least
	 * one byte, @c false otherwise
	 *
	 * @note
	 * The descriptor returned in `desc` points to the buffer supplied with
	 * the corresponding call to platform_usart_cdc_recv_async().
	 *
	 * @note
	 * This function "consumes" the completion; that is, if this function
	 * had returned @c true and no new transaction was enqueued in the
	 * time being, @c false will be returned.
	 */
	extern bool platform_usart_cdc_recv_get(struct platform_usart_recv_data_type *desc);


#ifdef __cplusplus
}
#endif
#endif
