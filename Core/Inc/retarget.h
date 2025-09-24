#ifndef RETARGET_H
#define RETARGET_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- printf redirection (UART-backed) ------------------------------------ */
/**
 * Low-level character output used by printf.
 * Inserts a '\r' before '\n' for terminal compatibility.
 */
int __io_putchar(int ch);

/**
 * Direct write used by some printf libraries to flush a block.
 * Safe to call with HAL; may block until UART completes.
 */
int __io_write(const char *buf, int len);

#ifdef __cplusplus
}
#endif
#endif /* RETARGET_H */
