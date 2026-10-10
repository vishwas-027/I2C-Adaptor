#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

/* Configure USART2 on PA2 (TX) and PA3 (RX) for 8-N-1 operation. */
void uart_init(uint32_t baudrate);

/* Blocking transmit helpers. They must not be called from an ISR. */
void uart_putc(char c);

void uart_puts(const char *str);

void uart_write(const void *buffer, uint32_t length);

/* Blocking receive of one byte. The CLI task waits here while idle. */
char uart_getc(void);

/* Non-blocking receive: if a byte is already waiting (RXNE set), consumes
 * it into *c and returns true; otherwise returns false immediately without
 * touching DR or blocking. There is no separate RX buffer behind this - it
 * is a direct, one-shot check of the same single-byte hardware register
 * uart_getc() reads, so at most one pending byte can ever be observed this
 * way regardless of how many arrived. Intended for cooperative cancellation
 * checkpoints inside loops that would otherwise run for a while without
 * ever calling the blocking uart_getc(). */
bool uart_try_getc(char *c);

#endif