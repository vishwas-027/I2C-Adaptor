/******************************************************************************
 * @file    uart.c
 * @brief   STM32F411 UART Driver (USART2)
 ******************************************************************************/

#include <stdint.h>
#include <string.h>

#include "../include/stm32f411.h"
#include "../board/system.h"
#include "uart.h"

/******************************************************************************
 * Configuration
 ******************************************************************************/

#define UARTx           USART2

#define UART_GPIO       GPIOA

#define UART_TX_PIN     2U
#define UART_RX_PIN     3U

#define UART_AF         7U

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void uart_gpio_init(void);
static void uart_peripheral_init(uint32_t baudrate);

/******************************************************************************
 * Public Functions
 ******************************************************************************/

void uart_init(uint32_t baudrate)
{
    uart_gpio_init();

    uart_peripheral_init(baudrate);
}

void uart_putc(char c)
{
    while ((UARTx->SR & USART_SR_TXE) == 0U)
    {
    }

    UARTx->DR = (uint8_t)c;
}

void uart_write(const void *buffer, uint32_t length)
{
    const uint8_t *ptr = (const uint8_t *)buffer;

    while (length--)
    {
        uart_putc(*ptr++);
    }

    /* Wait until last byte completely transmitted */
    while ((UARTx->SR & USART_SR_TC) == 0U)
    {
    }
}

void uart_puts(const char *str)
{
    uart_write(str, strlen(str));
}

char uart_getc(void)
{
    while ((UARTx->SR & USART_SR_RXNE) == 0U)
    {
    }

    return (char)(UARTx->DR & 0xFFU);
}

bool uart_try_getc(char *c)
{
    if ((UARTx->SR & USART_SR_RXNE) == 0U)
    {
        return false;
    }

    *c = (char)(UARTx->DR & 0xFFU);
    return true;
}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

static void uart_gpio_init(void)
{
    /* Enable GPIOA Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* ---------- PA2 -> USART2_TX ---------- */

    UART_GPIO->MODER &= ~(3UL << (UART_TX_PIN * 2));
    UART_GPIO->MODER |=  (2UL << (UART_TX_PIN * 2));

    /* ---------- PA3 -> USART2_RX ---------- */

    UART_GPIO->MODER &= ~(3UL << (UART_RX_PIN * 2));
    UART_GPIO->MODER |=  (2UL << (UART_RX_PIN * 2));

    /* Push-Pull */

    UART_GPIO->OTYPER &= ~(1UL << UART_TX_PIN);
    UART_GPIO->OTYPER &= ~(1UL << UART_RX_PIN);

    /* High Speed */

    UART_GPIO->OSPEEDR |= (3UL << (UART_TX_PIN * 2));
    UART_GPIO->OSPEEDR |= (3UL << (UART_RX_PIN * 2));

    /* No Pull-up / Pull-down */

    UART_GPIO->PUPDR &= ~(3UL << (UART_TX_PIN * 2));
    UART_GPIO->PUPDR &= ~(3UL << (UART_RX_PIN * 2));

    /* Alternate Function AF7 */

    UART_GPIO->AFR[0] &= ~(0xFUL << (UART_TX_PIN * 4));
    UART_GPIO->AFR[0] |=  (UART_AF << (UART_TX_PIN * 4));

    UART_GPIO->AFR[0] &= ~(0xFUL << (UART_RX_PIN * 4));
    UART_GPIO->AFR[0] |=  (UART_AF << (UART_RX_PIN * 4));
}

static void uart_peripheral_init(uint32_t baudrate)
{
    /* Enable USART2 Clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /*
     * BRR = Peripheral Clock / Baud Rate
     *
     * PCLK1 = 42 MHz
     */

    UARTx->BRR =
        (SystemClock.pclk1 + (baudrate / 2U)) / baudrate;

    /* 8 Data Bits
       No Parity
       1 Stop Bit
       TX Enable
       RX Enable */

    UARTx->CR1 =
          USART_CR1_TE
        | USART_CR1_RE;

    UARTx->CR2 = 0U;
    UARTx->CR3 = 0U;

    /* Enable UART */

    UARTx->CR1 |= USART_CR1_UE;
}