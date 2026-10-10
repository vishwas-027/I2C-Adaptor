#include "../include/stm32f411.h"
#include "led.h"

/* The Black Pill user LED is wired active-low to PC13. */
#define LED_PIN 13U

void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER &= ~(3UL << (2 * LED_PIN));
    GPIOC->MODER |=  (1UL << (2 * LED_PIN));

    GPIOC->OTYPER &= ~(1UL << LED_PIN);
}

void led_on(void)
{
    /* Resetting the output bit drives PC13 low, lighting the LED. */
    GPIOC->BSRR = (1UL << (LED_PIN + 16));
}

void led_off(void)
{
    /* Setting the output bit drives PC13 high, turning the LED off. */
    GPIOC->BSRR = (1UL << LED_PIN);
}

void led_toggle(void)
{
    if (GPIOC->ODR & (1UL << LED_PIN))
    {
        led_on();
    }
    else
    {
        led_off();
    }
}
