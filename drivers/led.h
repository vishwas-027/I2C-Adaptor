#ifndef LED_H
#define LED_H

/* Configure the active-low user LED connected to PC13. */
void led_init(void);
/* Drive the physical LED on or off; names abstract its active-low wiring. */
void led_on(void);
void led_off(void);
void led_toggle(void);

#endif
