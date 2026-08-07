/*
double-blinks, then pauses (visually distinct from left):
*/

#include "pico/stdlib.h"

int main() {
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	while (true) {
		for (int i = 0; i < 2; i++) {
			gpio_put(PICO_DEFAULT_LED_PIN, 1);
			sleep_ms(150);
			gpio_put(PICO_DEFAULT_LED_PIN, 0);
			sleep_ms(150);
		}
		sleep_ms(700);
	}
}
