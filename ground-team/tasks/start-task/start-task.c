#include "include/start-task.h"
#include <time.h>
#include <stdio.h>

time_t inline clock_to_millis(time_t clock) {
	return clock / CLOCKS_PER_SEC * 1000;
}

time_t measure_pulse_width_millis(int handle) {
	time_t start_time = clock_to_millis(clock());
	while (lgGpioRead(handle, PHOTORESISTOR_GPIO) == 0);
	time_t end_time = clock_to_millis(clock());

	return end_time - start_time;
}

int init_gpio_input() {
	int handle = lgGpiochipOpen(0);
	if (handle < 0) {
		fprintf(stderr, "Failed to open GPIO chip: %s\n", lguErrorText(handle));
		return ERR_BUS_FAIL;
	}

	int status = lgGpiochipClaimInput(handle, 0, PHOTORESISTOR_GPIO);
	if (rc < 0) {
		fprintf(stderr, "Failed to claim GPIO %d: %s", pins[i], lguErrorText(rc));
		lgGpiochipClose(handle);
		return ERR_WRITE_FAIL;
	}

	return handle
}

void wait_for_light() {
	int handle = init_gpio_input();
	if (handle < 0) {
		perror("wait_for_light");
		return;
	}
	
	printf("Waiting for led.\n");

	time_t pulse_width;
	while (pulse_width < MAX_CYCLE_TO_START_MILLIS) {
		pulse_width = measure_pulse_width_millis(handle);
	}
}
