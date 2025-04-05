#ifndef PF_HAL_H
#define PF_HAL_H

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} pf_colour_t;

extern pf_colour_t pf_sensor_a_colour;
extern pf_colour_t pf_sensor_b_colour;
extern pf_colour_t pf_sensor_c_colour;

void printColour(pf_colour_t *colour);

#endif
