#ifndef PF_HAL_H
#define PF_HAL_H

#include "motor_ll.h"

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} pf_colour_t;

extern pf_colour_t pf_sensor_a_colour;
extern pf_colour_t pf_sensor_b_colour;
extern pf_colour_t pf_sensor_c_colour;

void printColour(pf_colour_t *colour);
void pf_motor_control(motor_id_t motor_id, motor_direction_t motor_direction);
void pf_motor_set_speed(motor_id_t motor_id, uint8_t speed);
void pf_encoder_get_tick(motor_id_t motor_id, int32_t *tick);
void pf_logging(const char *format, ...);

#endif
