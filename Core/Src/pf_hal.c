#include "stm32f4xx_hal.h"
#include "main.h"
#include "my_print.h"
#include "tcs32725.h"
#include "pf_hal.h"
#include "motor_ll.h"
#include "tca9548.h"

pf_colour_t pf_sensor_a_colour;
pf_colour_t pf_sensor_b_colour;
pf_colour_t pf_sensor_c_colour;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // Select channel for sensor A, then read RGB values
    selectTCAChannel(0);
    getRGB(&pf_sensor_a_colour.r, &pf_sensor_a_colour.g, &pf_sensor_a_colour.b);

    // Select channel for sensor B, then read RGB values
    selectTCAChannel(1);
    getRGB(&pf_sensor_b_colour.r, &pf_sensor_b_colour.g, &pf_sensor_b_colour.b);

    // Select channel for sensor C, then read RGB values
    selectTCAChannel(2);
    getRGB(&pf_sensor_c_colour.r, &pf_sensor_c_colour.g, &pf_sensor_c_colour.b);
  }
}

uint32_t count_sensor = 0;
void printColour(pf_colour_t *colour) {
    my_printf("RGB Values from %d: R = %d, G = %d, B = %d\r\n", (count_sensor%3), colour->r, colour->g, colour->b);
    count_sensor++;
}

void pf_motor_control(motor_id_t motor_id, motor_direction_t motor_direction)
{
	motor_control(motor_id, motor_direction);
}

void pf_motor_set_speed(motor_id_t motor_id, uint8_t speed)
{
	motor_set_speed(motor_id, speed);
}

void pf_encoder_get_tick(motor_id_t motor_id, int32_t *tick)
{
	encoder_get_tick(motor_id, tick);
}

void pf_logging(const char *format, ...)
{
	my_printf(format);
}
