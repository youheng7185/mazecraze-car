#include "main.h"
#include "stm32f4xx_hal.h"
#include "motor_ll.h"

void motor_init()
{
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // make it stop
	  HAL_GPIO_WritePin(GPIOA, motor_stdby_Pin, GPIO_PIN_SET); // make standby pin high, activate the motor driver
	  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void motor_control(motor_id_t motor_id, motor_direction_t motor_direction)
{
    switch (motor_id)
    {
    case MOTOR_A:
        switch (motor_direction)
        {
			case FORWARD:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // PB13 HIGH
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // PB12 LOW
				break;
			case REVERSE:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // PB13 LOW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // PB12 HIGH
				break;
			case BRAKE:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  // PB13 HIGH
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // PB12 HIGH
				break;
			case STOP:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);  // PB13 LOW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // PB12 LOW
				break;
			default:
				break;
        }
        break;

    case MOTOR_B:
        switch (motor_direction)
        {
			case FORWARD:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // PB13 HIGH
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // PB12 LOW
				break;
			case REVERSE:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // PB13 LOW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // PB12 HIGH
				break;
			case BRAKE:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);  // PB13 HIGH
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // PB12 HIGH
				break;
			case STOP:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);  // PB13 LOW
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // PB12 LOW
				break;
			default:
				break;
        }
        break;
    default:
        break;
    }
}

/*
 *  pls set speed from 0 to 49 only
 */

void motor_set_speed(motor_id_t motor_id, uint8_t speed)
{
	if (speed > 49) return;

	switch(motor_id) {
	case MOTOR_A:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
		break;
	case MOTOR_B:
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
		break;
	default:
		break;
	}
}

void encoder_get_tick(motor_id_t motor_id, int32_t *tick)
{
	if (tick == NULL) return;

	switch(motor_id)
	{
		case MOTOR_A:
			*tick = __HAL_TIM_GET_COUNTER(&htim4);
			break;
		case MOTOR_B:
			*tick = __HAL_TIM_GET_COUNTER(&htim5);
			break;
		default:
			*tick = 0;
			break;
	}
}

