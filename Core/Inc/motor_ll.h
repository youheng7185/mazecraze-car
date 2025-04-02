/*
 * motor_ll.h
 *
 *  Created on: Mar 29, 2025
 *      Author: lapchong
 */

#ifndef INC_MOTOR_LL_H_
#define INC_MOTOR_LL_H_

typedef enum {
	FORWARD,
	REVERSE,
	STOP,
	BRAKE
} motor_direction_t;

typedef enum {
	MOTOR_A,
	MOTOR_B
} motor_id_t;

void motor_init();
void motor_control(motor_id_t motor_id, motor_direction_t motor_direction);
void motor_set_speed(motor_id_t motor_id, uint8_t speed);
void encoder_get_tick(motor_id_t motor_id, int32_t *tick);

#endif /* INC_MOTOR_LL_H_ */
