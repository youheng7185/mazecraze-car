/*
 * qmc5883p.h
 *
 *  Created on: Apr 5, 2025
 *      Author: lapchong
 */

#ifndef INC_QMC5883P_H_
#define INC_QMC5883P_H_

#include <stdbool.h>

bool qmc5883p_init();
void qmc5883p_read_all(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);

#endif /* INC_QMC5883P_H_ */
