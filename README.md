
# Mazecraze car

A brief description of what this project does and who it's for


## API Reference

### Get colour value from colour sensor

```
pf_colour_t pf_sensor_a_colour;
pf_colour_t pf_sensor_b_colour;
pf_colour_t pf_sensor_c_colour;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} pf_colour_t;
```

the value will update every 100ms, no need additional function to update value manually

| Parameter | Type     | Description                |
| :-------- | :------- | :------------------------- |
| `r` | `uint8_t` | r value from 0 to 255 |
| `g` | `uint8_t` | g value from 0 to 255 |
| `b` | `uint8_t` | b value from 0 to 255 |

### Wheel control

```
void pf_motor_control(motor_id_t motor_id, motor_direction_t motor_direction)
```

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `motor_id`      | `motor_id_t` | `MOTOR_A` or `MOTOR_B` |
| `motor_direction`      | `motor_direction_t` | `FORWARD`, `REVERSE`, `BRAKE`, `STOP` (pls dont use brake for long time)|

```
void pf_motor_set_speed(motor_id_t motor_id, uint8_t speed)
```
| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `motor_id`      | `motor_id_t` | `MOTOR_A` or `MOTOR_B` |
| `speed`      | `uint8_t` | fill value between 0 to 49 inclusive, 0 is stop, 49 is max speed, this is pwm control underlying |

```
void pf_encoder_get_tick(motor_id_t motor_id, int32_t *tick)
```
| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `motor_id`      | `motor_id_t` | `MOTOR_A` or `MOTOR_B` |
| `tick`      | `int32_t *` | return encoder value, 2700 for a single turn, negative means revrese direction |

### Logging

```
void pf_logging(const char *format, ...)
```
| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `format`      | `char *` | use it as printf |



