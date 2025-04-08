#ifndef ALGO_H
#define ALGO_H

typedef enum pf_grid_state_e {
	GRID_UNKNOWN = -1,
	GRID_PATH = 0,
	GRID_NO_WALL = 0,
	GRID_WALL = 1,
	GRID_VISITED_WALL = 2,
} pf_grid_state_t;

typedef enum pf_direction_e {
	TOPLEFT, TOP, TOPRIGHT,
	LEFT,            RIGHT,
	BOTLEFT, BOT, BOTRIGHT
} pf_direction_t;

/*
Grid 12x12
wall = 12 + 1 = 13 per line
mixed = 25 * 25, even = wall, odd = grid
*/

typedef struct pf_vec2_s
{
	int x;
	int y;
} pf_vec2_t;

typedef pf_vec2_t pf_cord_t;
typedef int** pf_grid_t;

typedef struct pf_map_s {
	int size;
	pf_grid_t grid;
	pf_cord_t cord;
	pf_direction_t direction;
} pf_map_t;

typedef int t_distance;

pf_map_t *pf_map_new(void);
void pf_map_update_robot_pos(pf_map_t *map, pf_cord_t pos, pf_cord_t velocity);
void pf_map_update_left_distance(pf_map_t *map, t_distance distance);
void pf_map_update_right_distance(pf_map_t *map, t_distance distance);
void pf_map_update_front_distance(pf_map_t *map, t_distance distance);
void pf_map_update_back_distance(pf_map_t *map, t_distance distance);
pf_cord_t pf_map_find_closest_unscanned(pf_map_t *map, t_distance distance);
pf_cord_t pf_map_find_closest_unknown(pf_map_t *map, t_distance distance);
pf_cord_t pf_map_find_closest_objective(pf_map_t *map, t_distance distance); // closest of unscanned and unkown
// int pf_map_calculate_cost(pf_map_t *map, pf_cord_t target_cord);

#endif  // ALGO_H
