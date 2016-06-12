#include "robot.h"

#include <math.h>

GRID* known_grid = 0;
int cur_destination_count = 0;
POINT cur_destination;

GRID* create_grid() {
	GRID* m = malloc(sizeof(GRID));

	int i, j;
	for (i = 0; i < WIDTH/CELL_SIZE; ++i) {
		for (j = 0; j < HEIGHT/CELL_SIZE; ++j) {
			m->cells[i][j] = 5;
		}
	}

	return m;
}

ROBOT* create_robot (int port) {
	ROBOT* r = malloc(sizeof(ROBOT));

	int full_port = port;

	r->client = playerc_client_create (NULL, "localhost", full_port);
	if (playerc_client_connect(r->client) != 0) {
		fprintf(stderr, "Error: %s\n", playerc_error_str());
		free(r);
		return NULL;
	}

	r->port = full_port;				// Identify the robot


	r->max_speed = 0.5f;

	r->vlong = 0;
	r->vrot = 0;

	// Setup initial pos
	r->initial_x = 0;
	if (port == 5) {
		r->initial_y = -1;
	}
	else if (port == 6) {
		r->initial_y = 0;
	}
	else if (port == 7) {
		r->initial_y = 1;
	}

	setup(r);

	robot_read(r);
	robot_read(r);
	robot_read(r);
	robot_read(r);

	known_grid = create_grid();

	cur_destination.x = -1;
	cur_destination.y = -1;

	return r;
}

int setup (ROBOT* r) {
	// Position
	r->position2d = playerc_position2d_create(r->client, 0);
	if (playerc_position2d_subscribe(r->position2d, PLAYERC_OPEN_MODE) != 0) {
		fprintf(stderr, "Error: %s\n", playerc_error_str());
		return 0;
	}
	else {
		// Activate
		playerc_position2d_enable(r->position2d, 1);
	}

	// Laser
	r->laser = playerc_laser_create(r->client, 0);
	if (playerc_laser_subscribe(r->laser, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "Error: %s\n", playerc_error_str());
		return 0;
	}

	// Camera
	r->bf = playerc_blobfinder_create(r->client, 0);
	if (playerc_blobfinder_subscribe(r->bf, PLAYER_OPEN_MODE)) {
		fprintf(stderr, "Error: %s\n", playerc_error_str());
		return 0;
	}

	return 1;
}

void robot_read (ROBOT* r) {
	playerc_client_read(r->client);
}

void draw_line(GRID* g, int x0, int y0, int x1, int y1) {
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
  int err = (dx>dy ? dx : -dy)/2, e2;

  for(;;){
    g->cells[x0][y0] += 1;
	if (g->cells[x0][y0] > 10) {
		g->cells[x0][y0] = 10;
		known_grid->cells[x0][y0] = 10;
	}

    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}

/*
** Get robot's angle, set points accordingly
**
*/
void update (ROBOT* r, GRID* g) {
	// Check laser / update grid
	int i;
	float front = 8;
	for (i = 0; i < 360; i++) {
		float range = r->laser->scan[i][0];
		float angle = r->laser->scan[i][1];
		float ang_rot = angle + r->position2d->pa;
		float diff_ok = atan2(sin(ang_rot), cos(ang_rot));

		int x1 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4;
		int y1 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3;
		int x2 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4 + range * cos(-diff_ok) * 4;
		int y2 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3 + range * sin(-diff_ok) * 3;
		if (x2 >= WIDTH/CELL_SIZE) {
			x2 = (WIDTH/CELL_SIZE) - 1;
		}
		if (y2 >= HEIGHT/CELL_SIZE) {
			y2 = (HEIGHT/CELL_SIZE) - 1;
		}

		if (i > 150 && i < 210) {
			if (front > range) front = range;
		}

		draw_line(g, x1, y1, x2, y2);

		if (range < 8) {
			g->cells[x2][y2] = 0;
			known_grid->cells[x2][y2] = 10;
		}

	}

	// Current position is empty
	int cur_x = (int)((WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4);
	int cur_y = (int)((HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3);
	g->cells[cur_x][cur_y] = 10;

	// Find pos to go
	POINT d = find_pos(main_grid);
	// If is already going there
	if (d.x == cur_destination.x && d.y == cur_destination.y) {
		// Add to count
		cur_destination_count++;
		// If has been trying to go there for too long, give up
		if (cur_destination_count > 200) {
			known_grid->cells[d.x][d.y] = 10;
		}
	}
	else {
		// If is new destination, set it and its count
		cur_destination_count = 0;
		cur_destination = d;
	}

	// If found a point and there's nothing in front
	if (d.x > -1 && front > 1) {
		int x = (d.x - (WIDTH/2.f)/CELL_SIZE)/4;
		int y = (-d.y + (HEIGHT/2.f)/CELL_SIZE)/3;

		go_to(r, x, y);
	}
	else {
		// No good destiny, stop and turn until finds new destiny
		set_speed(r, 0);
		turn_left(r);
	}
}

void no_turn (ROBOT* r) {
	r->vrot = 0;
}

void turn_left (ROBOT* r) {
	r->vrot = 0.4f;
}

void turn_right (ROBOT* r) {
	r->vrot = -0.4f;
}

void set_speed (ROBOT* r, float val) {
	r->vlong = val;
}

int go_to (ROBOT* r, float x, float y) {
	r->dest_x = x;
	r->dest_y = y;

	r->vlong = r->max_speed;

	//Calcula vel longitudinal
	float dist = distance(r->position2d->px, r->position2d->py, x, y);
	if (dist < 0.8) {
		return 1;
	}

	//Calcula força de atração
	float angdest = atan2(y - r->position2d->py, x - r->position2d->px);
	float ang_rot = angdest - r->position2d->pa;
	float diff_ok = atan2(sin(ang_rot), cos(ang_rot));

	r->vrot = diff_ok;

	//Calcula força de repulsão
	float campo_obst=0;

	int i;
	for(i=200; i<=360; i+=30) {
		if (r->laser->ranges[i] < 2.0)
			campo_obst += 2.0 - r->laser->ranges[i];
	}

	for(i=160; i>=0; i-=30) {
		if (r->laser->ranges[i] < 2.0)
			campo_obst -= 2.0 - r->laser->ranges[i];
	}

	r->vrot -= 0.5 * campo_obst;

	return 0;
}

float diff (float a, float b) {
	float res = a - b;

	if (res < 0) return -res;
	else return res;
}

float distance (float x1, float y1, float x2, float y2) {
	return sqrt(((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)));
}

void execute (ROBOT* r) {
	playerc_position2d_set_cmd_vel(r->position2d, r->vlong, 0, r->vrot, 1);
}

void delete_robot (ROBOT* r) {
	// Disconnect
	playerc_position2d_unsubscribe(r->position2d);
	playerc_position2d_destroy(r->position2d);

	playerc_laser_unsubscribe(r->laser);
	playerc_laser_destroy(r->laser);

	playerc_blobfinder_unsubscribe(r->bf);
	playerc_blobfinder_destroy(r->bf);

	playerc_client_disconnect(r->client);
	playerc_client_destroy(r->client);

	// Free memory
	free(r);

	free(known_grid);
}

/*
** Check grid
** If finds an unknown position, check if surroundings
** (up, left, right and down) are free
** If is, assume robot can go to it, unless
** it's a known position already (set by known_grid).
*/
POINT find_pos (GRID* m) {
	int i, j, w, h, found;
	w = WIDTH/CELL_SIZE;
	h = HEIGHT/CELL_SIZE;
	found = 0;

	POINT p;
	p.x = -1;
	p.y = -1;

	for (i = 0; i < w; ++i) {
		for (j = 0; j < h; ++j) {
			if (m->cells[i][j] > 3 && m->cells[i][j] < 7) {
				// Check up, down, left and right
				// UP
				if (j-1 > 0 && m->cells[i][j-1]) {
					// UP
					if (m->cells[i][j-1] >= 7) {
						found = 1;
					}
				}
				else if (j+1 < h && m->cells[i][j+1]) {
					// DOWN
					if (m->cells[i][j+1] >= 7) {
						found = 1;
					}
				}
				else if (i-1 > 0 && m->cells[i-1][j]) {
					// LEFT
					if (m->cells[i-1][j] >= 7) {
						found = 1;
					}
				}
				else if (i+1 < w && m->cells[i+1][j]) {
					// RIGHT
					if (m->cells[i+1][j] >= 7) {
						found = 1;
					}
				}
			}

			if (found && known_grid->cells[i][j] != 10) {
				p.x = i;
				p.y = j;
				return p;
			}
			else {
				found = 0;
			}
		}
	}

	return p;
}
