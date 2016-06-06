#include "robot.h"

#include <math.h>

POINT* go_to_pos;
int go_to_pos_index = 0;

GRID* walked_grid = 0;

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


	// Create go_to_pos
	go_to_pos = malloc(1000 * sizeof(POINT));
	walked_grid = create_grid();

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
	if (g->cells[x0][y0] > 10) g->cells[x0][y0] = 10;

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
	// Get obstacles
	go_to_pos_index = 0;
	int i;
	for (i = 0; i < 360; i++) {
		float range = r->laser->scan[i][0];
		float angle = r->laser->scan[i][1];
		float ang_rot = angle + r->position2d->pa;
		float diff_ok = atan2(sin(ang_rot), cos(ang_rot));

		int x1 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4;
		int y1 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3;
		int x2 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4 + range * cos(-diff_ok) * 4;
		int y2 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3 + range * sin(-diff_ok) * 3;
		if (x2 > WIDTH) x2 = WIDTH-1;
		if (y2 > HEIGHT) y2 = HEIGHT-1;
		draw_line(g, x1, y1, x2, y2);

		if (range < 8) {
			g->cells[x2][y2] = 0;
		}

		if (range > go_to_pos[go_to_pos_index].range) {
			// Add to list of to go squares
			go_to_pos[go_to_pos_index].x = x2;
			go_to_pos[go_to_pos_index].y = y2;
			go_to_pos[go_to_pos_index].range = range;
		}

	}

	// Current position is empty
	int cur_x = (int)((WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4);
	int cur_y = (int)((HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3);
	g->cells[cur_x][cur_y] = 10;

	// Go to a go_to_pos
	// Get
	/*int x = go_to_pos[go_to_pos_index].x;
	int y = go_to_pos[go_to_pos_index].y;
	if (go_to_pos[go_to_pos_index].range > 5 && walked_grid->cells[x][y] != 10) {
		go_to(r, x, y);
		// Add current position to walked_grid
		walked_grid->cells[cur_x][cur_y] = 10;
	}
	else {
		// No destiny, stop and turn until
		set_speed(r, 0);
		turn_left(r);
	}*/
	// Add current position to walked_grid
	//walked_grid->cells[cur_x][cur_y] = 10;

	POINT d = find_pos(main_grid);
	if (d.x > -1) {
		int x = (d.x - (WIDTH/2.f)/CELL_SIZE)/4;
		int y = (-d.y + (HEIGHT/2.f)/CELL_SIZE)/3;
		DEBUG((float)x);
		DEBUG((float)y);

		go_to(r, x, y);
	}
	else {
		// No destiny, stop and turn until
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

	//printf("AngRot: %f\n", ang_rot);

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
}

void wander (ROBOT* r, GRID* g) {
	// Look for unknow square in grid
	// Do dfs/bfs algorithm to see if can go
	// If now can, try to go
	// If can't, repeat for next square
}

/*int round (float n) {

}*/

POINT find_pos (GRID* m) {
	// Loop, if finds unknow, check surrounding, if is free go
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

			if (found) {
				p.x = i;
				p.y = j;
				return p;
			}
		}
	}
}
