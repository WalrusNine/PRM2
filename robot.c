#include "robot.h"

#include <math.h>

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
	
	
	r->max_speed = 2.0f;
	
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
    g->cells[x0][y0] = 10;
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
	int i;
	for (i = 0; i < 360; i++) {
		float range = r->laser->scan[i][0];
		float angle = r->laser->scan[i][1];
		float ang_rot = angle + r->position2d->pa;
		float diff_ok = atan2(sin(ang_rot), cos(ang_rot));

		// TODO: use round function
		int x1 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4;
		int y1 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3;
		int x2 = (WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4 + range * cos(-diff_ok) * 4;
		int y2 = (HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3 + range * sin(-diff_ok) * 3;
		if (x2 > WIDTH) x2 = WIDTH-1;
		if (y2 > HEIGHT) y2 = HEIGHT-1;
		draw_line(g, x1, y1, x2, y2);
		if (distance(x1 / 4, y1 / 3, x2 / 4, y2 / 3) < 8) {
				g->cells[x2][y2] = 0;
		}
		
	}

	float dx = (r->laser->point[1].px);
	float dy = (r->laser->point[1].py);
	float range = r->laser->ranges[1];
	int x1 = r->position2d->px;
	int y1 = r->position2d->py;
	int x2 = r->position2d->px + dx;
	int y2 = r->position2d->py - dy;
	
	float ac_x2 = r->position2d->px + range * cos(r->position2d->pa);
	float ac_y2 = r->position2d->py + range * sin(r->position2d->pa);

	/*DEBUG((float)x1);
	DEBUG((float)y1);
	DEBUG(ac_x2);
	DEBUG(ac_y2);*/

	//DEBUG((float)x2);
	//DEBUG((float)y2);
	
	g->cells[(int)((WIDTH/2.f)/CELL_SIZE + r->position2d->px * 4)][(int)((HEIGHT/2.f)/CELL_SIZE - r->position2d->py * 3)] = 10;
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
	//playerc_position2d_set_cmd_vel(r->position2d, r->vlong, 0, r->vrot, 1);
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
