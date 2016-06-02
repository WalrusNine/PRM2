#ifndef ROBOT_H
#define ROBOT_H

#include <libplayerc/playerc.h>

#define WIDTH 640
#define HEIGHT 480
#define CELL_SIZE 10

#define DEBUG(msg) printf("DEBUG: %f - %d, %s\n", msg, __LINE__, __FILE__)

typedef struct matrix {
	int cells[WIDTH/CELL_SIZE][HEIGHT/CELL_SIZE];
} GRID;

typedef struct robot {
	// State
	int state;
	
	// 6665, 6666, 6667
	int port;
	
	// Config
	playerc_client_t* 		client;
	playerc_position2d_t* 	position2d;
	playerc_laser_t* 		laser;
	playerc_blobfinder_t* 	bf;			// Camera
	
	// Properties
	float dest_x, dest_y;	// Destination
	float vlong, vrot;
	float initial_x, initial_y;
	
	float max_speed;
	
} ROBOT;

ROBOT* create_robot (int port);

int setup (ROBOT* r);

void robot_read (ROBOT* r);

void update (ROBOT* r, GRID* g);

void turn_left (ROBOT* r);

void turn_right (ROBOT* r);

int go_to (ROBOT* r, float x, float y);

float diff (float a, float b);

void execute (ROBOT* r);

void delete_robot (ROBOT* r);

void no_turn (ROBOT* r);

float distance (float x1, float y1, float x2, float y2);

void set_speed (ROBOT* r, float val);

void draw_line (GRID* g, int x1, int y1, int x2, int y2);

#endif













