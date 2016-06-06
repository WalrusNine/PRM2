#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <libplayerc/playerc.h>
#include "robot.h"

#define CELL_IS_FREE(value) (value <= 3)
#define CELL_IS_OCCUPIED(value) (value >= 7)

GRID* main_grid = 0;
IplImage *image = 0;

void draw_point(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
	image->imageData[3*((y*WIDTH)+x)]=b;
	image->imageData[3*((y*WIDTH)+x)+1]=g;
	image->imageData[3*((y*WIDTH)+x)+2]=r;
}

void background(IplImage* img) {
	int i, j;
	for (i = 0; i < WIDTH; ++i) {
		for (j = 0; j < HEIGHT; ++j) {
			draw_point(i, j, 255,255, 255);
		}
	}
}

void refresh_grid(GRID* g) {
	int i, j;
	for (i = 0; i < WIDTH; ++i) {
		for (j = 0; j < HEIGHT; ++j) {
			if (i % CELL_SIZE == 0) draw_point(i, j, 0, 0, 0);
			else if (j % CELL_SIZE == 0) draw_point(i, j, 0, 0, 0);
			else {
				// Inside a cell, put color
				int color = g->cells[i / CELL_SIZE][j / CELL_SIZE] * 25.5f;
				draw_point(i, j, color, color, color);
			}
		}
	}
}

void refresh_screen(GRID* g) {
	background(image);
	refresh_grid(g);

	cvNamedWindow("Test OpenCV", 1);
	cvShowImage("Test OpenCV", image);
	if (cvWaitKey(50)!=-1)  exit(0) ;
}

void clear_grid(GRID* g) {
	int i, j;
	for (i = 0; i < WIDTH/CELL_SIZE; ++i) {
		for (j = 0; j < HEIGHT/CELL_SIZE; ++j) {
			g->cells[i][j] = 5;
		}
	}
}

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

void update_main_grid(GRID* g) {
	// Put g on top of main
	int i, j;
	for (i = 0; i < WIDTH/CELL_SIZE; ++i) {
		for (j = 0; j < HEIGHT/CELL_SIZE; ++j) {
			if (CELL_IS_OCCUPIED(g->cells[i][j])) {
				main_grid->cells[i][j] += 1;
				if (main_grid->cells[i][j] > 10) main_grid->cells[i][j] = 10;
			}
			else if (CELL_IS_FREE(g->cells[i][j])) {
				main_grid->cells[i][j] -= 1;
				if (main_grid->cells[i][j] < 0) main_grid->cells[i][j] = 0;
			}
		}
	}
}

int main () {
	// Initialize grid
	GRID* g = create_grid();
	main_grid = create_grid();

	// Create image
	image  = cvCreateImage(cvSize(WIDTH,HEIGHT), IPL_DEPTH_8U, 3 );

	// Initialize robot
	ROBOT* robot = create_robot(6665);
	setup(robot);

	while(true) {
		clear_grid(g);
		robot_read(robot);
		update(robot, g);
		execute(robot);

		update_main_grid(g);

		refresh_screen(main_grid);
	}

	while (!cvWaitKey(0));

	free(g);

	return 0;

}
