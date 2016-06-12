all: main

main:
	gcc -o main robot.c main.c `pkg-config --cflags --libs opencv` -I/usr/local/include/player-3.0 -L/usr/local/lib64 -lplayerc

clean:
	rm *.o *~ main

run:
	./main
