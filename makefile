LIBS=-lraylib -lm
CFLAGS=-L./raylib/src/ -I./raylib/src/ -ggdb

main: main.c config.o physics.o graph.o raylib
	gcc -o main main.c config.o physics.o graph.o ${LIBS} ${CFLAGS}

config.o: config.c config.h
physics.o: physics.c physics.h
graph.o: graph.c graph.h

raylib:
	git clone --depth 1 https://github.com/raysan5/raylib.git raylib
	cd raylib/src/
	make PLATFORM=PLATFORM_DESKTOP
