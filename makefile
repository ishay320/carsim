LIBS=-lraylib -lm
CFLAGS=-L./raylib/src/ -I./raylib/src/ -ggdb

main: main.c config.o raylib
	gcc -o main main.c config.o ${LIBS} ${CFLAGS}

config.o: config.c config.h

raylib:
	git clone --depth 1 https://github.com/raysan5/raylib.git raylib
	cd raylib/src/
	make PLATFORM=PLATFORM_DESKTOP
