LIBS=-lraylib -lm
CFLAGS=-L./raylib/src/ -I./raylib/src/

main: main.c
	gcc -o main main.c ${LIBS} ${CFLAGS}


raylib:
	git clone --depth 1 https://github.com/raysan5/raylib.git raylib
	cd raylib/src/
	make PLATFORM=PLATFORM_DESKTOP
