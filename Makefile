CC=clang

all: mdrbootstrap

mdrbootstrap: mdrbootstrap.c
		${CC} -g -ggdb -o mdrbootstrap mdrbootstrap.c -O2
