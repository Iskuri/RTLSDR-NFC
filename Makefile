all:
	gcc -std=gnu99 -lm $(shell pkg-config --libs librtlsdr) -o rtl_nfc main.c
