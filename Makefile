CC ?= gcc
PREFIX ?= $(HOME)/.local
VERSION = 0.2.0
CFLAGS ?= -O2 -Wall -Wextra -std=c11 -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE -Isrc
GTK := $(shell pkg-config --cflags --libs gtk+-3.0)
FRENECT := -I/usr/include/libfreenect -I/usr/include/libusb-1.0 -lfreenect -lm -lpthread

.PHONY: all clean install

all: bin/kinect_panel

bin/kinect_panel: src/kinect_panel.c src/kinect_v4l.c src/kinect_motor.c src/kinect_bridge.c \
                  src/kinect_ir.c src/kinect_cloud_view.c src/cube_kinect_cloud.c \
                  src/kinect_v4l.h src/kinect_motor.h src/kinect_bridge.h src/kinect_ir.h \
                  src/kinect_cloud_view.h src/cube_kinect_cloud.h src/cube_paths.h
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ src/kinect_panel.c src/kinect_v4l.c src/kinect_motor.c \
	  src/kinect_bridge.c src/kinect_ir.c src/kinect_cloud_view.c src/cube_kinect_cloud.c \
	  $(GTK) $(FRENECT)

install: all
	install -d $(PREFIX)/bin
	install -m 0755 bin/kinect_panel $(PREFIX)/bin/kinect-panel

clean:
	rm -f bin/kinect_panel

clean:
	rm -f bin/kinect_panel
