CC ?= gcc
HIPCC ?= /opt/rocm/bin/hipcc
PREFIX ?= $(HOME)/.local
VERSION = 0.3.0
ROCM ?= /opt/rocm
HIP_ARCH ?= gfx1201
CFLAGS ?= -O2 -Wall -Wextra -std=c11 -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE -Isrc
GTK := $(shell pkg-config --cflags --libs gtk+-3.0)
FRENECT := -I/usr/include/libfreenect -I/usr/include/libusb-1.0 -lfreenect -lm -lpthread
HIPFLAGS ?= -O2 --offload-arch=$(HIP_ARCH) -fPIC -Isrc
HIPLIBS := -L$(ROCM)/lib -lamdhip64 -Wl,-rpath,$(ROCM)/lib

HAVE_HIP := $(shell test -x $(HIPCC) && echo 1 || echo 0)
ifeq ($(HAVE_HIP),1)
FG_HIP_O := bin/fg_hip.o
FG_DEF := -DFG_HAVE_HIP
else
FG_HIP_O :=
FG_DEF :=
HIPLIBS :=
endif

.PHONY: all clean install

all: bin/kinect_panel bin/kinect_depth_dump bin/fg_probe

bin/fg_hip.o: src/fg/fg_hip.hip src/fg/fg_hip.h
	@mkdir -p bin
	$(HIPCC) $(HIPFLAGS) -c src/fg/fg_hip.hip -o $@

bin/kinect_panel: src/kinect_panel.c src/kinect_v4l.c src/kinect_motor.c src/kinect_bridge.c \
                  src/kinect_ir.c src/kinect_cloud_view.c src/cube_kinect_cloud.c src/fg/fg.c \
                  src/kinect_v4l.h src/kinect_motor.h src/kinect_bridge.h src/kinect_ir.h \
                  src/kinect_cloud_view.h src/cube_kinect_cloud.h src/cube_paths.h src/fg/fg.h \
                  $(FG_HIP_O)
	@mkdir -p bin
	$(CC) $(CFLAGS) $(FG_DEF) -o $@ src/kinect_panel.c src/kinect_v4l.c src/kinect_motor.c \
	  src/kinect_bridge.c src/kinect_ir.c src/kinect_cloud_view.c src/cube_kinect_cloud.c \
	  src/fg/fg.c $(FG_HIP_O) $(GTK) $(FRENECT) $(HIPLIBS)

bin/kinect_depth_dump: src/kinect_depth_dump.c
	@mkdir -p bin
	$(CC) $(CFLAGS) -o $@ src/kinect_depth_dump.c \
	  -I/usr/include/libfreenect -lfreenect_sync -lfreenect -lm

bin/fg_probe: src/fg/fg_probe.c src/fg/fg.c src/fg/fg.h $(FG_HIP_O)
	@mkdir -p bin
	$(CC) $(CFLAGS) $(FG_DEF) -o $@ src/fg/fg_probe.c src/fg/fg.c $(FG_HIP_O) $(HIPLIBS) -lm

install: all
	install -d $(PREFIX)/bin
	install -m 0755 bin/kinect_panel $(PREFIX)/bin/kinect-panel
	install -m 0755 bin/kinect_depth_dump $(PREFIX)/bin/kinect_depth_dump

clean:
	rm -f bin/kinect_panel bin/kinect_depth_dump bin/fg_probe bin/fg_hip.o
