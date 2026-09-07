# Thin wrapper around CMake. The project is CMakeLists.txt.
BUILD ?= build
PRESET ?= default

.PHONY: all configure install clean

all: configure
	cmake --build "$(BUILD)"

configure:
	@if [ -f CMakePresets.json ] && command -v ninja >/dev/null; then \
	  cmake --preset "$(PRESET)"; \
	else \
	  cmake -S . -B "$(BUILD)" -DCMAKE_BUILD_TYPE=Release; \
	fi

install: all
	cmake --install "$(BUILD)" --prefix "$(HOME)/.local"

clean:
	cmake --build "$(BUILD)" --target clean 2>/dev/null || true
	rm -rf "$(BUILD)"
