PY ?= /usr/bin/python3
SHELL := /bin/bash
.PHONY: run debug headless stop ps

# ===== Northstrike SIM Makefile =====
# Quick commands:
#   make run     -> headless + detached (good for CI/training)
#   make debug   -> GUI + (optional) QGC, blocks until Ctrl+C
#   make stop    -> kill Gazebo/PX4/QGC
#   make headless-> headless but blocking (no QGC)
#   make ps      -> list gz/px4 processes
#
# Tune defaults here (or override at call site: `make run DRONES=3`)
DRONES ?= 2
MODEL  ?= x500
OFFSET ?= 2.0
# Set a default QGC path here if you want auto-QGC in `make debug`
# QGC_APPIMAGE ?= /home/brad/dev/QGroundControl.AppImage

PYTHON := python3
SCRIPTS := scripts
RUN_ALL := $(SCRIPTS)/run_all_linux.py
CLEAN   := $(SCRIPTS)/clean_env_linux.py

.PHONY: run debug headless stop ps

run: stop
	@echo ">>> Headless + detached launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PYTHON) $(RUN_ALL) --drones $(DRONES) --model $(MODEL) --offset $(OFFSET) --headless true --qgc false --detach true
	@echo ">>> Use 'make ps' to see running processes, 'make stop' to clean up."

# --- patched debug target (GUI) ---

# --- debug (GUI) ---
debug:
	@echo ">>> GUI debug launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PY) scripts/clean_env_linux.py
	@echo ">>> Gazebo soft-GL: $${GZ_SOFT:=0}"
	$(PY) scripts/launch_gazebo_world_linux.py
	@if [ "$(QGC)" = "true" ]; then \
		if [ -z "$(QGC_APPIMAGE)" ]; then echo "[WARN] QGC not launched: export QGC_APPIMAGE to AppImage path"; else \
			$(PY) scripts/launch_qgroundcontrol_linux.py; \
		fi; \
	fi
	$(PY) scripts/launch_px4_linux.py -n $(DRONES) -o $(OFFSET) -m $(MODEL)
# quick check of vars inside make
print-env:
	@echo "QGC_APPIMAGE=$(QGC_APPIMAGE)"
	@echo "PY=$(PY)"

# Show if we forced software GL for Gazebo
# (already added inside debug, this just gives us a convenience wrapper)
.PHONY: demo_soft
demo_soft:
	@echo ">>> demo_soft: forcing GZ_SOFT=1 for Gazebo GUI"
	GZ_SOFT=1 $(MAKE) debug QGC=true

# --- patched debug (GUI, non-blocking Gazebo, QGC optional) ---
debug:
	@echo ">>> GUI debug launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PY) scripts/clean_env_linux.py
	@echo ">>> Gazebo soft-GL: $${GZ_SOFT:=0}"
	$(PY) scripts/launch_gazebo_world_linux.py
	echo ">>> Launching QGC=$(QGC) APP=$(QGC_APPIMAGE)"
	@if [ "$(QGC)" = "true" ]; then \
		if [ -z "$(QGC_APPIMAGE)" ]; then echo "[WARN] QGC not launched: export QGC_APPIMAGE to AppImage path"; else \
			$(PY) scripts/launch_qgroundcontrol_linux.py; \
		fi; \
	fi
	$(PY) scripts/launch_px4_linux.py -n $(DRONES) -o $(OFFSET) -m $(MODEL)

# Convenience: run debug with software GL for Gazebo
.PHONY: demo_soft
demo_soft:
	@echo ">>> demo_soft: forcing GZ_SOFT=1 for Gazebo GUI"
	GZ_SOFT=1 $(MAKE) debug QGC=true

.PHONY: venv test_fly
venv:
	@python3 -m venv .venv && . .venv/bin/activate && pip -q install --upgrade pip

test_fly: venv
	@echo ">>> MAVSDK smoke test (arm → takeoff → land)"
	@. .venv/bin/activate && pip -q show mavsdk >/dev/null 2>&1 || (. .venv/bin/activate && pip -q install mavsdk)
	@. .venv/bin/activate && python3 scripts/test_fly_mavsdk.py

# --- debug (GUI, non-blocking Gazebo, QGC optional, auto-unpause) ---
debug:
	@echo ">>> GUI debug launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PY) scripts/clean_env_linux.py
	@echo ">>> Gazebo soft-GL: $${GZ_SOFT:=0}"
	$(PY) scripts/launch_gazebo_world_linux.py
	@if [ "$(AUTOPLAY)" != "0" ]; then \
		( $(PY) scripts/gz_autoplay.py & ); \
		echo ">>> Gazebo autoplay enabled (world=${GZ_WORLD:-default}, AUTOPLAY=1)"; \
	else \
		echo ">>> Gazebo autoplay disabled (AUTOPLAY=0)"; \
	fi
	echo ">>> Launching QGC=$(QGC) APP=$(QGC_APPIMAGE)"
	@if [ "$(QGC)" = "true" ]; then \
		if [ -z "$(QGC_APPIMAGE)" ]; then echo "[WARN] QGC not launched: export QGC_APPIMAGE to AppImage path"; else \
			$(PY) scripts/launch_qgroundcontrol_linux.py; \
		fi; \
	fi
	$(PY) scripts/launch_px4_linux.py -n $(DRONES) -o $(OFFSET) -m $(MODEL)

# MAVSDK quick test flights
test_fly1:
	@test -d .venv || python3 -m venv .venv
	. .venv/bin/activate && pip install -q --upgrade pip mavsdk && \
	PORT=14550 DRONE=1 python -u scripts/test_fly_mavsdk.py

test_fly2:
	@test -d .venv || python3 -m venv .venv
	. .venv/bin/activate && pip install -q --upgrade pip mavsdk && \
	PORT=14551 DRONE=2 python -u scripts/test_fly_mavsdk.py
