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

debug: stop
	@echo ">>> GUI debug launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PY) scripts/clean_env_linux.py
	$(PY) scripts/launch_gazebo_world_linux.py
	@if [ "$(QGC)" = "true" ]; then \
		if [ -z "$(QGC_APPIMAGE)" ]; then echo "[WARN] QGC not launched: export QGC_APPIMAGE to AppImage path"; else \
			$(PY) scripts/launch_qgroundcontrol_linux.py; \
		fi; \
	fi
	$(PY) scripts/launch_px4_linux.py -n $(DRONES) -o $(OFFSET) -m $(MODEL)
