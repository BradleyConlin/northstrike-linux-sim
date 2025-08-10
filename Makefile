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
	@# If you want QGC auto-launch, ensure QGC_APPIMAGE is exported in your shell or set above.
	@# Example: make debug QGC_APPIMAGE=/home/brad/dev/QGroundControl.AppImage
	@env QGC_APPIMAGE="$(QGC_APPIMAGE)" \
	$(PYTHON) $(RUN_ALL) --drones $(DRONES) --model $(MODEL) --offset $(OFFSET) --headless false --qgc true

headless: stop
	@echo ">>> Headless (blocking) launch (DRONES=$(DRONES), MODEL=$(MODEL), OFFSET=$(OFFSET))"
	$(PYTHON) $(RUN_ALL) --drones $(DRONES) --model $(MODEL) --offset $(OFFSET) --headless true --qgc false

stop:
	@echo ">>> Cleaning up Gazebo/PX4/QGC…"
	-$(PYTHON) $(CLEAN)
	@echo ">>> Clean done."

ps:
	@echo ">>> Running gz/px4 processes:"
	@pgrep -fa "gz|px4" || echo "(none)"
