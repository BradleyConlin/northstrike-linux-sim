.PHONY: run headless smoke stop

# Run with GUI (quiet: no XRCE unless you pass --uxrce)
run:
	./scripts/run_all.sh --world=default

# Headless (no GUI, no QGC) - good for CI/training
headless:
	./scripts/run_all.sh --headless --no-qgc

# Quick smoke: launch sim (no QGC), wait, run MAVSDK smoke test
smoke:
	( ./scripts/run_all.sh --world=default --no-qgc & echo $$! > .run_pid ); \
	sleep 8; \
	python3 ./scripts/smoke_mavsdk_takeoff.py || true

# Stop everything
stop:
	./scripts/stop_all.sh

# === northstrike helpers ===
SHELL := /bin/bash

.PHONY: health demo_hover

health:
	@./scripts/sim_healthcheck.sh

demo_hover: health
	@echo "Running OFFBOARD hover demo..."
	@. /opt/ros/humble/setup.bash; \
	if [ -f .ros_env ]; then set -a; source .ros_env; set +a; fi; \
	python3 scripts/ros2_offboard_hover.py
