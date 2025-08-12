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
