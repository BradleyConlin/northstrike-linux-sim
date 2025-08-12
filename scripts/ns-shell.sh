#!/usr/bin/env bash
python3 ~/dev/px4-autopilot-harmonic/Tools/mavlink_shell.py udpout:127.0.0.1:14580
# (Use udpout so we don't bind to PX4's port. Offboard tools normally listen on 14540; QGC on 14550.)
