#!/usr/bin/env bash
REPO="$HOME/dev/northstrike-linux-sim"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${REPO}/simulation:${REPO}/simulation/models:${REPO}/simulation/worlds"
echo "[ns-sim-env] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
