#!/bin/bash

##
## Copyright (c) 2024 Numurus <https://www.numurus.com>.
##
## This file is part of nepi setup tools (nepi_setup) repo
## (see https://github.com/nepi-engine/nepi_setup)
##
## License: nepi setup tools are licensed under the "Numurus Software License",
## which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
##
## Redistributions in source code must retain this top-level comment block.
## Plagiarizing this software to sidestep the license obligations is illegal.
##
## Contact Information:
## ====================
## - mailto:nepi@numurus.com
##

# Turnkey entry point for the Gazebo simulation subsystem. Launches Gazebo
# itself against the world staged in SIM_ENV (with SIM_ROBOT added to
# GAZEBO_MODEL_PATH so any robot models staged there resolve), records its
# PID/state into nepi_gazebo_config.yaml, then launches nepi_gazebo.sh (the
# ongoing poll-config/act-on-change/report-status service) in the
# background. nepi_gazebo.sh's own startup already knows how to reattach to
# a GAZEBO_PID it finds alive in the config rather than relaunching -- this
# script is what puts that PID there in the first place.
#
# Safe to re-run: it skips launching whichever of Gazebo / the service is
# already running rather than starting a second copy.

GAZEBO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${GAZEBO_FOLDER}")
SIM_FOLDER=${RESOURCES_FOLDER}/simulation

SIM_CFG_FOLDER=${SIM_FOLDER}/SIM_CFG
SIM_ENV_FOLDER=${SIM_FOLDER}/SIM_ENV
SIM_ROBOT_FOLDER=${SIM_FOLDER}/SIM_ROBOT

GAZEBO_CONFIG_FILE=${SIM_CFG_FOLDER}/nepi_gazebo_config.yaml
GAZEBO_SERVICE=${GAZEBO_FOLDER}/nepi_gazebo.sh

NEPI_UTILS_SOURCE=${RESOURCES_FOLDER}/bash/nepi_gazebo_bash_utils
source "$NEPI_UTILS_SOURCE"

if [[ ! -f "$GAZEBO_CONFIG_FILE" ]]; then
    echo "Gazebo config file not found: ${GAZEBO_CONFIG_FILE}"
    exit 1
fi

if [[ ! -f "$GAZEBO_SERVICE" ]]; then
    echo "Gazebo service not found: ${GAZEBO_SERVICE}"
    exit 1
fi


####################################
# 1. Launch Gazebo, if it isn't already running. nepi_gazebo.sh keeps its
# own copy of this same find-a-world/set-search-paths/launch logic for a
# later config-driven start -- kept inline in both places rather than a
# shared hook script, since it's short and each caller backgrounds it
# differently (this one direct via nohup, the service via its own poll
# loop).

# Same one-file convention as load_config.sh: the one *.world file staged
# in SIM_ENV, picked up by name rather than hardcoded to a specific world.
WORLD_FILE=$(ls ${SIM_ENV_FOLDER}/*.world 2>/dev/null | head -n 1)
if [[ -z "$WORLD_FILE" ]]; then
    echo "No .world file found in ${SIM_ENV_FOLDER} -- stage one before running this script"
    exit 1
fi

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIM_ROBOT_FOLDER}
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${SIM_ENV_FOLDER}

if pgrep -f "gazebo --verbose ${WORLD_FILE}" >/dev/null 2>&1; then
    echo "Gazebo is already running against ${WORLD_FILE}"
else
    echo "Launching Gazebo with world: ${WORLD_FILE}"
    nohup gazebo --verbose "$WORLD_FILE" >/tmp/nepi_gazebo.log 2>&1 &
    gazebo_pid=$!

    sleep 2
    if ! kill -0 "$gazebo_pid" 2>/dev/null; then
        echo "Gazebo exited immediately after launch -- see /tmp/nepi_gazebo.log"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "Gazebo exited immediately after launch" "$GAZEBO_CONFIG_FILE"
        exit 1
    fi

    echo "Gazebo running with PID ${gazebo_pid}"
    update_yaml_value GAZEBO_PID "$gazebo_pid" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_START 1 "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_STOP 0 "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_LAST_ERROR "" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_LAST_UPDATED "$(date +%s)" "$GAZEBO_CONFIG_FILE"
fi


####################################
# 2. Launch the nepi_gazebo management service, if it isn't already running.
# It reads GAZEBO_PID/GAZEBO_STATE back out of the config on startup and
# reattaches to the Gazebo process above rather than relaunching it.

if pgrep -f "bash ${GAZEBO_SERVICE}" >/dev/null 2>&1; then
    echo "nepi_gazebo service is already running"
else
    echo "Starting nepi_gazebo service"
    nohup bash "$GAZEBO_SERVICE" >/tmp/nepi_gazebo_service.log 2>&1 &
    echo "nepi_gazebo service started with PID $!"
fi
