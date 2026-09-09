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

# This script is the NEPI Gazebo Simulator Management Service. It runs on
# the VM/host machine and polls nepi_gazebo_config.yaml for start/stop/
# install requests written by the device side (see the per-request-file
# protocol referenced in that file's comments), and reports GAZEBO_STATE,
# GAZEBO_PID and GAZEBO_LAST_ERROR back into the same file so the device
# can observe progress.
#
# Modeled on nepi_docker.sh's poll-config / act-on-change / report-status
# loop, without the root/network/filesystem concerns that script also
# handles -- this service only ever starts or stops one local process.
#
# Assumes nepi_gazebo_bash_utils is already sourced by whatever launches
# this script (same assumption nepi_docker.sh makes of nepi_bash_utils) --
# update_yaml_value is used as an already-exported command, not defined
# here. nepi_gazebo_start.sh, the normal way this service gets started,
# sources it before backgrounding this script.

GAZEBO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)
RESOURCES_FOLDER=$(dirname "${GAZEBO_FOLDER}")
SIM_FOLDER=${RESOURCES_FOLDER}/simulation
SIM_ENV_FOLDER=${SIM_FOLDER}/SIM_ENV
SIM_ROBOT_FOLDER=${SIM_FOLDER}/SIM_ROBOT

GAZEBO_CONFIG_FILE=${SIM_FOLDER}/SIM_CFG/nepi_gazebo_config.yaml
GAZEBO_CONFIG_LOAD_FILE=${SIM_FOLDER}/SIM_CFG/load_config.sh

# INSTALL_SCRIPT remains an unfilled extension point: dropped here rather
# than guessed, since the real install invocation depends on how this VM's
# Gazebo/ROS environment is laid out. There is no equivalent LAUNCH_SCRIPT
# -- launching Gazebo (find the staged .world file, extend
# GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH, run it) is small enough that this
# service and nepi_gazebo_start.sh each keep their own inline copy rather
# than sharing a hook script.
INSTALL_SCRIPT=${SIM_FOLDER}/SIM_ENV/install_gazebo.sh

POLL_SECONDS=1
LAUNCH_SETTLE_SECONDS=1

if [[ ! -f "$GAZEBO_CONFIG_FILE" ]]; then
    echo "Gazebo config file not found: ${GAZEBO_CONFIG_FILE}"
    exit 1
fi

if [[ ! -f "$GAZEBO_CONFIG_LOAD_FILE" ]]; then
    echo "Load script not found: ${GAZEBO_CONFIG_LOAD_FILE}"
    exit 1
fi

if ! command -v update_yaml_value >/dev/null 2>&1; then
    echo "update_yaml_value is not defined -- source nepi_gazebo_bash_utils before running this script"
    exit 1
fi

function run_install(){
    if [[ -f "$INSTALL_SCRIPT" ]]; then
        bash "$INSTALL_SCRIPT"
        return $?
    fi
    echo "No install script found at ${INSTALL_SCRIPT} -- skipping install step"
    return 0
}

function is_gazebo_running(){
    [[ "$gazebo_pid" -ne 0 ]] && kill -0 "$gazebo_pid" 2>/dev/null
}

function clear_gazebo_last_error(){
    # update_yaml_value's yq env() call errors on an empty value ("Value
    # for env variable ... not provided in env()") -- that's a yq quirk
    # inside the shared function, not something callable around, so
    # clear this one field directly instead of going through it.
    yq e -i '.GAZEBO_LAST_ERROR = ""' "$GAZEBO_CONFIG_FILE"
}

echo ""
echo "##########################"
echo "*** STARTING NEPI GAZEBO SERVICE ***"
echo "##########################"
echo ""
echo "Watching config file: ${GAZEBO_CONFIG_FILE}"

source "$GAZEBO_CONFIG_LOAD_FILE" "$GAZEBO_CONFIG_FILE" > /dev/null 2>&1

gazebo_pid=0
if [[ "$GAZEBO_PID" -ne 0 ]] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
    echo "Reattaching to already-running Gazebo process ${GAZEBO_PID}"
    gazebo_pid=$GAZEBO_PID
elif [[ "$GAZEBO_STATE" == "running" || "$GAZEBO_STATE" == "starting" || "$GAZEBO_STATE" == "installing" ]]; then
    echo "Recorded state was '${GAZEBO_STATE}' but no matching process is running -- resetting to idle"
    update_yaml_value GAZEBO_STATE "idle" "$GAZEBO_CONFIG_FILE"
    update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
fi

# -1 guarantees the first loop iteration always reconciles desired vs.
# actual state, regardless of what GAZEBO_LAST_UPDATED already says.
last_acted_on=-1

while true; do
    source "$GAZEBO_CONFIG_LOAD_FILE" "$GAZEBO_CONFIG_FILE" > /dev/null 2>&1

    if [[ "$GAZEBO_LAST_UPDATED" -gt "$last_acted_on" ]]; then
        last_acted_on=$GAZEBO_LAST_UPDATED
        clear_gazebo_last_error

        # Request flags are cleared here, by the service, once the action
        # they name has actually been carried out -- same convention
        # nepi_docker.sh uses for its own request flags (e.g.
        # NEPI_UPDATE_CONFIG, NEPI_EXPAND_FS): do the work (with an
        # "-ing"/GAZEBO_STATE update while it's in flight), then zero the
        # flag once the outcome is known. Left uncleared, a later
        # unrelated request that just bumps GAZEBO_LAST_UPDATED would find
        # the flag still 1 and re-run the same action again.
        if [[ "$GAZEBO_INSTALL" -eq 1 ]]; then
            echo "Install requested -- running install step"
            update_yaml_value GAZEBO_STATE "installing" "$GAZEBO_CONFIG_FILE"
            if run_install; then
                echo "Install step completed"
                update_yaml_value GAZEBO_INSTALL 0 "$GAZEBO_CONFIG_FILE"
            else
                echo "Install step failed"
                update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
                update_yaml_value GAZEBO_LAST_ERROR "install step failed" "$GAZEBO_CONFIG_FILE"
                update_yaml_value GAZEBO_INSTALL 0 "$GAZEBO_CONFIG_FILE"
                sleep "$POLL_SECONDS"
                continue
            fi
        fi

        if [[ "$GAZEBO_STOP" -eq 1 ]]; then
            if is_gazebo_running; then
                echo "Stop requested -- stopping Gazebo (pid ${gazebo_pid})"
                update_yaml_value GAZEBO_STATE "stopping" "$GAZEBO_CONFIG_FILE"
                kill "$gazebo_pid" 2>/dev/null
                wait "$gazebo_pid" 2>/dev/null
                gazebo_pid=0
                update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
                update_yaml_value GAZEBO_STATE "idle" "$GAZEBO_CONFIG_FILE"
            fi
            update_yaml_value GAZEBO_STOP 0 "$GAZEBO_CONFIG_FILE"
        elif [[ "$GAZEBO_START" -eq 1 ]]; then
            if ! is_gazebo_running; then
                # Same one-file convention as load_config.sh: the one
                # *.world file staged in SIM_ENV, picked up by name rather
                # than hardcoded to a specific world.
                WORLD_FILE=$(ls ${SIM_ENV_FOLDER}/*.world 2>/dev/null | head -n 1)
                if [[ -z "$WORLD_FILE" ]]; then
                    echo "Start requested but no .world file found in ${SIM_ENV_FOLDER}"
                    update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
                    update_yaml_value GAZEBO_LAST_ERROR "no .world file found in ${SIM_ENV_FOLDER}" "$GAZEBO_CONFIG_FILE"
                else
                    echo "Start requested -- launching Gazebo"
                    update_yaml_value GAZEBO_STATE "starting" "$GAZEBO_CONFIG_FILE"

                    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SIM_ROBOT_FOLDER}
                    export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${SIM_ENV_FOLDER}
                    gazebo --verbose "$WORLD_FILE" &
                    gazebo_pid=$!
                    update_yaml_value GAZEBO_PID "$gazebo_pid" "$GAZEBO_CONFIG_FILE"

                    sleep "$LAUNCH_SETTLE_SECONDS"
                    if kill -0 "$gazebo_pid" 2>/dev/null; then
                        update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
                    else
                        echo "Gazebo exited immediately after launch"
                        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
                        update_yaml_value GAZEBO_LAST_ERROR "Gazebo exited immediately after launch" "$GAZEBO_CONFIG_FILE"
                        update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
                        gazebo_pid=0
                    fi
                fi
            else
                # Already running (e.g. reattached after a service
                # restart) -- reaffirm state in case the install step
                # above left it on "installing".
                update_yaml_value GAZEBO_STATE "running" "$GAZEBO_CONFIG_FILE"
            fi
            update_yaml_value GAZEBO_START 0 "$GAZEBO_CONFIG_FILE"
        fi
    fi

    # Catch a crash even without a new device request.
    if [[ "$gazebo_pid" -ne 0 ]] && ! kill -0 "$gazebo_pid" 2>/dev/null; then
        echo "Gazebo process ${gazebo_pid} is no longer running"
        update_yaml_value GAZEBO_STATE "failed" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_LAST_ERROR "Gazebo process exited unexpectedly" "$GAZEBO_CONFIG_FILE"
        update_yaml_value GAZEBO_PID 0 "$GAZEBO_CONFIG_FILE"
        gazebo_pid=0
    fi

    update_yaml_value GAZEBO_LAST_POLL "$(date +%s)" "$GAZEBO_CONFIG_FILE"

    sleep "$POLL_SECONDS"
done
