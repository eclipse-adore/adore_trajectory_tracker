#!/usr/bin/env bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS2_WORKSPACE_DIRECTORY="$(realpath "${SCRIPT_DIRECTORY}/../../")"
ros2_package=$(grep -oP '<name>\K[^<]*' "${SCRIPT_DIRECTORY}/package.xml" | tr -d '[:space:]')

(
cd ${ROS2_WORKSPACE_DIRECTORY}
node=${ros2_package}
source "${ROS2_WORKSPACE_DIRECTORY}/install/setup.bash"

if [[ -z $(ros2 pkg list | grep "${ros2_package}") ]]; then
    echo "ERROR: ROS2 Package: '${ros2_package}' not found. Did you build it?" >&2
    exit 1
fi
set -x
ros2 run ${ros2_package} ${node}
)


