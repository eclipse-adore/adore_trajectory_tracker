#!/usr/bin/env bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS2_WORKSPACE_DIRECTORY="$(realpath "${SCRIPT_DIRECTORY}/../../")"
ros2_package=$(grep -oP '<name>\K[^<]*' "${SCRIPT_DIRECTORY}/package.xml" | tr -d '[:space:]')

(
set -x
cd ${ROS2_WORKSPACE_DIRECTORY}
colcon build --packages-select ${ros2_package}
)


