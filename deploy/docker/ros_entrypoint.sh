#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /arts_ws/install/setup.bash ]; then
    source /arts_ws/install/setup.bash
fi

exec "$@"
