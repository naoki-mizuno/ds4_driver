#!/bin/bash

DS4_DRIVER_LOCAL_PATH="${1}"

# Override with local copy of ds4_driver (for development)
if [[ -n $DS4_DRIVER_LOCAL_PATH ]]; then
    _ws_prefix=/opt/underlay_ws
    docker run -it \
        -v "/dev:/dev" \
        --privileged \
        -v "$( realpath $DS4_DRIVER_LOCAL_PATH ):$_ws_prefix/src/ds4_driver" \
        naomiz/ds4_driver:foxy \
        bash -c "colcon build && bash -i"
else
    docker run -it \
        -v "/dev:/dev" \
        --privileged \
        naomiz/ds4_driver:foxy \
        bash
fi
