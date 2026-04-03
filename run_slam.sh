#!/bin/bash
# Wrapper script to run NAO SLAM with correct library paths

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$1" in
    mono)
        "$SCRIPT_DIR/build/mono_nao" \
            /dev/data/build/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            "$SCRIPT_DIR/NAO_640x480.yaml"
        ;;
    inertial)
        "$SCRIPT_DIR/build/mono_inertial_nao" \
            /dev/data/build/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            "$SCRIPT_DIR/NAO_640x480.yaml"
        ;;
    control)
        "$SCRIPT_DIR/build/mono_nao_control" \
            /dev/data/build/ORB_SLAM3/Vocabulary/ORBvoc.txt \
            "$SCRIPT_DIR/NAO_640x480.yaml"
        ;;
    *)
        echo "Usage: $0 {mono|inertial|control}"
        echo "  mono     - Monocular SLAM (camera only)"
        echo "  inertial - Monocular-Inertial SLAM (camera + IMU)"
        echo "  control  - Monocular SLAM + WASD robot control"
        exit 1
        ;;
esac
