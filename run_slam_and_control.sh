#!/bin/bash
# Run SLAM and robot controller together

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

echo "=== NAO SLAM with Robot Control ==="
echo ""
echo "This will:"
echo "  1. Start SLAM in background"
echo "  2. Start robot controller in foreground"
echo ""
echo "In the controller window (NAO Controller):"
echo "  W/S: forward/backward"
echo "  A/D: strafe left/right"
echo "  Q/E: turn left/right"
echo "  SPACE: stop"
echo "  ESC: quit (kills both SLAM and controller)"
echo ""
echo "TIP: Move robot slowly in a well-lit area with lots of texture"
echo "     to help SLAM initialize. Look for green dots in the viewer!"
echo ""
echo "Press any key to start..."
read -n1

# Start SLAM in background
"$SCRIPT_DIR/build/mono_nao" \
    /dev/data/build/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    "$SCRIPT_DIR/NAO_640x480.yaml" &

SLAM_PID=$!
echo "SLAM started (PID: $SLAM_PID)"

# Give SLAM time to load vocabulary
sleep 3

# Start controller (will kill SLAM on exit)
python "$SCRIPT_DIR/slam_controller.py"

# Controller exited - kill SLAM
echo ""
echo "Stopping SLAM..."
kill $SLAM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null
echo "Done!"
