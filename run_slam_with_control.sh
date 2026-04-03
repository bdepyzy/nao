#!/bin/bash
# Start SLAM in background, then controller in foreground

SCRIPT_DIR="/dev/data/build/nao_agents"
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

echo "=== Starting SLAM + Controller ==="
echo ""
echo "1. Starting SLAM in background..."

# Start SLAM in background, redirect output
"$SCRIPT_DIR/build/mono_nao" \
    /dev/data/build/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    "$SCRIPT_DIR/NAO_640x480.yaml" > /tmp/slam.log 2>&1 &

SLAM_PID=$!
echo "   SLAM PID: $SLAM_PID"

# Wait for vocabulary to load
echo "2. Waiting for SLAM to load vocabulary..."
while grep -q "Loading ORB Vocabulary" /tmp/slam.log 2>/dev/null; do
    sleep 1
done

# Wait a bit more for viewer to start
sleep 3

echo ""
echo "3. Starting controller..."
echo ""
echo "=== CONTROLS ==="
echo "  W/S: forward/backward"
echo "  A/D: strafe left/right"
echo "  Q/E: turn left/right"
echo "  ESC: quit (also stops SLAM)"
echo ""
echo "Look for these windows:"
echo "  - 'Fast NAO Controller' - robot view + controls"
echo "  - 'ORB-SLAM3: Map Viewer' - 3D SLAM map"
echo ""
echo "Move robot around to build the map!"
echo ""

# Start controller (this will block)
python2 "$SCRIPT_DIR/controller.py"

# Controller exited - cleanup
echo ""
echo "Stopping SLAM (PID: $SLAM_PID)..."
kill $SLAM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null

echo "Done! Trajectory saved to KeyFrameTrajectory.txt"
