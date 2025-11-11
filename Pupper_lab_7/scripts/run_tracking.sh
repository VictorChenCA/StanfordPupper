#!/bin/bash
# Launch script for Interactive Tracking Test
# Starts only the components needed for keyboard-controlled tracking test

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is one level up from scripts directory
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Change to project directory
cd "$PROJECT_ROOT"

# Cleanup function - runs on exit or Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Interrupt received. Running cleanup..."
    
    # Kill background processes if they exist
    [ ! -z "$STATE_PID" ] && kill $STATE_PID 2>/dev/null
    [ ! -z "$HAILO_PID" ] && kill $HAILO_PID 2>/dev/null
    [ ! -z "$FOX_PID" ] && kill $FOX_PID 2>/dev/null
    [ ! -z "$ROS_PID" ] && kill $ROS_PID 2>/dev/null
    
    sleep 1
    
    # Run comprehensive cleanup script
    echo "Running comprehensive cleanup..."
    bash "$SCRIPT_DIR/cleanup_hailo.sh"
    
    echo "✅ Cleanup complete"
    exit 0
}

# Trap Ctrl+C (SIGINT) and SIGTERM
trap cleanup INT TERM

echo "========================================="
echo "🎯 Interactive Tracking Test Launcher"
echo "========================================="
echo ""
echo "This will launch:"
echo "  1. ROS2 Control + Camera"
echo "  2. Foxglove Bridge"
echo "  3. Hailo Object Detection"
echo "  4. Tracking State Machine"
echo "  5. Interactive Test Script"
echo ""
echo "No voice or OpenAI needed - just tracking!"
echo ""
echo "💡 Press Ctrl+C anytime to cleanup and exit"
echo ""

# Initial cleanup
echo "🧹 Cleaning up existing processes..."
pkill -f "lab_7.launch.py" 2>/dev/null || true
pkill -f "foxglove_bridge" 2>/dev/null || true
pkill -f "hailo_detection.py" 2>/dev/null || true
pkill -f "lab_7.py" 2>/dev/null || true
pkill -f "test_tracking.py" 2>/dev/null || true
sleep 1

# Start ROS2 launch file (control + camera) directly
echo "1️⃣  Launching ROS2 Control + Camera..."
ros2 launch "$PROJECT_ROOT/lab_7.launch.py" > /tmp/ros2_test.log 2>&1 &
ROS_PID=$!
echo "   PID: $ROS_PID | Log: /tmp/ros2_test.log"
sleep 5

# Start Foxglove bridge
echo "2️⃣  Launching Foxglove Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /tmp/foxglove_test.log 2>&1 &
FOX_PID=$!
echo "   PID: $FOX_PID | Log: /tmp/foxglove_test.log"
sleep 2

# Start Hailo detection node
echo "3️⃣  Launching Hailo Detection..."
python "$PROJECT_ROOT/hailo_detection.py" > /tmp/hailo_test.log 2>&1 &
HAILO_PID=$!
echo "   PID: $HAILO_PID | Log: /tmp/hailo_test.log"
sleep 4

# Start tracking state machine
echo "4️⃣  Launching Tracking State Machine..."
python "$PROJECT_ROOT/lab_7.py" > /tmp/tracking_test.log 2>&1 &
STATE_PID=$!
echo "   PID: $STATE_PID | Log: /tmp/tracking_test.log"
sleep 2

echo ""
echo "========================================="
echo "✅ Background components ready!"
echo "========================================="
echo ""
echo "Running components:"
echo "  • ROS2 Control + Camera (PID: $ROS_PID)"
echo "  • Foxglove Bridge (PID: $FOX_PID)"
echo "  • Hailo Detection (PID: $HAILO_PID)"
echo "  • Tracking State Machine (PID: $STATE_PID)"
echo ""
echo "📊 Logs:"
echo "  • ROS2: /tmp/ros2_test.log"
echo "  • Foxglove: /tmp/foxglove_test.log"
echo "  • Hailo: /tmp/hailo_test.log"
echo "  • Tracking: /tmp/tracking_test.log"
echo ""
echo "💡 Tip: In another terminal, run 'tail -f /tmp/tracking_test.log' to see live tracking logs"
echo ""
echo "🎮 Starting Interactive Test in 2 seconds..."
sleep 2
echo ""

# Start interactive test in foreground
python "$PROJECT_ROOT/test_tracking.py"

# When test exits normally, run cleanup
cleanup


