#!/bin/bash
# Launch script for Lab 7 + Twitch Integration
# Starts full Pupper system plus Twitch chat control

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is one level up from scripts directory
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
# Twitch directory
TWITCH_DIR="$( cd "$PROJECT_ROOT/../twitch" && pwd )"

# Change to project directory
cd "$PROJECT_ROOT"

# Cleanup function - runs on exit or Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Shutting down Lab 7 + Twitch system..."

    # Kill background processes if they exist
    [ ! -z "$TWITCH_PID" ] && kill $TWITCH_PID 2>/dev/null
    [ ! -z "$COMMANDER_PID" ] && kill $COMMANDER_PID 2>/dev/null
    [ ! -z "$VOICE_PID" ] && kill $VOICE_PID 2>/dev/null
    [ ! -z "$STATE_PID" ] && kill $STATE_PID 2>/dev/null
    [ ! -z "$HAILO_PID" ] && kill $HAILO_PID 2>/dev/null
    [ ! -z "$FOX_PID" ] && kill $FOX_PID 2>/dev/null
    [ ! -z "$ROS_PID" ] && kill $ROS_PID 2>/dev/null

    sleep 1

    # Run comprehensive cleanup
    echo "Running comprehensive cleanup..."
    bash "$SCRIPT_DIR/cleanup_hailo.sh"

    echo "✅ All processes stopped"
    exit 0
}

# Trap Ctrl+C (SIGINT) and SIGTERM
trap cleanup INT TERM

echo "========================================="
echo "🐕🎮 Pupper + Twitch Streaming Mode"
echo "========================================="
echo ""
echo "💡 Press Ctrl+C anytime to cleanup and exit"
echo ""

# Check Twitch setup first
echo "🔍 Checking Twitch integration setup..."
if [ -f "$TWITCH_DIR/check_setup.py" ]; then
    python3 "$TWITCH_DIR/check_setup.py"
    if [ $? -ne 0 ]; then
        echo ""
        echo "❌ Twitch setup incomplete!"
        echo "Please fix the errors above before launching."
        echo ""
        exit 1
    fi
else
    echo "⚠️  Warning: check_setup.py not found, skipping verification"
fi

echo ""
echo "✅ Twitch setup verified!"
echo ""

# Add project directory to Python path
export PYTHONPATH="${PYTHONPATH}:$PROJECT_ROOT"

echo ""
echo "Starting Lab 7 + Twitch components..."
echo "========================================"
echo ""

# Kill any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -f "message_grabber.py" 2>/dev/null || true
pkill -f "hailo_detection.py" 2>/dev/null || true
pkill -f "lab_7.py" 2>/dev/null || true
pkill -f "realtime_voice.py" 2>/dev/null || true
pkill -f "karel_realtime_commander.py" 2>/dev/null || true
sleep 1

# Start ROS2 launch file (control + camera) - silenced
echo "1️⃣  Launching ROS2 system (control + camera)..."
ros2 launch "$PROJECT_ROOT/lab_7.launch.py" > /dev/null 2>&1 &
ROS_PID=$!
sleep 3

# Start Foxglove bridge - silenced
echo "2️⃣  Launching Foxglove Bridge..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /dev/null 2>&1 &
FOX_PID=$!
sleep 2

# Start Hailo detection node - silenced
echo "3️⃣  Launching Hailo Detection (with tracking support)..."
python "$PROJECT_ROOT/hailo_detection.py" > /dev/null 2>&1 &
HAILO_PID=$!
sleep 3

# Start tracking state machine - silenced
echo "4️⃣  Launching Tracking State Machine..."
python "$PROJECT_ROOT/lab_7.py" > /dev/null 2>&1 &
STATE_PID=$!
sleep 2

# Start OpenAI Realtime Voice - VISIBLE
echo "5️⃣  Launching OpenAI Realtime Voice (with vision)..."
python -u "$PROJECT_ROOT/pupper_llm/realtime_voice.py" 2>&1 | sed 's/^/[VOICE] /' &
VOICE_PID=$!
sleep 3

# Start Karel Command Parser - VISIBLE
echo "6️⃣  Launching Karel Command Parser..."
python -u "$PROJECT_ROOT/pupper_llm/karel/karel_realtime_commander.py" 2>&1 | sed 's/^/[KAREL] /' &
COMMANDER_PID=$!
sleep 2

# Start Twitch Integration - VISIBLE
echo "7️⃣  Launching Twitch Chat Integration..."
cd "$TWITCH_DIR"
python -u message_grabber.py 2>&1 | sed 's/^/[TWITCH] /' &
TWITCH_PID=$!
cd "$PROJECT_ROOT"

echo ""
echo "========================================="
echo "✅ All components launched!"
echo "========================================="
echo ""
echo "Running processes:"
echo "  • ROS2 Control + Camera (PID: $ROS_PID) - silenced"
echo "  • Foxglove Bridge (PID: $FOX_PID) - silenced"
echo "  • Hailo Detection (PID: $HAILO_PID) - silenced"
echo "  • Tracking State Machine (PID: $STATE_PID) - silenced"
echo "  • Realtime Voice (PID: $VOICE_PID) - 🔊 VISIBLE"
echo "  • Command Parser (PID: $COMMANDER_PID) - 🔊 VISIBLE"
echo "  • Twitch Chat Bot (PID: $TWITCH_PID) - 🔊 VISIBLE"
echo ""
echo "🎤 Voice Control Ready!"
echo "📷 Vision System Active!"
echo "🎯 Tracking System Online!"
echo "🎮 Twitch Chat Control Active!"
echo ""
echo "📺 Log Display:"
echo "  [VOICE]  = OpenAI voice transcriptions & responses"
echo "  [KAREL]  = Robot command execution & actions"
echo "  [TWITCH] = Twitch chat messages & voting system"
echo ""
echo "Try these commands:"
echo ""
echo "Voice commands:"
echo "  • 'Track that person'"
echo "  • 'Follow the dog'"
echo "  • 'What do you see?'"
echo "  • 'Move forward'"
echo "  • 'Dance!'"
echo ""
echo "Twitch chat (viewers type in your stream):"
echo "  • !pupper dance"
echo "  • !pupper move forward"
echo "  • !pupper follow that person"
echo "  • !pupper wiggle"
echo ""
echo "🗳️  Voting System Active:"
echo "  • Viewers vote for commands with !pupper"
echo "  • Each vote adds 30 seconds to that command's bucket"
echo "  • Most popular command executes when robot is ready"
echo "  • Votes decay 1s per second (keeps it dynamic)"
echo ""
echo "🌐 Foxglove: ws://localhost:8765"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo "========================================="
echo ""

# Wait for any process to exit
wait
