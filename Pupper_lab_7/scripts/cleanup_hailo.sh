#!/bin/bash
# Cleanup script for Hailo device lock issues

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is one level up from scripts directory
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

echo "🧹 Cleaning up Hailo and Lab 7 processes..."
echo ""

# Kill all related processes
echo "Stopping processes..."
pkill -9 -f "hailo_detection.py" 2>/dev/null && echo "  ✓ Killed hailo_detection.py" || echo "  - hailo_detection.py not running"
pkill -9 -f "lab_7.py" 2>/dev/null && echo "  ✓ Killed lab_7.py" || echo "  - lab_7.py not running"
pkill -9 -f "test_tracking.py" 2>/dev/null && echo "  ✓ Killed test_tracking.py" || echo "  - test_tracking.py not running"
pkill -9 -f "realtime_voice.py" 2>/dev/null && echo "  ✓ Killed realtime_voice.py" || echo "  - realtime_voice.py not running"
pkill -9 -f "karel_realtime_commander.py" 2>/dev/null && echo "  ✓ Killed karel_realtime_commander.py" || echo "  - karel_realtime_commander.py not running"
pkill -f "lab_7.launch.py" 2>/dev/null && echo "  ✓ Killed lab_7.launch.py" || echo "  - lab_7.launch.py not running"
pkill -f "foxglove_bridge" 2>/dev/null && echo "  ✓ Killed foxglove_bridge" || echo "  - foxglove_bridge not running"

echo ""
echo "Waiting for processes to terminate..."
sleep 2

# Check for remaining python processes
REMAINING=$(pgrep -f "hailo_detection\|lab_7\.py" | wc -l)
if [ $REMAINING -gt 0 ]; then
    echo "⚠️  Warning: $REMAINING process(es) still running"
    echo "   Use 'pkill -9 -f hailo_detection' to force kill"
else
    echo "✅ All processes terminated"
fi

echo ""
echo "✅ Cleanup complete!"
echo ""
echo "You can now run:"
echo "  $SCRIPT_DIR/run_tracking.sh"
echo "  or"
echo "  $SCRIPT_DIR/run_full_system.sh"
echo ""


