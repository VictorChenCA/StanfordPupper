from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import numpy as np
import sys
import os
import time

# Add pupper_llm to path
sys.path.append(os.path.dirname(__file__))

IMAGE_WIDTH = 700

# TODO: Define constants for the state machine behavior
TIMEOUT = 5  # TODO: Set the timeout threshold (in seconds) for determining when a detection is too old
SEARCH_YAW_VEL = 0.4  # TODO: Set the angular velocity (rad/s) for rotating while searching for the target
TRACK_FORWARD_VEL = 0.4  # TODO: Set the forward velocity (m/s) while tracking the target
KP = 7  # TODO: Set the proportional gain for the proportional controller that centers the target

class State(Enum):
    IDLE = 0     # Stay in place, no tracking
    SEARCH = 1   # Rotate to search for target
    TRACK = 2    # Follow the target

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribe to tracking control to enable/disable tracking
        self.tracking_control_subscription = self.create_subscription(
            String,
            '/tracking_control',
            self.tracking_control_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Start in IDLE mode (no tracking until commanded)
        self.state = State.IDLE
        self.tracking_enabled = False

        # Initialize member variables to track detection state
        self.last_detection_pos = None  # Store the last detection in the image so that we choose the closest detection in this frame
        self.target_pos = 0  # Store the target's normalized position in the image (range: -0.5 to 0.5, where 0 is center)
        self.last_detection_time = None  # Store the timestamp of the most recent detection for timeout checking
        
        self.get_logger().info('State Machine Node initialized in IDLE state.')
        self.get_logger().info('Use begin_tracking(object) to enable tracking.')
    
    def tracking_control_callback(self, msg):
        """Handle tracking control commands."""
        command = msg.data
        self.get_logger().info(f'📥 Received tracking control: "{command}"')
        
        if command.startswith("start:"):
            self.tracking_enabled = True
            obj_name = command.split(":", 1)[1]
            self.get_logger().info(f'✅ Tracking enabled for: {obj_name}')
            self.get_logger().info(f'   State transition: {self.state.name} → SEARCH')
            self.state = State.SEARCH  # Start searching for target
        elif command == "stop":
            self.tracking_enabled = False
            self.get_logger().info('⏸️  Tracking disabled - returning to IDLE')
            self.get_logger().info(f'   State transition: {self.state.name} → IDLE')
            self.state = State.IDLE
            # Stop all movement
            cmd = Twist()
            self.command_publisher.publish(cmd)

    def detection_callback(self, msg):
        """
        Process incoming detections to identify and track the most central object.
        - Check if any detections exist in msg.detections
        - Calculate the normalized center position for each detection (x-coordinate / IMAGE_WIDTH - 0.5)
        - Initially, find the detection closest to the image center (smallest absolute normalized position)
        - After initial detection, find the detection closest to the last detection so that Pupper tracks the same person
        - Store the normalized position in self.target_pos
        - Update self.last_detection_time with the current timestamp
        """
        if len(msg.detections) == 0:
            return

        if self.last_detection_pos is None:
            # First detection: find closest to image center (x = IMAGE_WIDTH / 2)
            best_detection = None
            min_distance = float('inf')
            for detection in msg.detections:
                distance = abs(detection.bbox.center.position.x - IMAGE_WIDTH / 2)
                if distance < min_distance:
                    min_distance = distance
                    best_detection = detection
        else:
            # Subsequent detections: find closest to last position (track same object)
            best_detection = None
            min_distance = float('inf')
            for detection in msg.detections:
                distance = abs(detection.bbox.center.position.x - self.last_detection_pos)
                if distance < min_distance:
                    min_distance = distance
                    best_detection = detection

        x_coord = best_detection.bbox.center.position.x
        normalized_x = x_coord / IMAGE_WIDTH - 0.5

        self.last_detection_pos = x_coord
        self.target_pos = normalized_x
        self.last_detection_time = time.time()

    def timer_callback(self):
        """
        Timer callback that manages state transitions and controls robot motion.
        Called periodically (every 0.1 seconds) to update the robot's behavior.
        """
        # State machine logic
        if not self.tracking_enabled:
            # Not tracking - stay idle and DON'T publish
            # This allows Karel commands to control the robot
            self.state = State.IDLE
            return
        
        # Implement state transition logic based on detection timeout
        # Calculate time_since_detection by subtracting self.last_detection_time from current time
        # Convert the time difference from nanoseconds to seconds
        # If time_since_detection > TIMEOUT, transition to State.SEARCH
        # Otherwise, transition to State.TRACK
        
        # If no detection yet, use a large value to trigger search
        if self.last_detection_time is None:
            time_since_detection = float('inf')
        else:
            # Calculate time difference and convert from nanoseconds to seconds
            time_diff = time.time() - self.last_detection_time
            time_since_detection = time_diff
        
        # Track previous state to detect transitions
        previous_state = self.state
        
        if time_since_detection > TIMEOUT:  # Check if detection is too old
            self.state = State.SEARCH
        else:
            self.state = State.TRACK
        
        # Print debug message when state changes
        if previous_state != self.state:
            self.get_logger().info(f'🔄 State transition: {previous_state.name} → {self.state.name}')

        # Execute state behavior
        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.IDLE:
            # Stay still
            yaw_command = 0.0
            forward_vel_command = 0.0
        
        elif self.state == State.SEARCH:
            # Set yaw_command to rotate in the direction where the target was last seen
            # Use SEARCH_YAW_VEL and rotate opposite to the sign of self.target_pos
            # Keep forward_vel_command = 0.0 (don't move forward while searching)
            if self.target_pos is not None:
                yaw_command = -SEARCH_YAW_VEL if self.target_pos > 0 else SEARCH_YAW_VEL
            else:
                yaw_command = SEARCH_YAW_VEL
            forward_vel_command = 0.0
        
        elif self.state == State.TRACK:
            # Set yaw_command using a proportional controller: -self.target_pos * KP
            # This will turn the robot to center the target in the camera view
            # Set forward_vel_command to TRACK_FORWARD_VEL to move toward the target
            yaw_command = -self.target_pos * KP
            forward_vel_command = TRACK_FORWARD_VEL

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
