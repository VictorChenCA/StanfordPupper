#!/usr/bin/env python3
"""
Karel Realtime Commander
Simplified commander for use with OpenAI Realtime API.
Focuses on command extraction and robot control since voice/LLM is handled by Realtime API.
"""

import asyncio
import re
import logging
from typing import Optional, Tuple
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import karel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("karel_realtime_commander")


class KarelRealtimeCommanderNode(Node):
    """Simplified commander that extracts and executes commands from Realtime API responses."""
    
    def __init__(self):
        super().__init__('karel_realtime_commander_node')
        
        # Subscribe to responses from Realtime API
        self.response_subscription = self.create_subscription(
            String,
            'gpt4_response_topic',
            self.response_callback,
            10
        )
        
        # Also subscribe to transcription for logging
        self.transcription_subscription = self.create_subscription(
            String,
            '/transcription',
            self.transcription_callback,
            10
        )
        
        # Initialize robot
        self.pupper = karel.KarelPupper()
        
        # Command queue with timestamps
        self.command_queue = asyncio.Queue()
        self.processing_commands = False
        self.command_timeout = 20.0  # Clear commands older than 20 seconds
        
        logger.info('Karel Realtime Commander initialized')
        logger.info('Listening for commands from Realtime API...')
    
    def transcription_callback(self, msg):
        """Log user transcriptions."""
        logger.debug(f"👤 User: {msg.data}")
    
    def response_callback(self, msg):
        """Process responses and extract commands line by line."""
        response = msg.data
        logger.info(f"🤖 Response: {response}")
        all_commands = []
        
        # TODO: Parse commands from the response text line by line and dispatch them in order into the `all_commands` list.
        # 1. Split the `response` string into lines using `\n` as a separator.
        # 2. For each line that is not blank, call `self.extract_commands_from_line(line.strip())` to get a list of commands from that line.
        # 3. Collect all commands, preserving the original order.
        # 4. Append each command to `all_commands` (should be a flat list, not nested).
        # 5. This ensures that multi-line responses generate a sequence of actions in the same order as the LLM output.
        # Example:
        #   If response is:
        #     "Move forward\nTurn left\nBark"
        #   Your code should process:
        #     ["move", "turn_left", "bark"]

        # Your code here:
        pass

        
        if all_commands:
            logger.info(f"📋 Commands (in order): {all_commands}")
            # Queue commands with timestamp in sequential order
            current_time = time.time()
            for cmd in all_commands:
                command_with_time = (cmd, current_time)
                asyncio.create_task(self.command_queue.put(command_with_time))
        else:
            logger.debug("No commands found")
    
    def extract_commands_from_line(self, line: str) -> list:
        """
        TODO: Implement this function to extract robot commands from a single line of text.

        HINTS:
        - The parsing logic will depend on exactly how you format your GPT model/system prompt!
        - The commands you define here will be used to execute the commands in the execute_command function, which you'll also implement below.
        - Think carefully about the sequential structure of the LLM's output and how your system prompt tells the model to format commands.
        - For example, if your prompt instructs GPT to output one command per line, you should parse only a single command from each line.
        - You may need to match/substitute multiple possible phrasings (e.g. "move forward", "walk forward") to a canonical command like "move".
        - Construct and return a list of action strings (e.g. ['move', 'turn_left']) extracted from the line.
        - Test your command extraction logic carefully, since if the output is not sequential, your robot may behave out of order!

        Example:
            line = "Move forward"
            returns ['move']

            line = "<move, turn_left>"
            returns ['move', 'turn_left']
        """
        pass
    
    async def execute_command(self, command: str) -> bool:
        """Execute a single robot command."""
        try:
            logger.info(f"⚙️  Executing {command}")
            
            # Movement commands
            if command in ["move", "go", "forward", "move_forward"]:
                self.pupper.move_forward()
                await asyncio.sleep(0.5)
            
            elif command in ["back", "backward", "move_backward", "reverse"]:
                self.pupper.move_backward()
                await asyncio.sleep(0.5)
            
            elif command in ["left", "move_left", "strafe_left"]:
                self.pupper.move_left()
                await asyncio.sleep(0.5)
            
            elif command in ["right", "move_right", "strafe_right"]:
                self.pupper.move_right()
                await asyncio.sleep(0.5)
            
            # Turning commands
            elif command in ["turn_left", "rotate_left", "spin_left"]:
                self.pupper.turn_left()
                await asyncio.sleep(0.5)
            
            elif command in ["turn_right", "rotate_right", "spin_right"]:
                self.pupper.turn_right()
                await asyncio.sleep(0.5)
            
            # Animation commands
            elif command in ["wiggle", "wag", "shake"]:
                self.pupper.wiggle()
                await asyncio.sleep(5.5)
            
            elif command in ["bob", "bounce"]:
                self.pupper.bob()
                await asyncio.sleep(5.5)
            
            elif command in ["dance", "rick_roll", "rickroll"]:
                self.pupper.dance()
                await asyncio.sleep(12.0)
            
            # Sound commands
            elif command in ["bark", "woof", "speak"]:
                self.pupper.bark()
                await asyncio.sleep(0.5)
            
            # Control commands
            elif command in ["stop", "halt", "wait"]:
                self.pupper.stop()
                await asyncio.sleep(0.5)
            
            # Tracking commands (Lab 7)
            elif command in ["begin_tracking", "start_tracking", "track"]:
                self.pupper.begin_tracking("person")
                await asyncio.sleep(0.5)
            
            elif command in ["end_tracking", "stop_tracking", "untrack"]:
                self.pupper.end_tracking()
                await asyncio.sleep(0.5)
            
            else:
                logger.warning(f"⚠️  Unknown command: {command}")
                return False
            
            logger.info(f"✅ Done")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error: {e}")
            return False
    
    async def command_processor_loop(self):
        """Process commands from the queue with timeout checking."""
        logger.info("🔄 Command processor started")
        
        while rclpy.ok():
            try:
                # Get next command with timestamp (wait up to 0.1s)
                command_data = await asyncio.wait_for(
                    self.command_queue.get(),
                    timeout=0.1
                )
                
                # Unpack command and timestamp
                command, timestamp = command_data
                
                # Check if command is stale (older than 20 seconds)
                age = time.time() - timestamp
                if age > self.command_timeout:
                    logger.warning(f"⏰ Discarding stale command '{command}' (age: {age:.1f}s)")
                    continue
                
                # Execute command
                await self.execute_command(command)
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Error in command processor: {e}")
                await asyncio.sleep(0.1)
    
    async def run(self):
        """Main run loop."""
        await self.command_processor_loop()


async def main_async(args=None):
    """Async main function."""
    rclpy.init(args=args)
    
    node = KarelRealtimeCommanderNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        logger.info("🚀 Karel Realtime Commander started")
        logger.info("Ready to receive commands from Realtime API")
        
        # Create tasks
        ros_task = asyncio.create_task(spin_ros_async(executor))
        command_task = asyncio.create_task(node.run())
        
        await asyncio.gather(ros_task, command_task)
        
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        executor.shutdown()
        rclpy.shutdown()


async def spin_ros_async(executor):
    """Spin ROS2 executor in async-friendly way."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)


def main(args=None):
    """Entry point."""
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Program interrupted")


if __name__ == '__main__':
    main()
