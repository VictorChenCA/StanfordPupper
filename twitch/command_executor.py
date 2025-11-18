#!/usr/bin/env python3
"""
Command Execution Coordinator for Bucket Voting System
Waits for robot to finish executing, then selects and executes next command.
"""

import asyncio
import logging
import time
from typing import Optional, Callable
from enum import Enum

from bucket_voting import VotingSystem

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("command_executor")


class ExecutionState(Enum):
    """Robot execution states."""
    IDLE = "idle"
    EXECUTING = "executing"
    WAITING = "waiting"


class CommandExecutor:
    """
    Coordinates command execution with voting system.

    Flow:
    1. Wait for robot to be idle
    2. Query VotingSystem for winner
    3. Publish winner to ROS2
    4. Wait for command to complete
    5. Clear winning bucket and start new cycle
    6. Repeat
    """

    # Command execution durations (in seconds)
    COMMAND_DURATIONS = {
        "move_forward": 0.5,
        "move_backward": 0.5,
        "move_left": 0.5,
        "move_right": 0.5,
        "turn_left": 0.5,
        "turn_right": 0.5,
        "wiggle": 5.5,
        "bob": 5.5,
        "dance": 12.0,
        "bark": 1.0,
        "stop_tracking": 0.5,
    }

    def __init__(
        self,
        voting_system: VotingSystem,
        ros_publisher=None,
        execution_interval: float = 0.5
    ):
        """
        Initialize command executor.

        Args:
            voting_system: VotingSystem instance to query for winners
            ros_publisher: ROS2 publisher (PupperCommandPublisher)
            execution_interval: How often to check for new commands (seconds)
        """
        self.voting_system = voting_system
        self.ros_publisher = ros_publisher
        self.execution_interval = execution_interval

        # State tracking
        self.state = ExecutionState.IDLE
        self.current_command: Optional[str] = None
        self.command_start_time: float = 0.0
        self.expected_duration: float = 0.0

        # Statistics
        self.commands_executed = 0
        self.total_execution_time = 0.0

        # Control
        self.running = False
        self.executor_task: Optional[asyncio.Task] = None

        logger.info("CommandExecutor initialized")

    def _get_command_duration(self, command: str) -> float:
        """
        Get expected duration for a command.

        Args:
            command: Command name (e.g., "dance", "move_forward")

        Returns:
            Duration in seconds
        """
        # Handle tracking commands
        if command.startswith("start_tracking") or command.startswith("track_"):
            return 0.5

        # Look up in duration table
        return self.COMMAND_DURATIONS.get(command, 1.0)  # Default 1s

    def is_idle(self) -> bool:
        """Check if executor is idle and ready for next command."""
        if self.state == ExecutionState.IDLE:
            return True

        # Check if current command should be done
        if self.state == ExecutionState.EXECUTING:
            elapsed = time.time() - self.command_start_time
            if elapsed >= self.expected_duration:
                self._finish_command()
                return True

        return False

    def _finish_command(self):
        """Mark current command as finished."""
        if self.current_command:
            elapsed = time.time() - self.command_start_time
            logger.info(
                f"Command '{self.current_command}' completed "
                f"(expected: {self.expected_duration:.1f}s, actual: {elapsed:.1f}s)"
            )

            # Clear the winning bucket
            self.voting_system.clear_winner(self.current_command)

            # Start new voting cycle
            self.voting_system.start_new_cycle()

            # Update stats
            self.commands_executed += 1
            self.total_execution_time += elapsed

        self.state = ExecutionState.IDLE
        self.current_command = None
        self.command_start_time = 0.0
        self.expected_duration = 0.0

    def execute_winner(self) -> bool:
        """
        Query voting system for winner and execute it.

        Returns:
            True if command was executed, False if no winner
        """
        if not self.is_idle():
            logger.debug("Cannot execute - robot is busy")
            return False

        # Get winner from voting system
        winner = self.voting_system.get_winner()

        if not winner:
            logger.debug("No winner - all buckets empty")
            return False

        command, priority = winner
        print(f"🏆 EXECUTING WINNER: '{command}' (priority: {priority:.1f}s)", flush=True)
        logger.info(f"Executing winner: '{command}' (priority: {priority:.1f}s)")

        # Publish to ROS2 if available
        if self.ros_publisher:
            print(f"🤖 Publishing to ROS2: {command}", flush=True)
            self.ros_publisher.publish_command(command)
        else:
            print(f"⚠️  No ROS2 publisher - would execute: {command}", flush=True)
            logger.warning(f"No ROS2 publisher - would execute: {command}")

        # Update state
        self.state = ExecutionState.EXECUTING
        self.current_command = command
        self.command_start_time = time.time()
        self.expected_duration = self._get_command_duration(command)

        print(f"⏱️  Robot will be busy for ~{self.expected_duration:.1f}s", flush=True)
        logger.info(f"Robot will be busy for ~{self.expected_duration:.1f}s")
        return True

    async def start(self):
        """Start the execution coordinator loop."""
        if self.running:
            logger.warning("Executor already running")
            return

        self.running = True
        self.executor_task = asyncio.create_task(self._execution_loop())
        logger.info("CommandExecutor started")

    async def stop(self):
        """Stop the execution coordinator loop."""
        self.running = False
        if self.executor_task:
            self.executor_task.cancel()
            try:
                await self.executor_task
            except asyncio.CancelledError:
                pass

        logger.info(
            f"CommandExecutor stopped "
            f"(executed: {self.commands_executed}, "
            f"total time: {self.total_execution_time:.1f}s)"
        )

    async def _execution_loop(self):
        """Main loop - checks for winners and executes them."""
        while self.running:
            try:
                # Check if we're idle and can execute
                if self.is_idle():
                    # Try to execute winner
                    executed = self.execute_winner()

                    if not executed:
                        # No winner, wait a bit before checking again
                        await asyncio.sleep(self.execution_interval)
                else:
                    # Currently executing, check periodically
                    await asyncio.sleep(self.execution_interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in execution loop: {e}")
                await asyncio.sleep(1.0)

    def get_status(self) -> dict:
        """Get current executor status."""
        status = {
            "state": self.state.value,
            "current_command": self.current_command,
            "commands_executed": self.commands_executed,
            "total_execution_time": self.total_execution_time
        }

        if self.state == ExecutionState.EXECUTING:
            elapsed = time.time() - self.command_start_time
            remaining = max(0, self.expected_duration - elapsed)
            status["elapsed"] = elapsed
            status["remaining"] = remaining

        return status


# Example usage
async def demo():
    """Demonstrate the command executor."""
    from bucket_voting import VotingSystem

    # Create voting system
    voting = VotingSystem(vote_duration=30, max_bucket_time=60)
    await voting.start_countdown()

    # Create executor (no ROS2 for demo)
    executor = CommandExecutor(voting_system=voting, ros_publisher=None)

    # Add some votes
    voting.add_vote("alice", "dance")
    voting.add_vote("bob", "dance")
    voting.add_vote("charlie", "dance")
    voting.add_vote("dave", "move_forward")

    print(f"\nBucket status: {voting.get_bucket_status()}")

    # Start executor
    await executor.start()

    # Let it run for a bit
    print("\nExecutor running...")
    for i in range(20):
        await asyncio.sleep(1)
        if i % 3 == 0:
            status = executor.get_status()
            print(f"Status: {status}")

        # Add more votes mid-execution
        if i == 5:
            voting.add_vote("eve", "wiggle")
            voting.add_vote("frank", "wiggle")
            print("\nNew votes added!")

    # Stop
    await executor.stop()
    await voting.stop_countdown()

    print(f"\nFinal stats: {executor.get_status()}")
    print(f"Voting stats: {voting.get_stats()}")


if __name__ == "__main__":
    asyncio.run(demo())
