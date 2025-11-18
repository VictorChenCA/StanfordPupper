#!/usr/bin/env python3
"""
Bucket-Based Voting System for Twitch Chat Commands
Implements democratic command selection where viewers vote for actions.
"""

import asyncio
import logging
import time
from typing import Dict, List, Optional, Set, Tuple
from dataclasses import dataclass, field
from collections import defaultdict

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("bucket_voting")


@dataclass
class CommandBucket:
    """
    Represents a vote bucket for a specific command.

    Each bucket accumulates voting time from multiple users.
    Time decays continuously (1 second per second).
    """
    command: str
    total_time: float = 0.0
    max_time: float = 60.0
    voters: Set[str] = field(default_factory=set)
    last_update: float = field(default_factory=time.time)

    def add_vote(self, username: str, vote_duration: float) -> bool:
        """
        Add a vote to this bucket.

        Args:
            username: User who is voting
            vote_duration: Seconds to add (typically 30)

        Returns:
            True if vote was added, False if user already voted
        """
        if username in self.voters:
            return False

        # Decay first, then add vote
        self.decay()

        # Add vote duration, capped at max_time
        self.total_time = min(self.total_time + vote_duration, self.max_time)
        self.voters.add(username)
        self.last_update = time.time()

        logger.debug(f"Vote added to '{self.command}' by {username}: {self.total_time:.1f}s")
        return True

    def decay(self):
        """Apply time decay based on elapsed time since last update."""
        now = time.time()
        elapsed = now - self.last_update

        if elapsed > 0 and self.total_time > 0:
            self.total_time = max(0, self.total_time - elapsed)
            self.last_update = now

    def clear(self):
        """Clear all votes from this bucket."""
        self.total_time = 0.0
        self.voters.clear()
        self.last_update = time.time()
        logger.debug(f"Bucket '{self.command}' cleared")

    def clear_voters(self):
        """Clear voter list but keep accumulated time (for new cycle)."""
        self.voters.clear()

    def get_priority(self) -> float:
        """Get current priority (total time after decay)."""
        self.decay()
        return self.total_time

    def is_empty(self) -> bool:
        """Check if bucket has any votes."""
        self.decay()
        return self.total_time <= 0.1  # Small epsilon for floating point


class VotingSystem:
    """
    Manages all command buckets and voting logic.

    Features:
    - One vote per user per execution cycle
    - Votes decay over time (1s/second)
    - Maximum bucket time cap (60s)
    - Winner selection based on highest priority
    """

    def __init__(
        self,
        vote_duration: float = 30.0,
        max_bucket_time: float = 60.0,
        countdown_interval: float = 1.0
    ):
        """
        Initialize voting system.

        Args:
            vote_duration: Seconds added per vote (default 30)
            max_bucket_time: Maximum seconds per bucket (default 60)
            countdown_interval: How often to decay buckets (default 1.0s)
        """
        self.vote_duration = vote_duration
        self.max_bucket_time = max_bucket_time
        self.countdown_interval = countdown_interval

        # Command buckets
        self.buckets: Dict[str, CommandBucket] = {}

        # Tracking
        self.total_votes_received = 0
        self.execution_cycles = 0

        # Background task
        self.countdown_task: Optional[asyncio.Task] = None
        self.running = False

        logger.info(f"VotingSystem initialized: {vote_duration}s/vote, {max_bucket_time}s max")

    def _get_or_create_bucket(self, command: str) -> CommandBucket:
        """Get existing bucket or create new one."""
        if command not in self.buckets:
            self.buckets[command] = CommandBucket(
                command=command,
                max_time=self.max_bucket_time
            )
        return self.buckets[command]

    def add_vote(self, username: str, command: str) -> bool:
        """
        Add a vote for a command.

        Args:
            username: User voting
            command: Command to vote for

        Returns:
            True if vote added, False if user already voted this cycle
        """
        bucket = self._get_or_create_bucket(command)
        success = bucket.add_vote(username, self.vote_duration)

        if success:
            self.total_votes_received += 1
            print(
                f"🗳️  VOTE: {username} → '{command}' "
                f"(bucket: {bucket.total_time:.1f}s, voters: {len(bucket.voters)})",
                flush=True
            )
            logger.info(
                f"Vote: {username} → '{command}' "
                f"(bucket: {bucket.total_time:.1f}s, voters: {len(bucket.voters)})"
            )
        else:
            print(f"❌ Duplicate vote rejected: {username} already voted for '{command}'", flush=True)
            logger.debug(f"Duplicate vote rejected: {username} already voted for '{command}'")

        return success

    def get_winner(self) -> Optional[Tuple[str, float]]:
        """
        Get the command with highest priority.

        Returns:
            Tuple of (command, priority) or None if no votes
        """
        # Decay all buckets first
        for bucket in self.buckets.values():
            bucket.decay()

        # Find bucket with highest priority
        max_priority = 0.0
        winner = None

        for command, bucket in self.buckets.items():
            priority = bucket.get_priority()
            if priority > max_priority:
                max_priority = priority
                winner = command

        if winner and max_priority > 0:
            logger.info(f"Winner: '{winner}' with {max_priority:.1f}s priority")
            return (winner, max_priority)

        return None

    def clear_winner(self, command: str):
        """
        Clear the winning bucket after execution.

        Args:
            command: Command that was executed
        """
        if command in self.buckets:
            self.buckets[command].clear()
            self.execution_cycles += 1
            logger.info(f"Cleared winning bucket: '{command}' (cycle #{self.execution_cycles})")

    def start_new_cycle(self):
        """
        Start a new voting cycle.
        Clears voter lists but keeps accumulated time in buckets.
        """
        for bucket in self.buckets.values():
            bucket.clear_voters()
        logger.debug("New voting cycle started - voter lists cleared")

    def get_bucket_status(self) -> Dict[str, float]:
        """
        Get current status of all buckets.

        Returns:
            Dict mapping command -> current time
        """
        status = {}
        for command, bucket in self.buckets.items():
            priority = bucket.get_priority()
            if priority > 0.1:  # Only show non-empty buckets
                status[command] = priority

        return dict(sorted(status.items(), key=lambda x: x[1], reverse=True))

    def get_top_commands(self, n: int = 5) -> List[Tuple[str, float]]:
        """
        Get top N commands by priority.

        Args:
            n: Number of commands to return

        Returns:
            List of (command, priority) tuples, sorted by priority descending
        """
        status = self.get_bucket_status()
        return sorted(status.items(), key=lambda x: x[1], reverse=True)[:n]

    async def start_countdown(self):
        """Start the background countdown timer."""
        if self.running:
            logger.warning("Countdown already running")
            return

        self.running = True
        self.countdown_task = asyncio.create_task(self._countdown_loop())
        logger.info("Countdown timer started")

    async def stop_countdown(self):
        """Stop the background countdown timer."""
        self.running = False
        if self.countdown_task:
            self.countdown_task.cancel()
            try:
                await self.countdown_task
            except asyncio.CancelledError:
                pass
        logger.info("Countdown timer stopped")

    async def _countdown_loop(self):
        """Background loop that decays all buckets."""
        while self.running:
            try:
                await asyncio.sleep(self.countdown_interval)

                # Decay all buckets
                for bucket in self.buckets.values():
                    bucket.decay()

                # Optional: Log status periodically (every 5 seconds)
                if int(time.time()) % 5 == 0:
                    status = self.get_bucket_status()
                    if status:
                        logger.debug(f"Bucket status: {status}")

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in countdown loop: {e}")
                await asyncio.sleep(1.0)

    def get_stats(self) -> Dict:
        """Get voting system statistics."""
        return {
            "total_votes": self.total_votes_received,
            "execution_cycles": self.execution_cycles,
            "active_buckets": len([b for b in self.buckets.values() if not b.is_empty()]),
            "total_buckets": len(self.buckets),
            "bucket_status": self.get_bucket_status()
        }


# Example usage
async def demo():
    """Demonstrate the voting system."""
    voting = VotingSystem(vote_duration=30, max_bucket_time=60)

    await voting.start_countdown()

    # Simulate votes
    voting.add_vote("alice", "dance")
    voting.add_vote("bob", "dance")
    voting.add_vote("charlie", "move_forward")
    voting.add_vote("alice", "dance")  # Duplicate, should be rejected

    print(f"\nBucket status: {voting.get_bucket_status()}")
    print(f"Top commands: {voting.get_top_commands(3)}")

    # Get winner
    winner = voting.get_winner()
    if winner:
        print(f"\nWinner: {winner[0]} ({winner[1]:.1f}s)")
        voting.clear_winner(winner[0])

    # New cycle
    voting.start_new_cycle()
    voting.add_vote("alice", "wiggle")  # Now alice can vote again

    print(f"\nStats: {voting.get_stats()}")

    await asyncio.sleep(3)
    print(f"\nAfter 3s decay: {voting.get_bucket_status()}")

    await voting.stop_countdown()


if __name__ == "__main__":
    asyncio.run(demo())
