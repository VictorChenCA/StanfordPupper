#!/usr/bin/env python3
"""
Twitch Chat Processor with LLM Integration
Batches messages and uses OpenAI Chat API to extract Pupper commands.
Optimized for Raspberry Pi with rate limiting and message filtering.
"""

import asyncio
import logging
import time
from typing import Optional, List, Dict
from dataclasses import dataclass
from datetime import datetime
import os

try:
    from openai import AsyncOpenAI
except ImportError:
    print("Warning: openai package not installed. Run: pip install openai")
    AsyncOpenAI = None

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("chat_processor")


@dataclass
class ChatMessage:
    """Represents a Twitch chat message."""
    username: str
    text: str
    timestamp: float
    broadcaster: str


class ChatProcessor:
    """
    Processes Twitch chat messages using OpenAI Chat API.
    Features:
    - Message batching to reduce API calls
    - Command prefix filtering
    - Rate limiting
    - Pattern matching fallback for common commands
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        command_prefix: str = "!pupper",
        batch_interval: float = 3.0,
        max_requests_per_minute: int = 12,
        model: str = "gpt-4o-mini",
        voting_system = None
    ):
        """
        Initialize the chat processor.

        Args:
            api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
            command_prefix: Prefix for commands (e.g., "!pupper")
            batch_interval: Seconds to wait before processing batch
            max_requests_per_minute: Rate limit for API calls
            model: OpenAI model to use (gpt-4o-mini is cheaper and faster)
            voting_system: VotingSystem instance for bucket voting (optional)
        """
        self.command_prefix = command_prefix.lower()
        self.batch_interval = batch_interval
        self.max_requests_per_minute = max_requests_per_minute
        self.model = model
        self.voting_system = voting_system

        # Initialize OpenAI client
        if AsyncOpenAI is None:
            logger.error("OpenAI package not installed. Install with: pip install openai")
            self.client = None
        else:
            api_key = api_key or os.getenv("OPENAI_API_KEY")
            if not api_key:
                logger.warning("No OpenAI API key provided. Set OPENAI_API_KEY environment variable.")
                self.client = None
            else:
                self.client = AsyncOpenAI(api_key=api_key)

        # Message queue and processing
        self.message_queue: List[ChatMessage] = []
        self.processing_task: Optional[asyncio.Task] = None
        self.running = False

        # Rate limiting
        self.request_timestamps: List[float] = []
        self.api_calls_made = 0
        self.commands_processed = 0

        # Pattern matching for common commands (no LLM needed)
        self.simple_patterns = {
            "sit": "sit",
            "stand": "stand",
            "walk": "move_forward",
            "move forward": "move_forward",
            "move backward": "move_backward",
            "move back": "move_backward",
            "turn left": "turn_left",
            "turn right": "turn_right",
            "move left": "move_left",
            "move right": "move_right",
            "dance": "dance",
            "wiggle": "wiggle",
            "bob": "bob",
            "bark": "bark",
            "stop": "stop_tracking",
            "stop tracking": "stop_tracking",
            "stop following": "stop_tracking",
        }

        # System prompt for LLM
        self.system_prompt = """You are a command parser for the Stanford Pupper robot, controlled via Twitch chat.

Your job is to extract robot commands from natural language messages. Return ONLY the command keywords, one per line, no explanations.

Available commands:
- Movement: move_forward, move_backward, move_left, move_right, turn_left, turn_right
- Tracking: start_tracking [object] (e.g., "start_tracking [person]", "start_tracking [dog]"), stop_tracking
- Behaviors: dance, wiggle, bob, bark

Examples:
Input: "walk forward then turn left"
Output:
move_forward
turn_left

Input: "follow that person"
Output:
start_tracking [person]

Input: "do a little dance!"
Output:
dance

Input: "what do you see?"
Output:
(return nothing - this is a query, not a command)

Input: "can you see the cat? follow it"
Output:
start_tracking [cat]

IMPORTANT:
- Only return valid command keywords
- For tracking commands, use the format: start_tracking [object_name]
- If the message is not a command (just chat/question), return nothing
- Keep responses minimal - just the commands, nothing else
- Multiple commands should be on separate lines in the order they should execute
"""

    async def start(self):
        """Start the message processor loop."""
        if self.running:
            logger.warning("Chat processor already running")
            return

        if self.client is None:
            logger.error("Cannot start: OpenAI client not initialized")
            return

        self.running = True
        self.processing_task = asyncio.create_task(self._processing_loop())
        logger.info(f"Chat processor started (prefix: '{self.command_prefix}', batch: {self.batch_interval}s)")

    async def stop(self):
        """Stop the message processor loop."""
        self.running = False
        if self.processing_task:
            self.processing_task.cancel()
            try:
                await self.processing_task
            except asyncio.CancelledError:
                pass
        logger.info(f"Chat processor stopped (API calls: {self.api_calls_made}, commands: {self.commands_processed})")

    def add_message(self, username: str, text: str, broadcaster: str = ""):
        """
        Add a message to the processing queue.
        Only adds messages that start with the command prefix.

        Args:
            username: Twitch username
            text: Message text
            broadcaster: Broadcaster name
        """
        # Filter by command prefix
        if not text.strip().lower().startswith(self.command_prefix):
            return

        # Remove prefix from text
        command_text = text.strip()[len(self.command_prefix):].strip()
        if not command_text:
            return

        msg = ChatMessage(
            username=username,
            text=command_text,
            timestamp=time.time(),
            broadcaster=broadcaster
        )

        self.message_queue.append(msg)
        logger.debug(f"Queued message from {username}: {command_text}")

    async def _processing_loop(self):
        """Main processing loop - batches and processes messages."""
        while self.running:
            try:
                # Wait for batch interval
                await asyncio.sleep(self.batch_interval)

                # Process batch if we have messages
                if self.message_queue:
                    await self._process_batch()

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in processing loop: {e}")
                await asyncio.sleep(1.0)

    async def _process_batch(self):
        """Process the current batch of messages."""
        if not self.message_queue:
            return

        # Get all messages from queue
        batch = self.message_queue.copy()
        self.message_queue.clear()

        logger.info(f"Processing batch of {len(batch)} messages")

        # Process each message
        for msg in batch:
            # Try simple pattern matching first
            commands = self._try_pattern_match(msg.text)

            if commands:
                logger.info(f"Pattern matched '{msg.text}' -> {commands}")
                self.commands_processed += len(commands)
                # Yield commands (will be handled by callback)
                for cmd in commands:
                    await self._on_command_extracted(msg.username, cmd)
            else:
                # Use LLM for complex/ambiguous commands
                commands = await self._extract_commands_with_llm(msg.text)
                if commands:
                    logger.info(f"LLM extracted from '{msg.text}' -> {commands}")
                    self.commands_processed += len(commands)
                    for cmd in commands:
                        await self._on_command_extracted(msg.username, cmd)

    def _try_pattern_match(self, text: str) -> List[str]:
        """
        Try to match message against simple patterns.
        Returns list of commands if matched, empty list otherwise.
        """
        text_lower = text.lower().strip()

        # Check exact matches
        for pattern, command in self.simple_patterns.items():
            if pattern in text_lower:
                return [command]

        # Check tracking patterns
        tracking_keywords = ["follow", "track", "chase"]
        object_keywords = ["person", "dog", "cat", "ball", "human", "people"]

        for keyword in tracking_keywords:
            if keyword in text_lower:
                # Try to find object
                for obj in object_keywords:
                    if obj in text_lower:
                        return [f"start_tracking [{obj}]"]
                # Default to person if no object specified
                return ["start_tracking [person]"]

        return []

    async def _extract_commands_with_llm(self, text: str) -> List[str]:
        """
        Use OpenAI Chat API to extract commands from text.
        Returns list of command strings.
        """
        if not self.client:
            return []

        # Check rate limit
        if not self._check_rate_limit():
            logger.warning("Rate limit exceeded, skipping LLM call")
            return []

        try:
            # Make API call
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": text}
                ],
                temperature=0.3,
                max_tokens=100
            )

            self.api_calls_made += 1
            self._record_request()

            # Parse response
            content = response.choices[0].message.content.strip()
            if not content:
                return []

            # Split into lines and clean
            commands = [line.strip() for line in content.split('\n') if line.strip()]
            return commands

        except Exception as e:
            logger.error(f"Error calling OpenAI API: {e}")
            return []

    def _check_rate_limit(self) -> bool:
        """Check if we're within rate limit."""
        current_time = time.time()

        # Remove timestamps older than 1 minute
        self.request_timestamps = [
            ts for ts in self.request_timestamps
            if current_time - ts < 60
        ]

        # Check if we can make another request
        return len(self.request_timestamps) < self.max_requests_per_minute

    def _record_request(self):
        """Record a request timestamp for rate limiting."""
        self.request_timestamps.append(time.time())

    async def _on_command_extracted(self, username: str, command: str):
        """
        Callback when a command is extracted.
        Sends vote to VotingSystem if available, otherwise uses legacy callback.
        """
        if self.voting_system:
            # Send vote to bucket voting system
            self.voting_system.add_vote(username, command)
        else:
            # Legacy direct execution (for backwards compatibility)
            logger.info(f"Command extracted from {username}: {command}")

    def set_command_callback(self, callback):
        """
        Set a callback function to handle extracted commands.

        Args:
            callback: async function(username: str, command: str) -> None
        """
        self._on_command_extracted = callback


# Example usage
async def main():
    """Test the chat processor."""
    processor = ChatProcessor(
        command_prefix="!pupper",
        batch_interval=3.0,
        max_requests_per_minute=12
    )

    # Set up callback
    async def handle_command(username: str, command: str):
        print(f"[COMMAND] {username}: {command}")

    processor.set_command_callback(handle_command)

    # Start processor
    await processor.start()

    # Simulate some messages
    processor.add_message("alice", "!pupper walk forward")
    processor.add_message("bob", "!pupper dance and wiggle")
    processor.add_message("charlie", "!pupper follow that person")
    processor.add_message("dave", "regular chat message")  # Won't be processed

    # Wait for processing
    await asyncio.sleep(5)

    # Stop
    await processor.stop()


if __name__ == "__main__":
    asyncio.run(main())
