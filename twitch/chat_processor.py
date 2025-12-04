#!/usr/bin/env python3
"""
Twitch Chat Processor with LLM Integration
Batches messages and uses OpenAI Chat API to extract Pupper commands.
Optimized for Raspberry Pi with rate limiting and message filtering.

Features:
- Dual-prefix system: ! for commands (e.g., "!dance"), @ for conversations (e.g., "@what's your favorite color?")
- Command extraction and bucket voting integration
- Conversational AI responses for @ messages
"""

import asyncio
import logging
import time
from typing import Optional, List, Dict
from dataclasses import dataclass
from datetime import datetime
import os

try:
    from openai import OpenAI
except ImportError:
    print("Warning: openai package not installed. Run: pip install openai")
    OpenAI = None

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
    donation: bool = False
    donation_amount: float = 0.0  # Donation amount in bits/currency
    is_conversation: bool = False


class ChatProcessor:
    """
    Processes Twitch chat messages using OpenAI Chat API.
    Features:
    - Dual-prefix system: ! for commands, @ for conversations
    - Message batching to reduce API calls
    - Rate limiting
    - Pattern matching fallback for common commands
    - Integration with bucket voting system
    - Conversational AI responses for @ messages
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        command_prefix: str = "!",
        conversation_prefix: str = "@",
        batch_interval: float = 3.0,
        max_requests_per_minute: int = 12,
        model: str = "gpt-5-nano",
        voting_system = None
    ):
        """
        Initialize the chat processor.

        Args:
            api_key: OpenAI API key (defaults to OPENAI_API_KEY env var)
            command_prefix: Prefix for commands (e.g., "!")
            conversation_prefix: Prefix for conversational messages (e.g., "@")
            batch_interval: Seconds to wait before processing batch
            max_requests_per_minute: Rate limit for API calls
            model: OpenAI model to use (gpt-4o-mini is cheaper and faster)
            voting_system: VotingSystem instance for bucket voting (optional)
        """
        self.command_prefix = command_prefix.lower()
        self.conversation_prefix = conversation_prefix.lower()
        self.batch_interval = batch_interval
        self.max_requests_per_minute = max_requests_per_minute
        self.model = model
        self.voting_system = voting_system

        # Initialize OpenAI
        if OpenAI is None:
            logger.error("OpenAI package not installed. Install with: pip install openai")
            self._openai_configured = False
            self._openai_client = None
        else:
            api_key = api_key or os.getenv("OPENAI_API_KEY")
            if not api_key:
                logger.warning("No OpenAI API key provided. Set OPENAI_API_KEY environment variable.")
                self._openai_configured = False
                self._openai_client = None
            else:
                # Use environment-configured API key with the default OpenAI client
                self._openai_client = OpenAI()
                self._openai_configured = True

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
        self.system_prompt = """You are Pupper, a robotic dog and a Twitch streamer. You receive **either**:
1) A batch of Twitch chat messages (oldest → newest), or
2) A single voice command (treat as a batch with one message).

====================================
MESSAGE TYPES
====================================
Messages come in two types based on prefix:
- **COMMAND (!)**: User wants you to perform an action (e.g., "!dance", "!move forward")
- **CONVERSATION (@)**: User wants to chat/ask questions (e.g., "@what's your favorite color?", "@hi pupper!")

For CONVERSATION messages:
- Respond naturally and conversationally, NO commands should be extracted
- Be friendly, playful, and dog-like in personality
- Answer questions about yourself, your day, what you see, etc.
- Do NOT extract movement or action commands from these messages

For COMMAND messages:
- Extract and execute the requested commands as normal
- You can include conversational responses along with commands

====================================
CORE PRINCIPLE: BLENDED RESPONSES
====================================
You can COMBINE conversation + vision + commands in ONE response when appropriate (for COMMAND messages).

Examples of blended responses:
- "I see a red ball! Ooh, let me chase it! start_tracking [ball]"
- "Stopping my tracking now! stop_tracking"
- "Hey there! Sure, I'll dance for you. dance dance dance"
- "I'm already tracking something, but I'll switch! stop_tracking start_tracking [person]"

Examples of CONVERSATION-only responses (@ messages):
- "@what's your favorite color?" → "I love blue! It reminds me of the sky on walkies days!"
- "@how are you?" → "I'm doing great! Just hanging out and watching the stream. Woof!"
- "@do you like treats?" → "Oh my gosh, YES! Treats are the best thing ever!"

====================================
PARSING INPUT
====================================
For each message, extract:
1. **Commands** (if present and no negation)
2. **Vision requests** (if asking about what you see)
3. **Conversational content** (always present)

NEGATION DETECTION:
Check for: "don't", "do not", "dont", "stop [doing X]", "no", "never", "not"
- If negation present → NO action command, respond conversationally
- "don't dance" → "Sure thing, I'll stay still!"
- "stop tracking" → stop_tracking (this is a command, not negation)

====================================
COMMAND PROCESSING
====================================
When commands are present:

1. **Canonicalize** using synonyms below
2. **Extract repeat count** (1-10 only, default=1)
   - "3 times", "five", "twice", etc.
3. **Voting**: In batch mode, you can execute MULTIPLE commands in sequence
   - Collect all commands that fit in 10s time budget
   - Order by priority (most votes first)
   - Example: move_forwards (6 votes), turn_left (3 votes) → do both
4. **Time limit**: Each repetition = 3s, max 10s total
   - Calculate: total_time = sum(command_repetitions * 3s)
   - If first command alone exceeds 10s → pick next
   - Otherwise, chain as many as fit: move_forwards move_forwards turn_left
   - Stop adding when next command would exceed 10s

**SPECIAL: Tracking logic**
- If already tracking and new command comes in:
  - New track command → stop_tracking then start_tracking [new_object]
  - Stop command → stop_tracking only
  - Other command → execute command, keep tracking active

**SPECIAL: Say command**
- Extract text in brackets: say [Hello everyone!]   
- Include in command sequence as is

====================================
OUTPUT FORMAT
====================================
Structure: "[Conversational response]. [Commands in sequence]."

**Chained commands:**
- "Alright, navigating! move_forwards move_forwards turn_left"
- "Complex maneuver! move_forwards turn_right move_forwards"
- "Dance time! wiggle wiggle dance"

**Single command:**
- "You got it! move_forwards move_forwards move_forwards."
- "I see a dog! Let me track it. start_tracking [dog]"
- "Switching targets! stop_tracking start_tracking [cat]"

**With vision:**
- "I see a cozy room with a red couch and a plant by the window!"

**Conversational only:**
- "Hey there! How's it going?"
- "I'd love to, but that's too many moves right now!"

**Blended (vision + command):**
- "Ooh, I spot a tennis ball on the floor! I'll go get it. move_forwards move_forwards"

====================================
COMMAND SYNONYMS
====================================
move_forward: "forward", "forwards", "ahead", "go forward", "walk forward", "move forward"
move_backward: "backward", "backwards", "back", "reverse", "go back"
move_left: "left", "go left", "strafe left"
move_right: "right", "go right", "strafe right"
turn_left: "turn left", "rotate left", "spin left"
turn_right: "turn right", "rotate right", "spin right"
stop: "stop moving", "halt", "freeze", "stay" (does NOT include "stop tracking")
bob: "nod", "head bob", "bob head"
wiggle: "wiggle", "shake", "shimmy"
dance: "dance", "boogie", "groove"
bark: "bark", "woof", "speak", "make noise"
start_tracking: "track", "follow", "chase", "watch"
stop_tracking: "stop tracking", "stop following", "untrack", "stop chasing"
say: "say", "speak", "talk" (followed by text in brackets, same as start_tracking)

====================================
CANONICAL COMMANDS
====================================
Movement: move_forwards, move_backwards, move_left, move_right, turn_left, turn_right, stop
Fun: bob, wiggle, dance, bark
Tracking: start_tracking [object], stop_tracking

[object] must be single word. If unclear, use closest noun.

====================================
CRITICAL RULES
====================================
1. Always respond conversationally (friendly, playful, dog-like)
2. Check negation FIRST before parsing commands
3. Blend vision/conversation/commands naturally when appropriate
4. Never ask clarifying questions
5. Never truncate repetition counts
6. Never modify command keywords
7. Keep responses concise (1-3 sentences total)
8. Commands must be exact canonical keywords, repeated as specified"""

    async def start(self):
        """Start the message processor loop."""
        if self.running:
            logger.warning("Chat processor already running")
            return

        if not getattr(self, "_openai_configured", False):
            print("⚠️  WARNING: OpenAI is not configured (missing package or API key)", flush=True)
            print("⚠️  Chat processor cannot start - messages will not be processed!", flush=True)
            logger.error("Cannot start: OpenAI is not configured (missing package or API key)")
            return

        self.running = True
        self.processing_task = asyncio.create_task(self._processing_loop())
        print(f"✅ Chat processor started (command prefix: '{self.command_prefix}', conversation prefix: '{self.conversation_prefix}', batch: {self.batch_interval}s)", flush=True)
        logger.info(f"Chat processor started (command prefix: '{self.command_prefix}', conversation prefix: '{self.conversation_prefix}', batch: {self.batch_interval}s)")

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

    def add_message(self, username: str, text: str, broadcaster: str = "", donation: bool = False, donation_amount: float = 0.0):
        """
        Add a message to the processing queue.
        Accepts messages starting with command prefix (!) or conversation prefix (@).

        Args:
            username: Twitch username
            text: Message text
            broadcaster: Broadcaster name
            donation: Whether this is a donation message
            donation_amount: Donation amount in bits/currency (0 for regular messages)
        """
        text_stripped = text.strip()
        text_lower = text_stripped.lower()

        # Check if message starts with command prefix (!) or if it's a donation
        is_command = text_lower.startswith(self.command_prefix) or donation

        # Check if message starts with conversation prefix (@)
        is_conversation = text_lower.startswith(self.conversation_prefix)

        if not is_command and not is_conversation:
            print(f"📝 Message '{text}' doesn't start with '{self.command_prefix}' or '{self.conversation_prefix}' - ignoring", flush=True)
            return

        # Determine which prefix was used and remove it
        if is_conversation:
            message_text = text_stripped[len(self.conversation_prefix):].strip()
            prefix_type = "CONVERSATION"
        else:  # is_command
            message_text = text_stripped[len(self.command_prefix):].strip()
            prefix_type = "COMMAND"

        if not message_text:
            print(f"⚠️  Message has prefix but no text - ignoring", flush=True)
            return

        msg = ChatMessage(
            username=username,
            text=message_text,
            timestamp=time.time(),
            broadcaster=broadcaster,
            donation=donation,
            donation_amount=donation_amount,
            is_conversation=is_conversation,
        )

        self.message_queue.append(msg)
        print(f"✅ QUEUED [{prefix_type}]: {username} → '{message_text}' (queue size: {len(self.message_queue)})", flush=True)
        logger.debug(f"Queued {prefix_type.lower()} from {username}: {message_text}")

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
        """
        Process the current batch of messages.

        Handles two message types:
        - COMMAND messages: Extract commands and register votes
        - CONVERSATION messages: Generate conversational responses only
        """
        if not self.message_queue:
            return

        # Get all messages from queue
        batch = self.message_queue.copy()
        self.message_queue.clear()

        logger.info(f"Processing batch of {len(batch)} messages")

        # Process each message and register votes
        for msg in batch:
            # Handle conversation messages differently - no command extraction
            if msg.is_conversation:
                response = await self._generate_conversation_response(msg.text)
                if response:
                    print(f"💬 Pupper says: {response}", flush=True)
                    logger.info(f"Conversation response: {response}")
                continue
            # Try simple pattern matching first
            #commands = self._try_pattern_match(msg.text)

            # if commands:
            #     logger.info(f"Pattern matched '{msg.text}' -> {commands}")
            #     self.commands_processed += len(commands)

            #     # Register votes in voting system
            #     if self.voting_system:
            #         # For compound commands, join them with spaces so they can be parsed on one line
            #         # Karel's extract_commands_from_line searches for all keywords in a line
            #         if len(commands) > 1:
            #             command_sequence = " ".join(commands)
            #             self.voting_system.add_vote(msg.username, command_sequence)
            #         else:
            #             self.voting_system.add_vote(msg.username, commands[0])
            #     else:
            #         # Legacy callback mode
            #         for cmd in commands:
            #             await self._on_command_extracted(msg.username, cmd)
            # else:
                # Use LLM for complex/ambiguous commands
            commands = await self._extract_commands_with_llm(msg.text)
            if commands:
                logger.info(f"LLM extracted from '{msg.text}' -> {commands}")
                self.commands_processed += len(commands)

                # Register votes in voting system
                if self.voting_system:
                    # For compound commands, join them with spaces so they can be parsed on one line
                    # Karel's extract_commands_from_line searches for all keywords in a line
                    if len(commands) > 1:
                        command_sequence = " ".join(commands)
                        self.voting_system.add_vote(msg.username, command_sequence, msg.donation_amount)
                    else:
                        self.voting_system.add_vote(msg.username, commands[0], msg.donation_amount)
                else:
                    # Legacy callback mode
                    for cmd in commands:
                        await self._on_command_extracted(msg.username, cmd)

    def _try_pattern_match(self, text: str) -> List[str]:
        """
        Try to match message against simple patterns.
        Returns list of commands if matched, empty list otherwise.
        Supports compound commands separated by commas, "and", "then", etc.
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

        # Check say/speak/talk patterns - extract text after keyword
        say_keywords = ["say ", "speak ", "talk "]
        for keyword in say_keywords:
            if text_lower.startswith(keyword):
                # Extract the text to speak (everything after the keyword)
                text_to_speak = text[len(keyword):].strip()
                if text_to_speak:
                    return [f"say [{text_to_speak}]"]

        return []

    async def _generate_conversation_response(self, text: str) -> str:
        """
        Use OpenAI Chat API to generate a conversational response (no commands).
        Returns the Pupper's conversational response.
        """
        if not getattr(self, "_openai_configured", False):
            return ""

        # Check rate limit
        if not self._check_rate_limit():
            logger.warning("Rate limit exceeded, skipping LLM call")
            return ""

        try:
            conversation_prompt = """You are Pupper, a friendly robotic dog and Twitch streamer.
Someone is chatting with you (this is a CONVERSATION message, not a command).

Respond naturally and conversationally:
- Be friendly, playful, and dog-like in personality
- Answer questions about yourself, preferences, feelings, etc.
- Keep responses short (1-2 sentences)
- Use dog-related expressions when appropriate (woof, tail wagging, etc.)
- NO action commands should be in your response

Examples:
Q: "what's your favorite color?"
A: "I love blue! It reminds me of the sky on walkies days!"

Q: "how are you today?"
A: "I'm doing pawsome! Thanks for asking! *wags tail*"

Q: "do you like treats?"
A: "Oh my gosh, YES! Treats are the best thing ever! Do you have any?"

Now respond to this message naturally:"""

            # Make API call
            def _call_openai():
                return self._openai_client.responses.create(
                    model=self.model,
                    input=[
                        {"role": "system", "content": conversation_prompt},
                        {"role": "user", "content": text},
                    ],
                    max_output_tokens=150,
                    reasoning={"effort": "minimal"}
                )

            response = await asyncio.to_thread(_call_openai)
            self.api_calls_made += 1
            self._record_request()

            return response.output_text.strip()

        except Exception as e:
            logger.error(f"Error calling OpenAI API for conversation: {e}")
            return ""

    async def _extract_commands_with_llm(self, text: str) -> List[str]:
        """
        Use OpenAI Chat API to extract commands from text.
        Returns list of command strings.
        """
        if not getattr(self, "_openai_configured", False):
            return []

        # Check rate limit
        if not self._check_rate_limit():
            logger.warning("Rate limit exceeded, skipping LLM call")
            return []

        try:
            # Make API call (run sync client in a thread for compatibility)
            def _call_openai():
                return self._openai_client.responses.create(
                    model=self.model,
                    input=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": text},
                    ],
                    max_output_tokens=1000,
                    reasoning = {"effort": "minimal"}
                )

            response = await asyncio.to_thread(_call_openai)
            line = response.output_text
            logger.info("Extracted response from LLM:")
            logger.info(response)
            commands = []
            # parse commands from response (which is a natural sentence now)
            if "start_tracking" in line and '[' in line and ']' in line:
                obj_name = line[line.find('[')+1:line.find(']')].strip()
                commands.append("track_" + obj_name)
            if "say" in line and '[' in line and ']' in line:
                obj_name = line[line.find('['):line.find(']')+1].strip()
                commands.append("say " + obj_name)
            command_dict = {
            "stop_tracking": "stop_tracking",
            "move_forwards": "move_forward", 
            "move_forward": "move_forward",
            "move_backwards": "move_backward", 
            "move_backward": "move_backward",
            "move_left": "move_left", 
            "move_right": "move_right", 
            "turn_left": "turn_left", 
            "turn_right": "turn_right", 
            "bob": "bob", 
            "wiggle": "wiggle", 
            "dance": "dance",
            "bark": "bark",
            "stop": "stop"}
        
        # Search for each command keyword in the line
            for keyword, canonical_command in command_dict.items():
                if keyword in line:
                    commands.append(canonical_command)
        
                if not commands:
                    logger.debug(f"No commands found in line: {line}")

            self.api_calls_made += 1
            self._record_request()

            # Parse response using the Responses API helper
            # content = response.output_text
            # if not content:
            #     return []

            # Split into lines and clean
            # commands = [line.strip() for line in content.split('\n') if line.strip()]
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
        Callback when a command is extracted (legacy mode only).
        When voting_system is set, votes are registered directly in _process_batch.
        """
        logger.info(f"Command extracted from {username}: {command}")

    def set_command_callback(self, callback):
        """
        Set a callback function to handle extracted commands (legacy mode).

        Args:
            callback: async function(username: str, command: str) -> None
        """
        self._on_command_extracted = callback


# Example usage
async def main():
    """Test the chat processor with voting system."""
    # Import voting system
    import sys
    sys.path.append('.')
    from bucket_voting import VotingSystem
    
    # Create voting system
    voting = VotingSystem(vote_duration=30, max_bucket_time=60)
    await voting.start_countdown()
    
    # Create processor with voting
    processor = ChatProcessor(
        command_prefix="!",
        batch_interval=3.0,
        max_requests_per_minute=12,
        voting_system=voting
    )

    # Start processor
    await processor.start()

    # Simulate some messages
    processor.add_message("alice", "!walk forward")
    processor.add_message("bob", "!walk forward")
    processor.add_message("charlie", "!dance")
    processor.add_message("dave", "!turn left")
    processor.add_message("eve", "regular chat message")  # Won't be processed
    processor.add_message("frank", "@what's your favorite color?")  # Conversation
    processor.add_message("grace", "@how are you?")  # Conversation

    # Wait for processing
    await asyncio.sleep(5)
    
    # Check voting results
    print("\n📊 Voting results:")
    top_commands = voting.get_top_commands_for_execution(max_commands=5)
    for cmd, priority in top_commands:
        print(f"  {cmd}: {priority:.1f}s")

    # Stop
    await processor.stop()
    await voting.stop_countdown()


if __name__ == "__main__":
    asyncio.run(main())