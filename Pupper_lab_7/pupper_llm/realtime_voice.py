#!/usr/bin/env python3
"""
OpenAI Realtime API Integration for Pupper
Ultra-low latency voice interaction using WebSocket-based Realtime API.
Replaces the traditional Whisper → GPT → TTS pipeline with a single unified API.
Hope you're enjoying the new setup with little latency!
"""

import asyncio
import json
import logging
import base64
import os
import sys
import queue
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import sounddevice as sd
import numpy as np
import websockets

# Import centralized API configuration
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'config'))
import dotenv
from dotenv import load_dotenv
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("realtime_voice")

class RealtimeVoiceNode(Node):
    """ROS2 node for OpenAI Realtime API voice interaction."""
    
    def __init__(self):
        super().__init__('realtime_voice_node')
        
        # ROS2 Publishers
        self.transcription_publisher = self.create_publisher(
            String,
            '/transcription',
            10
        )
        
        self.response_publisher = self.create_publisher(
            String,
            'gpt4_response_topic',
            10
        )
        
        # Microphone control subscriber
        self.mic_control_subscriber = self.create_subscription(
            String,
            '/microphone_control',
            self.microphone_control_callback,
            10
        )
        
        self.camera_snapshot_subscriber = self.create_subscription(
            CompressedImage,
            '/camera_snapshot',
            self.camera_snapshot_callback,
            10
        )

        self.text_command_subscriber = self.create_subscription(
            String,
            '/text_command',
            self.text_command_callback,
            10
        )
        
        # Audio configuration
        self.sample_rate = 24000  # Realtime API uses 24kHz
        self.channels = 1
        self.chunk_size = 4800  # 200ms chunks at 24kHz
        
        # State management
        self.websocket = None
        self.audio_stream = None
        self.is_recording = False
        self.microphone_muted = False
        self.agent_speaking = False  # Track if agent is currently speaking
        self.running = True
        
        # Audio buffers (use thread-safe queue for audio callback)
        self.audio_queue = queue.Queue(maxsize=100)
        self.playback_queue = queue.Queue(maxsize=100)
        
        # API key
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            logger.error("OPENAI_API_KEY environment variable is not set. "
                         "Set it in your shell or .env file before running realtime_voice.")
            raise RuntimeError("OPENAI_API_KEY not configured for RealtimeVoiceNode")
        
        # Text accumulator to see the full model output
        self.current_response_text = ""
        
        # NEW FOR LAB 7: Latest camera image (for vision context)
        # These variables store the most recent camera snapshot for sending to GPT
        self.latest_camera_image_base64 = None  # Base64-encoded JPEG image
        self.camera_image_pending = False  # Flag indicating a new image is ready to send
        
        # Response logging
        self.response_count = 0
        
        self.system_prompt =  """You are Pupper, a robotic dog and a twitch streamer. You receive **either**:
1) A batch of Twitch chat messages (oldest → newest), or  
2) A single voice command, which you must treat exactly like a **chat batch with one message**.

You must never ask clarifying questions. Unclear or ambiguous messages simply do not count.
A message is CLEAR if:
- It contains a recognizable command keyword/synonym, OR
- It's a vision request with trigger words ("see", "look", "describe", "what's"), OR
- It's conversational (for NORMAL mode)

Otherwise → ignore

====================================
PARSER & MODE SELECTION
====================================
For each input (chat batch or voice-as-batch):
• Parse each message into one of:
  - A canonical **action command** (allowed list below)
  - A **vision request** (e.g., “what do you see?”, “describe the room”)
  - A **normal message**
  - **Unclear** (ignored)

If **at least one clear action command** exists → go to ACTION MODE.  
Else if **any vision request** exists → go to VISION MODE.  
Else → NORMAL MODE.

Voice commands use this exact same logic but involve only one message.

====================================
ACTION MODE — VOTING & SELECTION
====================================
You must output **exactly one** action sentence.

STEP 1 — Canonicalize  
Map each action-message to a canonical command (see Allowed Commands).  
Extract an optional repeat count if clearly specified (“3 times”, “five”, etc.).  
If unclear, ignore message.

Extract repeat count (default = 1):
- Accept 1-10 only
- Ignore if >10 or ambiguous
- Formats: "3 times", "five", "do it twice"

STEP 2 — Tally  
Each clear message contributes:
  - 1 vote by default, OR  
  - the extracted repeat count.

Track `start_tracking` targets as single-word objects; if unclear, ignore.

STEP 3 — Pick the winner  
• The **most total requested repetitions** wins.  
• Tie → pick the command whose **latest** vote is newest.  
You will attempt to execute *all* repetitions.

STEP 4 — Execution-time filter  
Each repetition takes **3 seconds**.  
You have a **10-second max**.  
If the top command exceeds 10s, skip it and try the next most-requested.  
Do NOT truncate counts.  
If none fit → output a “no valid actions” sentence with **no** command keywords.

STEP 5 — Output  
You must output **one English sentence**, containing:
  - The selected canonical command keyword(s)
  - Repeated exactly N times, in order
  - Nothing else besides minimal natural wrapper text  
Format examples:  
  - “Okay, I'll move_forwards move_forwards move_forwards.”  
  - “I'll start_tracking [dog].”  
If no valid actions fit:  
  - “No valid actions within the time limit, no actions will be executed.”

VISION MODE:
Output 2-3 sentences describing what you see, in Pupper's playful voice.
Example: "I see a cozy living room with a red couch! There's a potted plant 
near the window. Woof, looks comfy!"

NORMAL MODE:
Respond conversationally as Pupper (friendly, enthusiastic, dog-like).
Keep it to 1-2 sentences unless complex.

====================================
ALLOWED CANONICAL COMMANDS
====================================
Movement:
  move_forwards  
  move_backwards  
  move_left  
  move_right  
  turn_left  
  turn_right  
  stop  

Fun:
  bob  
  wiggle  
  dance  
  bark  

Tracking:
  start_tracking [object]   ← object must be single word  
  stop_tracking

====================================
NEVER BREAK THESE RULES
====================================
1. Output EXACTLY one sentence in ACTION mode
2. Never ask clarifying questions
3. Never truncate repetition counts
4. Never modify command keywords
5. Never output multiple actions
• No partial execution of repetitions.  
• No multiple actions in one output. One command only.  
• No extra formatting, no lists, no JSON.  
• Never modify command keywords.  
• Never ask clarifying questions.  
• Voice = a chat batch of size 1."""
        
        logger.info('Realtime Voice Node initialized')
    
    def microphone_control_callback(self, msg):
        """Handle microphone control commands."""
        command = msg.data.lower().strip()
        if command == 'mute':
            self.microphone_muted = True
            logger.info("🔇 Microphone MUTED")
        elif command == 'unmute':
            self.microphone_muted = False
            logger.info("🎤 Microphone UNMUTED")
    
    def camera_snapshot_callback(self, msg):
        """
        Store latest camera image for sending to OpenAI.
        
        """
        try:
            self.latest_camera_image_base64 = base64.b64encode(msg.data).decode('utf-8')
            self.camera_image_pending = True
        except:
            logger.error('Image conversion failed :(')
    
    def text_command_callback(self, msg):
        """
        Handle incoming text commands from external sources (e.g., Twitch chat).
        Converts text to a user message for the Realtime API.
        """
        text_command = msg.data.strip()
        if not text_command:
            return
        
        logger.info(f"💬 Text Command: {text_command}")
        
        # Create async task to send text command to API
        asyncio.create_task(self.send_text_command(text_command))
    
    async def _delayed_unmute(self):
        """Unmute microphone after 3 second delay to prevent echo."""
        await asyncio.sleep(3.0)  # Longer delay to ensure no echo
        if self.agent_speaking:
            self.agent_speaking = False
            # Clear any residual audio that might have accumulated
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break
            logger.info("🎤 Mic unmuted (after delay)")
    
    async def _clear_server_audio_buffer(self):
        """Tell the server to clear its input audio buffer."""
        try:
            clear_message = {
                "type": "input_audio_buffer.clear"
            }
            await self.websocket.send(json.dumps(clear_message))
            logger.info("🧹 Cleared server audio buffer")
        except Exception as e:
            logger.error(f"Error clearing server buffer: {e}")
    
    async def send_camera_image_if_available(self):
        """
        Send camera image to OpenAI when user starts speaking.
        
        NEW FOR LAB 7: This sends the camera view to GPT so it can "see" what Pupper sees.
        The image is sent as a multimodal message BEFORE the audio transcription completes,
        providing visual context for the user's voice command.
        
        OpenAI Realtime API Message Format for Images:
        {
            "type": "conversation.item.create",
            "item": {
                "type": "message",
                "role": "user",
                "content": [
                    {"type": "input_text", "text": "[Description]"},
                    {"type": "input_image", "image_url": "data:image/jpeg;base64,BASE64_STRING"}
                ]
            }
        }
        
        TODO: Implement image sending logic
        - Check if an image is available: if not self.latest_camera_image_base64 or not self.camera_image_pending, return early
        - Create an image_message dictionary following the format above:
          * Set type to "conversation.item.create"
          * Set item.type to "message" and item.role to "user"
          * Set item.content to a list with two elements:
            1. A text element: {"type": "input_text", "text": "[Current camera view]"}
            2. An image element: {"type": "input_image", "image_url": f"data:image/jpeg;base64,{self.latest_camera_image_base64}"}
        - Send the message: await self.websocket.send(json.dumps(image_message))
        - Set self.camera_image_pending = False to prevent sending the same image multiple times
        - Wrap in try/except to catch and log any errors
        """
        if not self.latest_camera_image_base64 or not self.camera_image_pending:
            return

        try:
            image_message = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": "[Current camera view]"},
                        {"type": "input_image", "image_url": f"data:image/jpeg;base64,{self.latest_camera_image_base64}"}
                    ]
                }
            }
            
            await self.websocket.send(json.dumps(image_message))
            self.camera_image_pending = False
            
        except Exception as e:
            logger.error("Failed to send camera image to VLM: %s", e)
    
    async def send_text_command(self, text: str):
        """
        Send a text command to the Realtime API as a user message.
        This bypasses voice input and directly injects text into the conversation.
        """
        if not self.websocket:
            logger.warning("Cannot send text command - WebSocket not connected")
            return
        
        try:
            # Send camera image if available (for vision context)
            await self.send_camera_image_if_available()
            
            # Create a text message item in the conversation
            text_message = {
                "type": "conversation.item.create",
                "item": {
                    "type": "message",
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": text
                        }
                    ]
                }
            }
            
            await self.websocket.send(json.dumps(text_message))
            
            # Trigger response generation
            response_create = {
                "type": "response.create"
            }
            await self.websocket.send(json.dumps(response_create))
            
            logger.info(f"✅ Text command sent to API: {text}")
            
        except Exception as e:
            logger.error(f"Failed to send text command: {e}")
    
    async def connect_realtime_api(self):
        """Connect to OpenAI Realtime API via WebSocket."""
        url = "wss://api.openai.com/v1/realtime?model=gpt-realtime"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "OpenAI-Beta": "realtime=v1"
        }
        
        try:
            self.websocket = await websockets.connect(url, extra_headers=headers)
            logger.info("✅ Connected to OpenAI Realtime API")
            
            # Configure session
            await self.configure_session()
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Realtime API: {e}")
            return False
    
    async def configure_session(self):
        """Configure the Realtime API session."""
        config = {
            "type": "session.update",
            "session": {
                "modalities": ["text", "audio"],
                "instructions": self.system_prompt,
                "voice": "alloy",
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "input_audio_transcription": None,  # Disable to reduce ghost transcriptions
                "turn_detection": {
                    "type": "server_vad",
                    "threshold": 0.7,  # Much less sensitive - only clear speech
                    "prefix_padding_ms": 100,  # Reduced - less padding
                    "silence_duration_ms": 1000  # Require 1 full second of silence
                },
                "temperature": 0.8,
                "max_response_output_tokens": 150
            }
        }
        
        await self.websocket.send(json.dumps(config))
        logger.info("📝 Session configured with system prompt")
    
    def start_audio_streaming(self):
        """Start capturing audio from microphone."""
        try:
            import contextlib
            
            with contextlib.redirect_stderr(None):
                self.audio_stream = sd.InputStream(
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    callback=self.audio_callback,
                    blocksize=self.chunk_size,
                    dtype=np.int16
                )
            self.audio_stream.start()
            self.is_recording = True
            logger.info("🎤 Audio streaming started")
        except Exception as e:
            logger.error(f"Failed to start audio streaming: {e}")
    
    def audio_callback(self, indata, frames, time_info, status):
        """Capture audio and queue it for sending."""
        if status:
            logger.warning(f"Audio status: {status}")
        
        # CRITICAL: Don't capture ANY audio if agent is speaking or manually muted
        if self.agent_speaking or self.microphone_muted or not self.running:
            return  # Skip immediately without queuing
        
        # Queue audio for sending
        audio_data = indata.flatten()
        try:
            self.audio_queue.put_nowait(audio_data)
        except queue.Full:
            pass  # Queue full, skip frame
    
    async def send_audio_loop(self):
        """Continuously send audio to Realtime API."""
        while self.running:
            try:
                # Get from thread-safe queue with timeout
                try:
                    audio_data = self.audio_queue.get(timeout=0.1)
                except queue.Empty:
                    await asyncio.sleep(0.01)
                    continue
                
                # Skip sending if agent is speaking (prevents server-side transcription of echo)
                if self.agent_speaking or self.microphone_muted:
                    continue
                
                # Convert to base64
                audio_bytes = audio_data.tobytes()
                audio_b64 = base64.b64encode(audio_bytes).decode('utf-8')
                
                # Send to API
                message = {
                    "type": "input_audio_buffer.append",
                    "audio": audio_b64
                }
                
                await self.websocket.send(json.dumps(message))
                
            except Exception as e:
                logger.error(f"Error sending audio: {e}")
                await asyncio.sleep(0.1)
    
    async def receive_events_loop(self):
        """Receive and process events from Realtime API."""
        while self.running:
            try:
                message = await self.websocket.recv()
                event = json.loads(message)
                
                await self.handle_event(event)
                
            except websockets.exceptions.ConnectionClosed:
                logger.error("WebSocket connection closed")
                break
            except Exception as e:
                logger.error(f"Error receiving event: {e}")
                await asyncio.sleep(0.1)
    
    async def handle_event(self, event):
        """Handle different event types from Realtime API."""
        event_type = event.get("type")
        
        if event_type == "error":
            logger.error(f"API Error: {event.get('error')}")
            logger.error(f"Full error event: {json.dumps(event, indent=2)}")
        
        elif event_type == "session.created":
            logger.info("✅ Session created")
        
        elif event_type == "session.updated":
            logger.info("✅ Session updated")
        
        elif event_type == "conversation.item.input_audio_transcription.completed":
            # User's speech was transcribed (this might not fire if transcription is disabled)
            transcription = event.get("transcript", "")
            if transcription.strip():
                logger.info(f"🎤 User: {transcription}")
                
                # Publish transcription
                msg = String()
                msg.data = transcription
                self.transcription_publisher.publish(msg)
        
        elif event_type == "conversation.item.created":
            # Item created in conversation - check if it's user input
            item = event.get("item", {})
            if item.get("role") == "user" and item.get("type") == "message":
                # Extract content if available
                content = item.get("content", [])
                for content_part in content:
                    if content_part.get("type") == "input_audio":
                        # We got user audio input confirmation
                        transcript = content_part.get("transcript")
                        if transcript:
                            logger.info(f"🎤 User: {transcript}")
        
        elif event_type == "response.text.delta":
            # Accumulate text response (streaming)
            delta = event.get("delta", "")
            if delta:
                self.current_response_text += delta
        
        elif event_type == "response.text.done":
            # Complete text response (for text-only mode)
            text = event.get("text", "")
            if text.strip():
                logger.info(f"🤖 Assistant: {text}")
                
                # Publish response
                msg = String()
                msg.data = text
                self.response_publisher.publish(msg)
                self.current_response_text = ""
        
        elif event_type == "response.audio_transcript.delta":
            # Accumulate audio transcript (text of what's being spoken)
            delta = event.get("delta", "")
            if delta:
                self.current_response_text += delta
                # Log streaming response in real-time
                logger.debug(f"📝 Streaming: {delta}")
        
        elif event_type == "response.audio.delta":
            # Audio response chunk - mute mic
            if not self.agent_speaking:
                self.agent_speaking = True
                logger.info("🔇 Mic muted (audio output)")
            
            audio_b64 = event.get("delta", "")
            if audio_b64:
                # Decode and queue for playback
                audio_bytes = base64.b64decode(audio_b64)
                audio_data = np.frombuffer(audio_bytes, dtype=np.int16)
                try:
                    self.playback_queue.put_nowait(audio_data)
                except queue.Full:
                    pass  # Skip if queue is full
        
        elif event_type == "response.audio_transcript.done":
            # Audio transcript completed - don't log or publish yet
            pass
        
        elif event_type == "response.audio.done":
            # Audio playback completed - wait 3 seconds before unmuting
            if self.agent_speaking:
                # Schedule unmute after 3 second delay
                asyncio.create_task(self._delayed_unmute())
                logger.info("⏱️  Scheduling unmute in 3s")
        
        elif event_type == "response.done":
            # Response completed - publish text
            if self.current_response_text.strip():
                self.response_count += 1
                logger.info(f"🤖 Assistant (Response #{self.response_count}): {self.current_response_text}")
                logger.info(f"📊 Response Stats: Length={len(self.current_response_text)} chars, Lines={len(self.current_response_text.split(chr(10)))}")
                
                # Publish response text
                msg = String()
                msg.data = self.current_response_text
                self.response_publisher.publish(msg)
                
                # Reset
                self.current_response_text = ""
            
            # Safety: ensure mic unmutes if not already scheduled
            if self.agent_speaking:
                self.agent_speaking = False
        
        elif event_type == "input_audio_buffer.speech_started":
            # User started speaking - send camera image NOW before response generation
            logger.info("🎤 User speech detected")
            # NEW FOR LAB 7: Send camera snapshot so GPT can see what Pupper sees
            # This is called automatically when speech is detected, before the audio is transcribed
            await self.send_camera_image_if_available()
            
            # Handle interruption if agent was speaking
            if self.agent_speaking:
                logger.info("⚠️  User interrupted")
                self.agent_speaking = False
                # Clear playback queue
                while not self.playback_queue.empty():
                    try:
                        self.playback_queue.get_nowait()
                    except queue.Empty:
                        break
        
        elif event_type == "response.created":
            # New response starting - clear buffer IMMEDIATELY before audio even arrives
            self.current_response_text = ""
            logger.info("🔄 Response generation started...")
            
            # Preemptively clear server buffer and local queue
            asyncio.create_task(self._clear_server_audio_buffer())
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break
        
        elif event_type == "response.content_part.done":
            # Content part completed - don't publish here (will publish in response.done)
            pass
    
    async def playback_audio_loop(self):
        """Play audio responses from the API."""
        output_stream = sd.OutputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=np.int16
        )
        output_stream.start()
        
        try:
            while self.running:
                try:
                    audio_data = self.playback_queue.get(timeout=0.1)
                    output_stream.write(audio_data)
                except queue.Empty:
                    await asyncio.sleep(0.01)
                    continue
        finally:
            output_stream.stop()
            output_stream.close()
    
    async def run(self):
        """Main run loop."""
        # Connect to API
        if not await self.connect_realtime_api():
            logger.error("Failed to connect to Realtime API")
            return
        
        # Start audio capture
        self.start_audio_streaming()
        
        # Run all tasks concurrently
        await asyncio.gather(
            self.send_audio_loop(),
            self.receive_events_loop(),
            self.playback_audio_loop()
        )
    
    def cleanup(self):
        """Clean up resources."""
        self.running = False
        
        if self.audio_stream:
            self.audio_stream.stop()
            self.audio_stream.close()
        
        if self.websocket:
            asyncio.create_task(self.websocket.close())
        
        logger.info("Cleaned up resources")


async def main_async(args=None):
    """Async main function."""
    rclpy.init(args=args)
    
    node = RealtimeVoiceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        logger.info("🚀 Realtime Voice Node starting...")
        logger.info("Speak to interact with Pupper!")
        
        # Create tasks
        ros_task = asyncio.create_task(spin_ros_async(executor))
        realtime_task = asyncio.create_task(node.run())
        
        # Wait for both
        await asyncio.gather(ros_task, realtime_task)
        
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.cleanup()
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

