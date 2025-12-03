# karel.py - Enhanced with Object Tracking and TTS
import time
import os
import tempfile
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import simpleaudio as sa
import pygame

class KarelPupper:
    def start():
        if not rclpy.ok():
            rclpy.init()

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # NEW FOR LAB 7: Tracking control publisher
        # This publisher sends tracking commands to the state machine node
        self.tracking_control_publisher = self.node.create_publisher(
            String, '/tracking_control', 10
        )
        
        # NEW FOR LAB 7: Track current tracking state
        self.tracking_enabled = False
        self.tracking_object = None

    def begin_tracking(self, obj: str = "person"):
        """
        NEW FOR LAB 7: Start tracking a specific object from the COCO dataset.
        
        This function enables visual tracking of objects detected by the camera.
        Pupper will automatically follow the specified object using the state machine.
        
        Args:
            obj: Object class to track (e.g., "person", "dog", "cat", "car", "bottle", "chair", etc.)
                 Default is "person". Uses COCO dataset class names (80+ objects supported).
        
        TODO: Implement tracking start logic
        - Set self.tracking_enabled = True
        - Store the object name in self.tracking_object
        - Create a String message with msg.data = f"start:{obj}"
        - Publish the message using self.tracking_control_publisher.publish(msg)
        - Call rclpy.spin_once(self.node, timeout_sec=0.1) to ensure message is sent
        - Log the action: self.node.get_logger().info(f'Started tracking: {obj}')
        """
        msg = String()
        self.tracking_enabled = True
        self.tracking_object = obj
        msg.data = f"start:{obj}"
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info(f'Started tracking: {obj}')
        
    def end_tracking(self):
        """
        NEW FOR LAB 7: Stop tracking and return to idle state.
        
        This function disables tracking mode and returns control to manual commands.
        
        TODO: Implement tracking stop logic
        - Set self.tracking_enabled = False
        - Clear self.tracking_object (set to None)
        - Create a String message with msg.data = "stop"
        - Publish the message using self.tracking_control_publisher.publish(msg)
        - Call rclpy.spin_once(self.node, timeout_sec=0.1) to ensure message is sent
        - Call self.stop() to halt movement
        - Log the action: self.node.get_logger().info('Stopped tracking')
        """
        msg = String()
        self.tracking_enabled = False
        self.tracking_object = None
        msg.data = "stop"
        self.tracking_control_publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        self.stop()
        self.node.get_logger().info('Stopped tracking')

    def move(self, linear_x, linear_y, angular_z):
        move_cmd = Twist()
        move_cmd.linear.x = linear_x
        move_cmd.linear.y = linear_y
        move_cmd.angular.z = angular_z
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move...')
        self.stop()
    
    def wiggle(self, wiggle_time=6, play_sound=True):
        # Play wiggle sound if requested
        if play_sound:
            pygame.mixer.init()
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
            wav_path = os.path.join(sounds_dir, 'puppy_wiggle.wav')
            wav_path = os.path.normpath(wav_path)
            
            try:
                wiggle_sound = pygame.mixer.Sound(wav_path)
                wiggle_sound.play()
                self.node.get_logger().info(f'Playing wiggle sound from: {wav_path}')
            except Exception as e:
                self.node.get_logger().warning(f"Could not play wiggle sound: {e}")

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        # Alternate wiggle directions for a total of 1 second
        single_wiggle_duration = 0.2  # seconds per half-wiggle
        angular_speed = 0.8
        
        start_time = time.time()
        direction = 1
        while time.time() - start_time < wiggle_time:
            move_cmd.angular.z = direction * angular_speed
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(single_wiggle_duration)
            direction *= -1  # Switch direction
        
        self.stop()

        self.node.get_logger().info('Wiggle!')
    
    def bob(self, bob_time=5, play_sound=True):
        """
        Makes the robot bob back and forth by moving forward and backward with a specified speed and duration.

        TODO: Paste your implementation from Lab 6
        1. Play a 'puppy_bob.wav' sound if play_sound is True.
            - Use pygame.mixer to initialize the sound engine.
            - Load the 'puppy_bob.wav' file from the sounds directory.
            - Play the sound and handle any exceptions gracefully, logging them with self.node.get_logger().
        2. Publish alternating Twist messages to make the robot bob forward and backward.
            - Bob back and forth with a configurable speed and duration (bob_time).
            - Alternate the direction of linear.x every 0.2 seconds (half_bob_duration).
            - Call rclpy.spin_once and use time.sleep to manage timing.
        3. Call self.stop() at the end to halt the robot.

        Remove the 'pass' statement after you implement the steps above.
        """
        # ==== TODO: Paste your Lab 6 implementation here ====
        if play_sound:
            pygame.mixer.init()
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
            wav_path = os.path.join(sounds_dir, 'puppy_bob.wav')
            wav_path = os.path.normpath(wav_path)
            
            try:
                bob_sound = pygame.mixer.Sound(wav_path)
                bob_sound.play()
                self.node.get_logger().info(f'Playing bob sound from: {wav_path}')
            except Exception as e:
                self.node.get_logger().warning(f"Could not play bob sound: {e}")


        move_cmd = Twist()
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        # Alternate movement directions for a total of 1 second
        single_bob_duration = 0.3  # seconds per half-wiggle
        bob_speed = 0.35
        
        start_time = time.time()
        direction = 1
        while time.time() - start_time < bob_time:
            move_cmd.linear.x = direction * bob_speed
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(single_bob_duration)
            direction *= -1  # Switch direction
        
        self.stop()

        self.node.get_logger().info('Bob!')

        self.node.get_logger().info('Bob!')

    def move_forward(self):
        """
        TODO: Implement moving Pupper forward.
        - Decide on an appropriate linear.x speed for safe forward movement.
        - Use the move() helper function that is implemented above, or manually construct move_cmd = Twist().
        - Publish the Twist command for a set duration, then stop.
        """
        self.move(1.0, 0.0, 0.0)
        self.node.get_logger().info("Finished running move_forward...")

    def move_backward(self):
        """
        TODO: Implement moving Pupper backward.
        - Decide on a negative linear.x value for safe backward movement.
        - Use move() or create your own Twist message.
        - Be careful with speed—backward motion is often best slower.
        """
        self.move(-1.0, 0.0, 0.0)
        self.node.get_logger().info("Finished running move_backward...")

    def move_left(self):
        """
        TODO: Implement moving Pupper to the left (translation).
        - Set an appropriate linear.y value for left strafe.
        - Use move() or build the move_cmd yourself.
        """
        self.move(0.0, 1.0, 0.0)
        self.node.get_logger().info("Finished running move_left...")

    def move_right(self):
        """
        TODO: Implement moving Pupper to the right (translation).
        - Set an appropriate negative linear.y value for right strafe.
        - Use move() or create your own move_cmd.
        """
        self.move(0.0, -1.0, 0.0)
        self.node.get_logger().info("Finished running move_right...")

    def turn_left(self):
        """
        TODO: Implement turning Pupper left (rotation).
        - Set a positive angular.z value for left rotation.
        - Use move() or build your own move_cmd.
        """
        self.move(0.0, 0.0, 1.0)
        self.node.get_logger().info("Finished running turn_left...")

    def turn_right(self):
        """
        TODO: Implement turning Pupper right (rotation).
        - Set a negative angular.z value for right rotation.
        - Use move() or make your own Twist message.
        """
        self.move(0.0, 0.0, -1.0)
        self.node.get_logger().info("Finished running turn_right...")

    def bark(self):
        self.node.get_logger().info('Bark...')
        pygame.mixer.init()

        # Directory-independent path to sound file
        # Get the directory of this file, then navigate to sounds directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
        bark_sound_path = os.path.join(sounds_dir, 'dog_bark.wav')

        bark_sound_path = os.path.normpath(bark_sound_path)
        bark_sound = pygame.mixer.Sound(bark_sound_path)
        bark_sound.play()
        self.node.get_logger().info(f'Playing bark sound from: {bark_sound_path}')
        self.stop()

    def _filter_text_with_gpt(self, text: str) -> str:
        """
        Filter text through ChatGPT to ensure it's appropriate for a robot dog to say.

        Args:
            text: The original text to filter

        Returns:
            Filtered text, or empty string if content should be blocked
        """
        try:
            from openai import OpenAI

            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                self.node.get_logger().warning('No OpenAI API key for filtering, allowing text')
                return text

            client = OpenAI(api_key=api_key)

            filter_prompt = """You are a content filter for a robot dog named Pupper that speaks to live audiences including children.

Your job is to check if the given text is appropriate for Pupper to say out loud.

Rules:
- Block profanity, slurs, hate speech, or inappropriate content
- Block anything sexual, violent, or harmful
- Block personal attacks or bullying
- Allow friendly, playful, and wholesome messages
- If the text is inappropriate, respond with exactly: BLOCKED
- If the text is appropriate, respond with the original text (you may clean up minor issues)
- Keep responses short and dog-friendly

Text to check:"""

            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": filter_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=1000,
            )

            # Extract response
            result = response.choices[0].message.content.strip()

            if result.upper() == "BLOCKED" or "BLOCKED" in result.upper():
                self.node.get_logger().warning(f'Content blocked by filter: {text}')
                return ""

            self.node.get_logger().info(f'Content approved: {result}')
            return result

        except Exception as e:
            self.node.get_logger().error(f'Content filter error: {e}, allowing original text')
            return text  # On error, allow the text through

    def say(self, text: str):
        """
        Speak the given text using ElevenLabs TTS with pyttsx3 fallback.
        Text is filtered through ChatGPT before being spoken.

        Args:
            text: The text to speak
        """
        self.node.get_logger().info(f'Say request: {text}')

        # Filter text through ChatGPT
        filtered_text = self._filter_text_with_gpt(text)
        if not filtered_text:
            self.node.get_logger().info('Text was blocked by content filter')
            return

        self.node.get_logger().info(f'Speaking: {filtered_text}')

        audio_file = None

        # Try ElevenLabs first
        api_key = os.getenv('ELEVENLABS_API_KEY')
        voice_id = os.getenv('ELEVENLABS_VOICE_ID', '21m00Tcm4TlvDq8ikWAM')
        self.node.get_logger().info(f'ElevenLabs API key set: {bool(api_key)}, Voice ID: {voice_id}')

        if api_key:
            try:
                import requests

                url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}"
                headers = {
                    "xi-api-key": api_key,
                    "Content-Type": "application/json"
                }
                data = {
                    "text": filtered_text,
                    "model_id": "eleven_monolingual_v1",
                    "voice_settings": {
                        "stability": 0.5,
                        "similarity_boost": 0.5
                    }
                }

                self.node.get_logger().info(f'Calling ElevenLabs API...')
                response = requests.post(url, json=data, headers=headers, timeout=10)
                self.node.get_logger().info(f'ElevenLabs response: {response.status_code}')

                if response.status_code == 200:
                    # Save to temp file
                    with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
                        f.write(response.content)
                        audio_file = f.name
                    self.node.get_logger().info(f'Using ElevenLabs TTS, saved to {audio_file}')
                else:
                    self.node.get_logger().warning(f'ElevenLabs API error: {response.status_code} - {response.text[:200]}')
            except Exception as e:
                self.node.get_logger().warning(f"ElevenLabs failed: {type(e).__name__}: {e}")
        else:
            self.node.get_logger().info('No ELEVENLABS_API_KEY set, skipping ElevenLabs')

        # Fallback to pyttsx3
        if audio_file is None:
            self.node.get_logger().info('Trying pyttsx3 fallback...')
            try:
                import pyttsx3
                self.node.get_logger().info(f'pyttsx3 module: {pyttsx3}, file: {getattr(pyttsx3, "__file__", "unknown")}')
                engine = pyttsx3.init()
                self.node.get_logger().info('pyttsx3 engine created')
                engine.say(filtered_text)
                engine.runAndWait()
                self.node.get_logger().info('Using pyttsx3 fallback TTS')
                return  # pyttsx3 plays directly, no file needed
            except Exception as e:
                self.node.get_logger().error(f"pyttsx3 fallback failed: {type(e).__name__}: {e}")
                # Try espeak as last resort on Linux/Pi
                self.node.get_logger().info('Trying espeak as last resort...')
                try:
                    import subprocess
                    subprocess.run(['espeak', filtered_text], check=True, timeout=10)
                    self.node.get_logger().info('Used espeak fallback')
                    return
                except Exception as espeak_err:
                    self.node.get_logger().error(f"espeak also failed: {espeak_err}")
                return

        # Play ElevenLabs audio file with pygame
        if audio_file:
            try:
                pygame.mixer.init()
                pygame.mixer.music.load(audio_file)
                pygame.mixer.music.play()
                # Wait for playback to finish
                while pygame.mixer.music.get_busy():
                    time.sleep(0.1)
                self.node.get_logger().info('Finished speaking')
            except Exception as e:
                self.node.get_logger().error(f"Audio playback failed: {e}")
            finally:
                # Clean up temp file
                try:
                    os.unlink(audio_file)
                except:
                    pass
    
    def dance(self):
        self.node.get_logger().info('Rick Rolling...')
        pygame.mixer.init()
        # Directory-independent path to sound file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sounds_dir = os.path.join(current_dir, '..', '..', 'sounds')
        dance_sound_path = os.path.join(sounds_dir, 'rickroll.wav')

        dance_sound_path = os.path.normpath(dance_sound_path)
        dance_sound = pygame.mixer.Sound(dance_sound_path)
        self.node.get_logger().info(f'Playing dance sound from: {dance_sound_path}')
        dance_sound.play()
        # TODO: Create your own awesome Pupper dance move sequence here!
        # Use combinations of self.wiggle(), self.turn_left(), self.turn_right(), self.bob(), and self.stop().
        # Be creative and choreograph the most exciting dance possible!
        self.wiggle()
        self.turn_left()
        self.turn_right()
        self.bob()
        self.turn_left()
        self.turn_right()
        self.wiggle()


    def stop(self):
        self.node.get_logger().info('Stopping...')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
    
    def __del__(self):
        self.node.get_logger().info('Tearing down...')
        self.node.destroy_node()
        rclpy.shutdown()

