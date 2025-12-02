#!/usr/bin/env python3
"""
API Key Configuration Template for Pupper LLM System

INSTRUCTIONS:
1. Copy this file to api_keys.py
2. Replace the placeholder values with your actual API keys
3. Never commit api_keys.py to version control

Security Notes:
- Keep your API keys secure and don't share them
- Consider using environment variables for production deployments
- Rotate keys regularly for security
"""

import os
from typing import Optional

# OpenAI API Configuration
# Replace with your actual OpenAI API key
OPENAI_API_KEY = "your-openai-api-key-here"

# ElevenLabs API Configuration (for high-quality TTS)
# Get your API key from: https://elevenlabs.io/
ELEVENLABS_API_KEY = "your-elevenlabs-api-key-here"
ELEVENLABS_VOICE_ID = "21m00Tcm4TlvDq8ikWAM"  # Default: Rachel voice

# Model Configuration
GPT_MODEL = "gpt-4"
WHISPER_MODEL = "whisper-1"
TTS_MODEL = "tts-1"
TTS_VOICE = "alloy"  # Options: alloy, echo, fable, onyx, nova, shimmer

# API Settings
MAX_TOKENS = 150
TEMPERATURE = 0.7
TIMEOUT_SECONDS = 30

def get_openai_api_key() -> str:
    """
    Get OpenAI API key with fallback to environment variable.
    
    Returns:
        str: The OpenAI API key
        
    Raises:
        ValueError: If no API key is found
    """
    # First try environment variable (for production/security)
    env_key = os.getenv('OPENAI_API_KEY')
    if env_key:
        return env_key
    
    # Fallback to hardcoded key
    if OPENAI_API_KEY and OPENAI_API_KEY != "your-openai-api-key-here":
        return OPENAI_API_KEY
    
    raise ValueError(
        "No OpenAI API key found. Please:\n"
        "1. Set OPENAI_API_KEY environment variable, or\n"
        "2. Update OPENAI_API_KEY in config/api_keys.py"
    )

def get_openai_client():
    """
    Get configured OpenAI client instance.

    Returns:
        OpenAI: Configured OpenAI client
    """
    from openai import OpenAI
    return OpenAI(api_key=get_openai_api_key())


def get_elevenlabs_api_key() -> Optional[str]:
    """
    Get ElevenLabs API key with fallback to environment variable.

    Returns:
        str or None: The ElevenLabs API key, or None if not configured
    """
    # First try environment variable (for production/security)
    env_key = os.getenv('ELEVENLABS_API_KEY')
    if env_key:
        return env_key

    # Fallback to hardcoded key
    if ELEVENLABS_API_KEY and ELEVENLABS_API_KEY != "your-elevenlabs-api-key-here":
        return ELEVENLABS_API_KEY

    return None  # ElevenLabs is optional, return None if not configured


def get_elevenlabs_voice_id() -> str:
    """
    Get ElevenLabs voice ID with fallback to environment variable.

    Returns:
        str: The ElevenLabs voice ID
    """
    return os.getenv('ELEVENLABS_VOICE_ID', ELEVENLABS_VOICE_ID)

def validate_api_key() -> bool:
    """
    Validate that the API key is properly configured.
    
    Returns:
        bool: True if API key is valid format, False otherwise
    """
    try:
        key = get_openai_api_key()
        return key.startswith('sk-') and len(key) > 20
    except ValueError:
        return False

# Configuration validation on import
if __name__ == "__main__":
    print("=== API Key Configuration Test ===")

    if validate_api_key():
        print("✓ OpenAI API key is properly configured")
        key = get_openai_api_key()
        print(f"✓ Key format: {key[:10]}...{key[-10:]}")
    else:
        print("✗ OpenAI API key is not properly configured")
        print("Please check your configuration in config/api_keys.py")

    elevenlabs_key = get_elevenlabs_api_key()
    if elevenlabs_key:
        print("✓ ElevenLabs API key is configured")
        print(f"✓ ElevenLabs Voice ID: {get_elevenlabs_voice_id()}")
    else:
        print("○ ElevenLabs API key not configured (will use pyttsx3 fallback)")

    print(f"✓ GPT Model: {GPT_MODEL}")
    print(f"✓ Whisper Model: {WHISPER_MODEL}")
    print(f"✓ TTS Model: {TTS_MODEL}")
    print(f"✓ TTS Voice: {TTS_VOICE}")
    print("=== Configuration Test Complete ===")


