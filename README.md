# Pupper Live
Bahram Mohmand, Victor Chen, Sonnet Xu, Issa Sadamoto

An interactive robotic dog platform with advanced AI capabilities including voice control, computer vision, and Twitch chat integration.

https://docs.google.com/presentation/d/1L-3QQ5CKZq2rWe0neAvxAw50_1ctcW2dSrgw9Fe6UWM/edit?usp=drive_link

[![Watch the demo](https://img.youtube.com/vi/WkV3JSaMH1Y/0.jpg)](https://www.youtube.com/watch?v=WkV3JSaMH1Y)

## Setup

### Prerequisites

- Raspberry Pi (for robot control)
- Python 3.x
- ROS2 (Robot Operating System)
- OpenAI API key

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/VictorChenCA/StanfordPupper. git
```

2. **Install dependencies**
```bash
pip install -r requirements.txt
```

3. **Set up API keys**
```bash
cd Pupper_lab_7
python setup_api_keys.py
```

Follow the prompts to securely configure your OpenAI API key.

## Project Structure

```
StanfordPupper/
├── Pupper_lab_7/          # Main robot control system
│   ├── config/            # API key configuration
│   ├── pupper_llm/        # LLM integration for voice control
│   │   └── karel/         # Karel commander for robot actions
│   └── scripts/           # Launch scripts
│
├── twitch/                # Twitch chat integration
│   ├── message_grabber.py     # Twitch message handler
│   ├── chat_processor.py      # Command extraction
│   ├── bucket_voting.py       # Democratic voting system
│   └── command_executor.py    # Command execution
│
└── requirements.txt       # Python dependencies
```

## Running Pupper Live

Launch everything with one command: 

```bash
cd Pupper_lab_7/scripts/
./launch_with_twitch.sh
```

This starts:
- Robot control system
- Camera and vision
- Voice control
- Twitch chat integration

## Interacting with Pupper

**Action Commands (use `!` prefix):**
- `!dance` - 12-second dance routine
- `!move forward` / `!walk forward`
- `!turn left` / `!turn right`
- `!say Hello` - Repeats "Hello"
- `!wiggle` / `!bob` / `!bark`

**Conversations (use `@` prefix):**
- `@what's your favorite color?`
- `@how are you feeling? `
- `@tell me about yourself`


## Configuration

### API Keys Setup

```bash
cd Pupper_lab_7
python setup_api_keys.py
```

Or manually edit `config/api_keys.py`:

```python
OPENAI_API_KEY = "sk-your-api-key-here"
GPT_MODEL = "gpt-4"
WHISPER_MODEL = "whisper-1"
TTS_MODEL = "tts-1"
TTS_VOICE = "alloy"
```

### Twitch Configuration

Copy and edit the environment file:

```bash
cd twitch/
cp .env. example .env
nano .env
```

Required credentials:
- `CLIENT_ID` - From https://dev.twitch.tv/console/apps
- `SECRET_ID` - Twitch app secret
- `BOT_ID` - Your Twitch user ID
- `OPENAI_API_KEY` - From https://platform.openai.com/api-keys


## Documentation

- **[Twitch Integration Guide](twitch/README.md)** - Detailed Twitch setup
- **[Voting System Mechanics](twitch/VOTING_SYSTEM.md)** - How voting works
- **[API Configuration](Pupper_lab_7/config/README.md)** - Secure key setup

## Contributing

Contributions welcome! Feel free to:
- Report issues
- Suggest features
- Submit pull requests
- Share your setup and modifications
