# Quick Start Guide - Twitch Chat Control for Pupper

## NEW: Bucket Voting System! 🗳️

**Your chat now votes for commands democratically!**
- Each `!pupper` message = 30 second vote for that command
- Most popular command wins and executes when robot finishes
- Votes decay over time (keeps it dynamic)
- One vote per user per execution cycle

**See `VOTING_SYSTEM.md` for full technical details.**

---

## 5-Minute Setup

### 1. Install Packages
```bash
cd twitch/

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate  # macOS/Linux

# Install dependencies
pip install openai twitchio asqlite python-dotenv
```

### 2. Create Config File
```bash
cp .env.example .env
nano .env  # Add your API keys
```

Required in `.env`:
- `CLIENT_ID` - From https://dev.twitch.tv/console
- `SECRET_ID` - From Twitch dev console
- `BOT_ID` - Your Twitch user ID
- `OPENAI_API_KEY` - From https://platform.openai.com/api-keys

### 3. Verify Setup
```bash
python check_setup.py
```

Fix any errors before continuing!

### 4. Launch Streaming Mode
```bash
cd ../Pupper_lab_7/scripts/
./launch_with_twitch.sh
```

**This starts EVERYTHING:**
- ✅ Robot control
- ✅ Camera/vision
- ✅ Voice control
- ✅ Twitch integration

**Or run just the Twitch bot:**
```bash
cd twitch/
python message_grabber.py
```

### 5. Test in Twitch Chat
Go live on Twitch, then have viewers type:
```
!pupper walk forward
!pupper dance
!pupper follow that person
```

---

## How It Saves Performance

### Without Optimization (❌ Bad)
```
100 chat messages/min → 100 API calls → $$ cost + Pi overload
```

### With This Implementation (✅ Good)
```
100 chat messages/min
  → Filter by !pupper → 10 command messages
  → Batch for 3 seconds → 3 API calls/min
  → Pattern match 50% → 1.5 API calls/min

Result: 90 API calls/hour vs 6000 API calls/hour
Cost: ~$0.002/hour vs ~$0.12/hour
```

---

## Architecture Comparison

### Voice System (realtime_voice.py)
```
Microphone → Persistent WebSocket → OpenAI Realtime API → Audio Response
                                          ↓
                                    ROS2 → Pupper
```
- **Connection:** 1 persistent WebSocket (efficient for audio)
- **Processing:** Server-side VAD triggers processing
- **Cost:** Based on audio streaming time

### Twitch System (this integration)
```
Twitch Chat → Filter → Batch → Pattern Match → OpenAI Chat API → Commands
                                      ↓                ↓
                                    (skip API)      ROS2 → Pupper
```
- **Connection:** REST API calls (efficient for text)
- **Processing:** Batched, filtered, pattern-matched first
- **Cost:** Per API call (minimized through batching)

**Both systems can run simultaneously** - they use the same ROS2 topic!

---

## Performance Metrics

### Message Batching
- **Interval:** 3 seconds (configurable)
- **Reduction:** ~10x fewer API calls

### Pattern Matching
Commands matched locally (no API call):
- `sit`, `stand`, `walk`, `turn left/right`
- `move forward/backward/left/right`
- `follow`, `track`, `stop`
- `dance`, `wiggle`, `bob`, `bark`

Complex/ambiguous commands use LLM:
- "can you walk forward and then turn left?"
- "follow that cute dog!"

### Rate Limiting
- **Default:** 12 requests/minute
- **Prevents:** API overload, budget overruns, Pi stress

### API Model
- **Using:** `gpt-4o-mini` (cheap and fast)
- **Not using:** `gpt-4` or Realtime API (too expensive)

---

## Configuration Options

Edit `.env` to customize:

```bash
# Change command prefix
PUPPER_COMMAND_PREFIX=!robot

# Adjust batch timing (higher = fewer API calls)
PUPPER_BATCH_INTERVAL=5.0

# Adjust rate limit (lower = more conservative)
PUPPER_MAX_REQUESTS_PER_MINUTE=6
```

---

## Running Both Voice and Chat

```bash
# Terminal 1: Voice control
cd Pupper_lab_7/pupper_llm
python realtime_voice.py

# Terminal 2: Twitch chat
cd twitch
python message_grabber.py

# Terminal 3: Command executor (handles both!)
cd Pupper_lab_7/pupper_llm/karel
python karel_realtime_commander.py
```

Both systems publish to `gpt4_response_topic` → Karel processes all commands.

---

## Troubleshooting One-Liners

```bash
# Check if ROS2 is receiving commands
ros2 topic echo /gpt4_response_topic

# Test chat processor without Twitch
cd twitch
python chat_processor.py

# Check OpenAI API key
python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('OK' if os.getenv('OPENAI_API_KEY') else 'Missing')"

# Monitor API usage
# Watch the console output - it logs every API call made
```

---

## Cost Calculator

**Scenario:** 1-hour Twitch stream, 100 active viewers

- Commands per minute: ~10
- After batching (3s): ~3-4 API calls/min
- After pattern matching (50%): ~1.5-2 API calls/min
- Total API calls: ~100/hour
- Average tokens: 50 input + 20 output
- **Cost:** ~$0.002/hour (less than a penny)

For comparison:
- Per-message processing: ~$0.12/hour (60x more expensive)
- Coffee: ~$5.00 (2500x more expensive than 1 hour of streaming)

---

## Key Takeaways

1. **Per-message API calls are BAD** ❌
   - Expensive, slow, overloads Pi

2. **This implementation is GOOD** ✅
   - Batches messages
   - Filters by prefix
   - Pattern matches first
   - Rate limits properly

3. **Raspberry Pi can handle it** ✅
   - ~2 API calls/minute average
   - Async processing (non-blocking)
   - Graceful fallbacks

4. **Cost is negligible** ✅
   - Fractions of a penny per hour
   - Much cheaper than alternatives

5. **Works alongside voice** ✅
   - Same ROS2 integration
   - No conflicts
   - Unified command processing

---

## Next Steps

1. **Test locally** without ROS2 (will log commands but not execute)
2. **Deploy to Raspberry Pi** with full ROS2 integration
3. **Monitor performance** during a test stream
4. **Adjust settings** based on chat activity
5. **Go live** and let chat control Pupper! 🐕🤖

See `README.md` for full documentation.
