# API Configuration

## 🔐 Security Notice

**IMPORTANT**: Never commit `api_keys.py` to version control!

This directory contains API key configuration for the Pupper LLM system.

---

## Quick Setup

### Option 1: Automated Setup (Recommended)

Run the setup script from the lab_7 root directory:

```bash
cd /home/pi/lab_7_fall_2025_solutions
python setup_api_keys.py
```

This will guide you through:
1. Creating your config file
2. Entering your API key securely
3. Validating the configuration

### Option 2: Manual Setup

1. **Copy the template**:
   ```bash
   cp config/api_keys_template.py config/api_keys.py
   ```

2. **Edit the file**:
   ```bash
   nano config/api_keys.py
   ```

3. **Replace the placeholder**:
   ```python
   OPENAI_API_KEY = "sk-your-actual-key-here"
   ```

4. **Save and test**:
   ```bash
   python config/api_keys.py
   ```

### Option 3: Environment Variable

Set the environment variable (doesn't require editing files):

```bash
export OPENAI_API_KEY="sk-your-actual-key-here"
```

Add to `~/.bashrc` to make it permanent:
```bash
echo 'export OPENAI_API_KEY="sk-your-actual-key-here"' >> ~/.bashrc
source ~/.bashrc
```

---

## Files in This Directory

| File | Purpose | Commit to Git? |
|------|---------|----------------|
| `api_keys_template.py` | Template with placeholders | ✅ YES - Safe to commit |
| `api_keys.py` | Your actual API keys | ❌ NO - Protected by .gitignore |
| `README.md` | This file | ✅ YES |

---

## Security Features

### Protected by .gitignore

The `.gitignore` file contains multiple rules to protect your keys:

```gitignore
# API Keys - NEVER commit these!
config/api_keys.py
**/api_keys.py
*.env
.env
```

Even if you run `git add .`, the API keys won't be added!

### Validation

The config file includes validation:
- Checks if key starts with `sk-`
- Checks minimum length
- Falls back to environment variables
- Provides clear error messages

---

## Getting Your OpenAI API Key

1. Go to: https://platform.openai.com/api-keys
2. Sign in or create an account
3. Click "Create new secret key"
4. Copy the key (starts with `sk-`)
5. Use it in the setup script or config file

⚠️ **Store it safely** - you won't be able to see it again!

---

## Testing Your Configuration

```bash
# Test the config file directly
python config/api_keys.py

# Or test with the setup script
python setup_api_keys.py
```

Expected output:
```
✓ OpenAI API key is properly configured
✓ Key format: sk-proj-ab...xyz123
✓ GPT Model: gpt-4
✓ Whisper Model: whisper-1
✓ TTS Model: tts-1
✓ TTS Voice: alloy
```

---

## Troubleshooting

### "No OpenAI API key found"

**Solution**: Your key isn't configured. Run:
```bash
python setup_api_keys.py
```

### "API key doesn't start with 'sk-'"

**Solution**: You copied the wrong thing. API keys always start with `sk-`

### Import errors in realtime_voice.py

**Solution**: Make sure `config/api_keys.py` exists:
```bash
ls -la config/api_keys.py
```

If not found, run `python setup_api_keys.py`

---

## Configuration Options

Edit `config/api_keys.py` to customize:

```python
# Model Configuration
GPT_MODEL = "gpt-4"              # Or "gpt-4-turbo", "gpt-3.5-turbo"
WHISPER_MODEL = "whisper-1"      # Speech-to-text model
TTS_MODEL = "tts-1"              # Text-to-speech (or "tts-1-hd")
TTS_VOICE = "alloy"              # Voice style

# API Settings
MAX_TOKENS = 150                 # Max response length
TEMPERATURE = 0.7                # Randomness (0.0-2.0)
TIMEOUT_SECONDS = 30             # API timeout
```

---

## Security Best Practices

1. ✅ **Never share** your API keys
2. ✅ **Never commit** api_keys.py (protected by .gitignore)
3. ✅ **Rotate keys** regularly
4. ✅ **Use environment variables** in production
5. ✅ **Set spending limits** in OpenAI dashboard
6. ✅ **Monitor usage** regularly

---

## For Students/Teaching

If distributing this code:

1. ✅ **Include**: `api_keys_template.py`
2. ✅ **Include**: `setup_api_keys.py`
3. ✅ **Include**: This `README.md`
4. ❌ **NEVER include**: `api_keys.py` (your actual keys!)

Students should:
1. Clone the repository
2. Run `python setup_api_keys.py`
3. Enter their own API key
4. Start using the system

The `.gitignore` will protect their keys from accidental commits!

---

## Verification

After setup, verify the protection is working:

```bash
# This should show api_keys.py as ignored
git status --ignored

# This should NOT show api_keys.py
git add .
git status

# If it shows up, check your .gitignore
cat .gitignore | grep api_keys
```

---

**Your API keys are now protected!** 🔐


