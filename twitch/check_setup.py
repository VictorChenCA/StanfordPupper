#!/usr/bin/env python3
"""
Twitch Integration Setup Checker
Verifies all prerequisites are met before launching the Twitch bot.
"""

import os
import sys
from pathlib import Path

# ANSI color codes for pretty output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"
CHECK = "✓"
CROSS = "✗"
WARN = "⚠"


def print_header(text):
    """Print a section header."""
    print(f"\n{BLUE}{'=' * 60}{RESET}")
    print(f"{BLUE}{text}{RESET}")
    print(f"{BLUE}{'=' * 60}{RESET}")


def print_success(text):
    """Print a success message."""
    print(f"{GREEN}{CHECK} {text}{RESET}")


def print_error(text):
    """Print an error message."""
    print(f"{RED}{CROSS} {text}{RESET}")


def print_warning(text):
    """Print a warning message."""
    print(f"{YELLOW}{WARN} {text}{RESET}")


def check_env_file():
    """Check if .env file exists and has required variables."""
    print_header("Checking .env Configuration")

    env_path = Path(__file__).parent / ".env"

    if not env_path.exists():
        print_error(".env file not found")
        print(f"  → Expected location: {env_path}")
        print(f"  → Run: cp .env.example .env")
        return False

    print_success(".env file exists")

    # Check required variables
    required_vars = {
        "CLIENT_ID": "Twitch Client ID",
        "SECRET_ID": "Twitch Secret",
        "BOT_ID": "Twitch Bot/User ID",
        "OPENAI_API_KEY": "OpenAI API Key",
    }

    missing_vars = []
    empty_vars = []

    # Load .env file
    with open(env_path) as f:
        env_content = f.read()

    for var, description in required_vars.items():
        if var not in env_content:
            missing_vars.append((var, description))
        else:
            # Check if it has a value
            for line in env_content.split('\n'):
                if line.startswith(var):
                    value = line.split('=', 1)[1].strip() if '=' in line else ''
                    if not value or value.startswith('your_') or value.startswith('sk-your'):
                        empty_vars.append((var, description))
                        break
                    else:
                        print_success(f"{var} is set")
                        break

    if missing_vars:
        print_error("Missing required variables:")
        for var, desc in missing_vars:
            print(f"  → {var} ({desc})")
        return False

    if empty_vars:
        print_error("Empty or placeholder variables:")
        for var, desc in empty_vars:
            print(f"  → {var} ({desc})")
        print("\nPlease edit .env and add your actual credentials")
        return False

    return True


def check_python_packages():
    """Check if required Python packages are installed."""
    print_header("Checking Python Dependencies")

    required_packages = {
        "twitchio": "TwitchIO",
        "asqlite": "asqlite",
        "openai": "OpenAI",
        "dotenv": "python-dotenv",
    }

    missing_packages = []

    for package, name in required_packages.items():
        try:
            if package == "dotenv":
                __import__("dotenv")
            else:
                __import__(package)
            print_success(f"{name} is installed")
        except ImportError:
            missing_packages.append(name)
            print_error(f"{name} is NOT installed")

    if missing_packages:
        print(f"\n{YELLOW}Install missing packages with:{RESET}")
        print(f"  pip install {' '.join(missing_packages.lower())}")
        return False

    return True


def check_ros2():
    """Check if ROS2 is available (optional)."""
    print_header("Checking ROS2 (Optional)")

    try:
        import rclpy
        print_success("ROS2 (rclpy) is available")
        print("  → Commands will be published to Pupper")
        return True
    except ImportError:
        print_warning("ROS2 (rclpy) is NOT available")
        print("  → Bot will run in chat-only mode (no robot control)")
        print("  → This is OK for testing")
        return True  # Not a failure, just a warning


def check_twitch_auth():
    """Check if Twitch authentication tokens exist."""
    print_header("Checking Twitch Authentication")

    tokens_db = Path(__file__).parent / "tokens.db"

    if not tokens_db.exists():
        print_warning("No authentication tokens found")
        print("  → This is normal for first-time setup")
        print("  → A browser will open for authentication when you run the bot")
        return True

    print_success("Authentication tokens found")
    print("  → You're already authenticated with Twitch")
    return True


def check_file_structure():
    """Check if all required files exist."""
    print_header("Checking File Structure")

    base_path = Path(__file__).parent

    required_files = {
        "message_grabber.py": "Main bot script",
        "chat_processor.py": "Command processor",
        "bucket_voting.py": "Voting system",
        "command_executor.py": "Command executor",
    }

    missing_files = []

    for file, description in required_files.items():
        file_path = base_path / file
        if file_path.exists():
            print_success(f"{file} ({description})")
        else:
            missing_files.append(file)
            print_error(f"{file} is MISSING ({description})")

    if missing_files:
        print("\n" + RED + "ERROR: Critical files are missing!" + RESET)
        return False

    return True


def print_summary(all_checks_passed, warnings):
    """Print final summary."""
    print_header("Setup Check Summary")

    if all_checks_passed:
        print(f"\n{GREEN}{'=' * 60}")
        print(f"  {CHECK} All checks passed! You're ready to go!")
        print(f"{'=' * 60}{RESET}\n")

        if warnings:
            print(f"{YELLOW}Notes:{RESET}")
            for warning in warnings:
                print(f"  {WARN} {warning}")
            print()

        print(f"{BLUE}To start the Twitch bot:{RESET}")
        print(f"  python message_grabber.py")
        print()
        print(f"{BLUE}Or use the full launcher (if on Raspberry Pi):{RESET}")
        print(f"  cd ../Pupper_lab_7/scripts/")
        print(f"  ./launch_with_twitch.sh")
        print()
    else:
        print(f"\n{RED}{'=' * 60}")
        print(f"  {CROSS} Setup incomplete - please fix errors above")
        print(f"{'=' * 60}{RESET}\n")

        print(f"{YELLOW}Quick Fix Guide:{RESET}")
        print(f"  1. Copy config:  cp .env.example .env")
        print(f"  2. Edit config:  nano .env")
        print(f"  3. Add your Twitch credentials from https://dev.twitch.tv/console")
        print(f"  4. Add your OpenAI API key from https://platform.openai.com/api-keys")
        print(f"  5. Install packages:  pip install twitchio asqlite openai python-dotenv")
        print(f"  6. Run this check again:  python check_setup.py")
        print()


def main():
    """Run all checks."""
    print(f"\n{BLUE}╔══════════════════════════════════════════════════════════╗")
    print(f"║  Twitch Integration Setup Checker                       ║")
    print(f"║  Stanford Pupper Robot                                  ║")
    print(f"╚══════════════════════════════════════════════════════════╝{RESET}\n")

    checks = []
    warnings = []

    # Run all checks
    checks.append(("File Structure", check_file_structure()))
    checks.append(("Python Packages", check_python_packages()))
    checks.append((".env Configuration", check_env_file()))
    checks.append(("Twitch Auth", check_twitch_auth()))

    # ROS2 check (warning only)
    if not check_ros2():
        warnings.append("ROS2 not available - bot will run in chat-only mode")

    # Determine overall status
    all_checks_passed = all(result for _, result in checks)

    # Print summary
    print_summary(all_checks_passed, warnings)

    # Exit with appropriate code
    sys.exit(0 if all_checks_passed else 1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}Check cancelled by user{RESET}\n")
        sys.exit(130)
    except Exception as e:
        print(f"\n{RED}Unexpected error: {e}{RESET}\n")
        sys.exit(1)
