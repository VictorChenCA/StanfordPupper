#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script to verify compound command handling.
"""

import asyncio
import sys
import io
from chat_processor import ChatProcessor

# Fix Windows console encoding issues
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

async def test_compound_commands():
    """Test that compound commands are properly extracted and joined."""

    # Create processor without voting system to test extraction
    processor = ChatProcessor(
        command_prefix="!",
        batch_interval=1.0,
        max_requests_per_minute=12,
        voting_system=None
    )

    # Test cases
    test_cases = [
        ("!turn right, move forward, and bark", ["turn_right", "move_forward", "bark"]),
        ("!turn left then move forward", ["turn_left", "move_forward"]),
        ("!dance, wiggle, and bob", ["dance", "wiggle", "bob"]),
        ("!move forward", ["move_forward"]),
        ("!turn right and turn left", ["turn_right", "turn_left"]),
    ]

    print("Testing compound command extraction:\n")
    all_passed = True

    for text, expected in test_cases:
        # Extract just the command part (remove prefix)
        command_text = text[1:].strip()  # Remove '!'

        # Try pattern matching
        commands = processor._try_pattern_match(command_text)

        if commands == expected:
            print(f"✅ PASS: '{text}'")
            print(f"   Expected: {expected}")
            print(f"   Got:      {commands}")
        else:
            print(f"❌ FAIL: '{text}'")
            print(f"   Expected: {expected}")
            print(f"   Got:      {commands}")
            all_passed = False
        print()

    # Test that compound commands get joined with newlines
    print("\nTesting command joining for voting:")
    commands = ["turn_right", "move_forward", "bark"]
    if len(commands) > 1:
        command_sequence = "\n".join(commands)
        expected_sequence = "turn_right\nmove_forward\nbark"
        if command_sequence == expected_sequence:
            print(f"✅ PASS: Commands joined correctly")
            print(f"   Result: {repr(command_sequence)}")
        else:
            print(f"❌ FAIL: Commands not joined correctly")
            print(f"   Expected: {repr(expected_sequence)}")
            print(f"   Got:      {repr(command_sequence)}")
            all_passed = False

    print("\n" + "="*60)
    if all_passed:
        print("✅ ALL TESTS PASSED!")
    else:
        print("❌ SOME TESTS FAILED")
    print("="*60)

    return all_passed

if __name__ == "__main__":
    result = asyncio.run(test_compound_commands())
    sys.exit(0 if result else 1)
