#!/usr/bin/env python3
"""
API Key Setup Script for Pupper LLM System
This script helps users configure their OpenAI API keys securely.
"""

import os
import shutil
import sys

def setup_api_keys():
    """Interactive setup for API keys."""
    print("=== Pupper LLM API Key Setup ===")
    print()
    
    config_dir = os.path.join(os.path.dirname(__file__), 'config')
    template_file = os.path.join(config_dir, 'api_keys_template.py')
    config_file = os.path.join(config_dir, 'api_keys.py')
    
    # Check if config file already exists
    if os.path.exists(config_file):
        print("API key configuration already exists.")
        response = input("Do you want to reconfigure? (y/N): ").strip().lower()
        if response != 'y':
            print("Setup cancelled.")
            return
    
    # Copy template to config file
    try:
        shutil.copy2(template_file, config_file)
        print(f"✓ Created config file: {config_file}")
    except Exception as e:
        print(f"✗ Error creating config file: {e}")
        return
    
    print()
    print("Configuration options:")
    print("1. Enter API key interactively (recommended)")
    print("2. Use environment variable OPENAI_API_KEY")
    print("3. Edit config file manually")
    
    choice = input("Choose option (1-3): ").strip()
    
    if choice == "1":
        # Interactive API key entry
        print()
        print("Please enter your OpenAI API key:")
        print("(You can get one from https://platform.openai.com/api-keys)")
        api_key = input("API Key: ").strip()
        
        if not api_key:
            print("✗ No API key provided. Setup cancelled.")
            return
        
        if not api_key.startswith('sk-'):
            print("⚠ Warning: API key doesn't start with 'sk-'. Please verify it's correct.")
        
        # Update the config file
        try:
            with open(config_file, 'r') as f:
                content = f.read()
            
            content = content.replace(
                'OPENAI_API_KEY = "your-openai-api-key-here"',
                f'OPENAI_API_KEY = "{api_key}"'
            )
            
            with open(config_file, 'w') as f:
                f.write(content)
            
            print("✓ API key configured successfully!")
            
        except Exception as e:
            print(f"✗ Error updating config file: {e}")
            return
    
    elif choice == "2":
        # Environment variable setup
        print()
        print("To use environment variable, run:")
        print("export OPENAI_API_KEY='your-api-key-here'")
        print()
        print("Add this to your ~/.bashrc to make it permanent.")
        
    elif choice == "3":
        # Manual editing
        print()
        print(f"Please edit the file: {config_file}")
        print("Replace 'your-openai-api-key-here' with your actual API key.")
        
    else:
        print("Invalid choice. Setup cancelled.")
        return
    
    # Test the configuration
    print()
    print("Testing configuration...")
    
    try:
        sys.path.insert(0, config_dir)
        from api_keys import validate_api_key, get_openai_api_key
        
        if validate_api_key():
            key = get_openai_api_key()
            print(f"✓ API key validation successful: {key[:10]}...{key[-10:]}")
            print("✓ Setup complete! You can now run the Pupper LLM system.")
        else:
            print("✗ API key validation failed. Please check your configuration.")
            
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        print("Please check your API key configuration.")

def main():
    """Main entry point."""
    try:
        setup_api_keys()
    except KeyboardInterrupt:
        print("\n\nSetup interrupted by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")

if __name__ == "__main__":
    main()
