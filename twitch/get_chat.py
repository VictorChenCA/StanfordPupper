import irc.client
import os
from dotenv import load_dotenv

load_dotenv()

USERNAME = "stanford_pupper"
TOKEN = os.getenv('TWITCH_OAUTH_TOKEN')  # Don't add oauth: prefix here
CHANNEL = f"#{USERNAME}"

def on_connect(connection, event):
    print("✅ Connected to Twitch IRC")
    # Request Twitch-specific capabilities
    connection.cap("REQ", ":twitch.tv/membership")
    connection.cap("REQ", ":twitch.tv/tags")
    connection.cap("REQ", ":twitch.tv/commands")
    print("📡 Requested Twitch capabilities")
    connection.join(CHANNEL)

def on_join(connection, event):
    print(f"🎉 Joined channel {CHANNEL}")

def on_pubmsg(connection, event):
    sender = event.source.nick
    message = event.arguments[0]
    print(f"{sender}: {message}")

def on_disconnect(connection, event):
    print("❌ Disconnected")
    print(f"   Reason: {event.arguments}")

def on_error(connection, event):
    print(f"⚠️ ERROR: {event.arguments}")

# Create IRC client
client = irc.client.Reactor()

print(f"🔐 Connection info:")
print(f"   Username: {USERNAME}")
print(f"   Token loaded: {bool(TOKEN)}")
print(f"   Token format: oauth:{TOKEN[:10]}..." if TOKEN else "   Token: MISSING")

try:
    # Connect with oauth: prefix
    conn = client.server().connect(
        "irc.chat.twitch.tv", 
        6667, 
        USERNAME, 
        f"oauth:{TOKEN}"
    )
except Exception as e:
    print(f"❌ Connection failed: {e}")
    exit(1)

# Register handlers
conn.add_global_handler("welcome", on_connect)
conn.add_global_handler("join", on_join)
conn.add_global_handler("pubmsg", on_pubmsg)
conn.add_global_handler("disconnect", on_disconnect)
conn.add_global_handler("error", on_error)

print("🎧 Listening for messages...")
client.process_forever()