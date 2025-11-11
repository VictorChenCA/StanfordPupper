import irc.client
import os

USERNAME = "stanford_pupper"
TOKEN = f"oauth:{os.environ['TWITCH_OAUTH_TOKEN']}"
CHANNEL = f"#{USERNAME}"  # Twitch channels are prefixed with #

def on_connect(connection, event):
    print("✅ Connected to Twitch IRC")
    connection.join(CHANNEL)

def on_join(connection, event):
    print(f"🎉 Joined channel {CHANNEL}")

def on_pubmsg(connection, event):
    sender = event.source.nick
    message = event.arguments[0]
    print(f"{sender}: {message}")

def on_disconnect(connection, event):
    print("❌ Disconnected")

# Create IRC client and connect
client = irc.client.Reactor()
try:
    conn = client.server().connect("irc.chat.twitch.tv", 6667, USERNAME, password=TOKEN)
except irc.client.ServerConnectionError as e:
    print(f"Connection error: {e}")
    exit(1)

# Register event handlers
conn.add_global_handler("welcome", on_connect)
conn.add_global_handler("join", on_join)
conn.add_global_handler("pubmsg", on_pubmsg)
conn.add_global_handler("disconnect", on_disconnect)

# Start listening
client.process_forever()