import os
from dotenv import load_dotenv
from twitchio.ext import commands

load_dotenv()

class Bot(commands.Bot):
    def __init__(self):
        token = os.getenv('TWITCH_OAUTH_TOKEN')
        
        if not token:
            raise ValueError("❌ TWITCH_OAUTH_TOKEN not found in .env file")
        
        print(f"🔐 Token loaded: {token[:10]}...")
        
        super().__init__(
            token=token,
            prefix='!',
            initial_channels=['stanford_pupper']
        )

    async def event_ready(self):
        """Called when the bot is ready"""
        print(f'✅ Logged in as: {self.nick}')
        print(f'🎉 Connected to channels: {self.connected_channels}')

    async def event_message(self, message):
        """Called when a message is received"""
        if message.echo:
            return
        
        print(f'{message.author.name}: {message.content}')
        await self.handle_commands(message)

    async def event_channel_joined(self, channel):
        """Called when the bot joins a channel"""
        print(f'🎊 Joined channel: {channel.name}')

    @commands.command(name='hello')
    async def hello_command(self, ctx):
        await ctx.send(f'Hello @{ctx.author.name}! 👋')

if __name__ == "__main__":
    print("🚀 Starting Twitch bot...")
    bot = Bot()
    bot.run()