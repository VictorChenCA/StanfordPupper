"""An example of connecting to a conduit and subscribing to EventSub when a User Authorizes the application.

This bot can be restarted as many times without needing to subscribe or worry about tokens:
- Tokens are stored in '.tio.tokens.json' by default
- Subscriptions last 72 hours after the bot is disconnected and refresh when the bot starts.

Consider reading through the documentation for AutoBot for more in depth explanations.
"""

"""comments from issa:
    This file requires no other files in the repo to run, aside from a .env file.
    line 123 is basically all that matters in terms of where the messages are extracted.
    If running this file on new device, initial setup/authentication is required. 
    
    We need to make a twitch application to get the user id and a secret. When making the application, make the callback URL
    http://localhost:4343/oauth/callback in order to authenticate easily.
    In particular, we need to get CLIENT_ID, SECRET_ID, and the twitch account's USER_ID and put it in an envfile. 
    Set another enviromental var BOT_ID as the same as USER_ID.
    Create a new twitch stream and go live.
    After that, follow authentication steps on https://twitchio.dev/en/latest/getting-started/quickstart.html (steps 4/5)
    The bot should be able to read all new messages after that!    
"""

import asyncio
import logging
from typing import TYPE_CHECKING

import asqlite

import twitchio
from twitchio import eventsub
from twitchio.ext import commands
from dotenv import load_dotenv
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


if TYPE_CHECKING:
    import sqlite3

load_dotenv()
LOGGER: logging.Logger = logging.getLogger("Bot")

# Add keys to an .env file in root folder!
# First-time setup on a new device requires some new authentication stuff, read the following link to see how to setup:
# https://twitchio.dev/en/latest/getting-started/quickstart.html

CLIENT_ID: str = os.getenv("CLIENT_ID")  # The CLIENT ID from the Twitch Dev Console
CLIENT_SECRET: str = os.getenv("SECRET_ID")  # The CLIENT SECRET from the Twitch Dev Console
BOT_ID = os.getenv("BOT_ID")  # The Account ID of the bot user...
OWNER_ID = BOT_ID  # We are using the streaming account to read stream messages so bot_id and owner_id are the same.


class Bot(commands.AutoBot):

    # DO NOT need to mess around with this for pupper
    def __init__(self, *, token_database: asqlite.Pool, subs: list[eventsub.SubscriptionPayload]) -> None:
        self.token_database = token_database

        super().__init__(
            client_id=CLIENT_ID,
            client_secret=CLIENT_SECRET,
            bot_id=BOT_ID,
            owner_id=OWNER_ID,
            prefix="!", #if we run commands this is probably what we can edit to change prefixes
            subscriptions=subs,
            force_subscribe=True,
        )

    # DO NOT need to mess around with this for pupper
    async def setup_hook(self) -> None:
        # Add our component which contains our commands for the bot!
        await self.add_component(MyComponent(self))
        print("added components")


    # DO NOT need to mess around with this for pupper
    async def event_oauth_authorized(self, payload: twitchio.authentication.UserTokenPayload) -> None:
        await self.add_token(payload.access_token, payload.refresh_token)

        if not payload.user_id:
            return

        # A list of subscriptions we would like to make to the newly authorized channel, we add new subsciptions in this list...
        subs: list[eventsub.SubscriptionPayload] = [
            eventsub.ChatMessageSubscription(broadcaster_user_id=payload.user_id, user_id=self.bot_id),
        ]
        print("added subscriptions list")

        resp: twitchio.MultiSubscribePayload = await self.multi_subscribe(subs)
        if resp.errors:
            LOGGER.warning("Failed to subscribe to: %r, for user: %s", resp.errors, payload.user_id)

    async def add_token(self, token: str, refresh: str) -> twitchio.authentication.ValidateTokenPayload:
        # Make sure to call super() as it will add the tokens interally and return us some data...
        resp: twitchio.authentication.ValidateTokenPayload = await super().add_token(token, refresh)

        # Store our tokens in a simple SQLite Database when they are authorized...
        # idk how this token stuff works but this is the default example twitchio had so i just copied/pasted
        query = """
        INSERT INTO tokens (user_id, token, refresh)
        VALUES (?, ?, ?)
        ON CONFLICT(user_id)
        DO UPDATE SET
            token = excluded.token,
            refresh = excluded.refresh;
        """

        async with self.token_database.acquire() as connection:
            await connection.execute(query, (resp.user_id, token, refresh))

        LOGGER.info("Added token to the database for user: %s", resp.user_id)
        return resp

    async def event_ready(self) -> None:
        LOGGER.info("Successfully logged in as: %s", self.bot_id)


class MyComponent(commands.Component):
    # An example of a Component with some simple commands and listeners
    # You can use Components within modules for a more organized codebase and hot-reloading.

    def __init__(self, bot: Bot) -> None:
        # Passing args is not required...
        # We pass bot here as an example...
        self.bot = bot

        rclpy.init()
        self.ros_node = Node('twitch_command_publisher')
        self.text_publisher = self.ros_node.create_publisher(
            String,
            '/text_command',
            10
        )

    # An example of listening to an event
    # We use a listener in our Component to display the messages received.
    @commands.Component.listener()
    async def event_message(self, payload: twitchio.ChatMessage) -> None:
        message = f"[{payload.broadcaster.name}] - {payload.chatter.name}: {payload.text}"
        print(message)

        # Publish text command to Pupper
        msg = String()
        msg.data = payload.text
        self.text_publisher.publish(msg)
        
        print(f"[{payload.broadcaster.name}] - {payload.chatter.name}: {payload.text}")
        return (payload.broadcaster.name, payload.chatter.name, payload.text)

    # EXAMPLES OF TWITCH COMMANDS - UNNECESSARY RIGHT NOW FOR PUPPER BUT SAVED FOR FUTURE REFERENCE
    # EXAMPLES OF TWITCH COMMANDS - UNNECESSARY RIGHT NOW FOR PUPPER BUT SAVED FOR FUTURE REFERENCE
    # EXAMPLES OF TWITCH COMMANDS - UNNECESSARY RIGHT NOW FOR PUPPER BUT SAVED FOR FUTURE REFERENCE
    # @commands.command()
    # async def hi(self, ctx: commands.Context) -> None:
    #     """Command that replies to the invoker with Hi <name>!

    #     !hi
    #     """
    #     await ctx.reply(f"Hi {ctx.chatter}!")

    # @commands.command()
    # async def say(self, ctx: commands.Context, *, message: str) -> None:
    #     """Command which repeats what the invoker sends.

    #     !say <message>
    #     """
    #     await ctx.send(message)

    # @commands.command()
    # async def add(self, ctx: commands.Context, left: int, right: int) -> None:
    #     """Command which adds to integers together.

    #     !add <number> <number>
    #     """
    #     await ctx.reply(f"{left} + {right} = {left + right}")

    # @commands.command()
    # async def choice(self, ctx: commands.Context, *choices: str) -> None:
    #     """Command which takes in an arbitrary amount of choices and randomly chooses one.

    #     !choice <choice_1> <choice_2> <choice_3> ...
    #     """
    #     await ctx.reply(f"You provided {len(choices)} choices, I choose: {random.choice(choices)}")

    # @commands.command(aliases=["thanks", "thank"])
    # async def give(self, ctx: commands.Context, user: twitchio.User, amount: int, *, message: str | None = None) -> None:
    #     """A more advanced example of a command which has makes use of the powerful argument parsing, argument converters and
    #     aliases.

    #     The first argument will be attempted to be converted to a User.
    #     The second argument will be converted to an integer if possible.
    #     The third argument is optional and will consume the reast of the message.

    #     !give <@user|user_name> <number> [message]
    #     !thank <@user|user_name> <number> [message]
    #     !thanks <@user|user_name> <number> [message]
    #     """
    #     msg = f"with message: {message}" if message else ""
    #     await ctx.send(f"{ctx.chatter.mention} gave {amount} thanks to {user.mention} {msg}")

    # @commands.group(invoke_fallback=True)
    # async def socials(self, ctx: commands.Context) -> None:
    #     """Group command for our social links.

    #     !socials
    #     """
    #     await ctx.send("discord.gg/..., youtube.com/..., twitch.tv/...")

    # @socials.command(name="discord")
    # async def socials_discord(self, ctx: commands.Context) -> None:
    #     """Sub command of socials that sends only our discord invite.

    #     !socials discord
    #     """
    #     await ctx.send("discord.gg/...")

#token database stuff that I didnt edit from the example bot; probably safer not to touch
async def setup_database(db: asqlite.Pool) -> tuple[list[tuple[str, str]], list[eventsub.SubscriptionPayload]]:
    # Create our token table, if it doesn't exist..
    # You should add the created files to .gitignore or potentially store them somewhere safer
    # Edit: I already added the created files to .gitignore

    # This is just for example purposes...

    query = """CREATE TABLE IF NOT EXISTS tokens(user_id TEXT PRIMARY KEY, token TEXT NOT NULL, refresh TEXT NOT NULL)"""
    async with db.acquire() as connection:
        await connection.execute(query)

        # Fetch any existing tokens...
        rows: list[sqlite3.Row] = await connection.fetchall("""SELECT * from tokens""")

        tokens: list[tuple[str, str]] = []
        subs: list[eventsub.SubscriptionPayload] = []

        for row in rows:
            tokens.append((row["token"], row["refresh"]))

            if row["user_id"] == BOT_ID:
                continue

            subs.extend([eventsub.ChatMessageSubscription(broadcaster_user_id=row["user_id"], user_id=BOT_ID)])

    return tokens, subs


# Our main entry point for our Bot
# Best to setup_logging here, before anything starts
# main loop for running the python file
def main() -> None:
    twitchio.utils.setup_logging(level=logging.INFO)

    async def runner() -> None:
        async with asqlite.create_pool("tokens.db") as tdb:
            tokens, subs = await setup_database(tdb)

            async with Bot(token_database=tdb, subs=subs) as bot:
                for pair in tokens:
                    await bot.add_token(*pair)

                print("starting bot")
                await bot.start(load_tokens=False)

    try:
        print("starting runner")
        asyncio.run(runner())
    except KeyboardInterrupt:
        LOGGER.warning("Shutting down due to KeyboardInterrupt")


if __name__ == "__main__":
    main()