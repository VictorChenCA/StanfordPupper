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

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    print("Warning: ROS2 not available. Running in chat-only mode.")
    ROS2_AVAILABLE = False
    Node = object  # Dummy for type hints

# Import chat processor and voting system
from chat_processor import ChatProcessor
from bucket_voting import VotingSystem
from command_executor import CommandExecutor


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

# Chat processor configuration
COMMAND_PREFIX = os.getenv("PUPPER_COMMAND_PREFIX", "!pupper")
BATCH_INTERVAL = float(os.getenv("PUPPER_BATCH_INTERVAL", "3.0"))
MAX_REQUESTS_PER_MINUTE = int(os.getenv("PUPPER_MAX_REQUESTS_PER_MINUTE", "12"))

# Bucket voting configuration
VOTE_DURATION = float(os.getenv("VOTE_DURATION", "30.0"))
MAX_BUCKET_TIME = float(os.getenv("MAX_BUCKET_TIME", "60.0"))
COUNTDOWN_INTERVAL = float(os.getenv("COUNTDOWN_INTERVAL", "1.0"))


class PupperCommandPublisher(Node):
    """ROS2 node that publishes commands to Pupper."""

    def __init__(self):
        super().__init__('twitch_command_publisher')
        # Publisher to the same topic that karel_realtime_commander listens to
        self.publisher = self.create_publisher(String, 'gpt4_response_topic', 10)
        self.get_logger().info('Twitch Command Publisher initialized')

    def publish_command(self, command: str):
        """Publish a command to Pupper."""
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')


class Bot(commands.AutoBot):

    # DO NOT need to mess around with this for pupper
    def __init__(
        self,
        *,
        token_database: asqlite.Pool,
        subs: list[eventsub.SubscriptionPayload],
        ros_publisher=None,
        voting_system=None,
        command_executor=None
    ) -> None:
        self.token_database = token_database
        self.ros_publisher = ros_publisher
        self.voting_system = voting_system
        self.command_executor = command_executor

        # Initialize chat processor with voting system
        self.chat_processor = ChatProcessor(
            command_prefix=COMMAND_PREFIX,
            batch_interval=BATCH_INTERVAL,
            max_requests_per_minute=MAX_REQUESTS_PER_MINUTE,
            voting_system=voting_system
        )

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
        # Note: Commands now go through bucket voting system
        # No need for direct callback - votes accumulate in buckets
        # CommandExecutor handles winner selection and execution

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
        print(f"✅ Successfully logged in to Twitch as: {self.bot_id}", flush=True)

        # Start voting system countdown
        if self.voting_system:
            await self.voting_system.start_countdown()
            print("✅ Voting system countdown started", flush=True)
            LOGGER.info("Voting system countdown started")

        # Start command executor
        if self.command_executor:
            await self.command_executor.start()
            print("✅ Command executor started", flush=True)
            LOGGER.info("Command executor started")

        # Start chat processor
        await self.chat_processor.start()
        print("✅ Chat processor initialization complete", flush=True)
        LOGGER.info("Chat processor started and ready to process commands")


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
        """Process incoming Twitch chat messages and pass to chat processor."""
        message = f"[{payload.broadcaster.name}] - {payload.chatter.name}: {payload.text}"
        print(f"🎮 MESSAGE RECEIVED: [{payload.broadcaster.name}] - {payload.chatter.name}: {payload.text}", flush=True)

        # Pass message to chat processor (will filter by command prefix)
        self.bot.chat_processor.add_message(
            username=payload.chatter.name,
            text=payload.text,
            broadcaster=payload.broadcaster.name
        )

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

    # Initialize ROS2 if available
    ros_publisher = None
    if ROS2_AVAILABLE:
        try:
            rclpy.init()
            ros_publisher = PupperCommandPublisher()
            LOGGER.info("ROS2 publisher initialized")
        except Exception as e:
            LOGGER.error(f"Failed to initialize ROS2: {e}")
            LOGGER.warning("Running in chat-only mode (no robot control)")

    # Initialize bucket voting system
    voting_system = VotingSystem(
        vote_duration=VOTE_DURATION,
        max_bucket_time=MAX_BUCKET_TIME,
        countdown_interval=COUNTDOWN_INTERVAL
    )
    LOGGER.info(f"Voting system initialized ({VOTE_DURATION}s/vote, {MAX_BUCKET_TIME}s max)")

    # Initialize command executor
    command_executor = CommandExecutor(
        voting_system=voting_system,
        ros_publisher=ros_publisher
    )
    LOGGER.info("Command executor initialized")

    async def runner() -> None:
        async with asqlite.create_pool("tokens.db") as tdb:
            tokens, subs = await setup_database(tdb)

            async with Bot(
                token_database=tdb,
                subs=subs,
                ros_publisher=ros_publisher,
                voting_system=voting_system,
                command_executor=command_executor
            ) as bot:
                for pair in tokens:
                    await bot.add_token(*pair)

                print("starting bot")
                await bot.start(load_tokens=False)

    try:
        print("starting runner")
        asyncio.run(runner())
    except KeyboardInterrupt:
        LOGGER.warning("Shutting down due to KeyboardInterrupt")
    finally:
        # Cleanup ROS2
        if ROS2_AVAILABLE and ros_publisher:
            try:
                ros_publisher.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                LOGGER.error(f"Error shutting down ROS2: {e}")


if __name__ == "__main__":
    main()