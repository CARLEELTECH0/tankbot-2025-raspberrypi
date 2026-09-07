#!/usr/bin/env python3
"""
Tankbot 2025 - Raspberry Pi Edition
Master Launcher: Runs the robot hardware controller and async mobile web server.
"""

import sys
import os
import signal
import socket
import logging
import argparse
import asyncio

# Add project root to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import SERVER_CONFIG
from core.tankbot import Tankbot
from server.web_server import TankbotWebServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("Tankbot.Main")


def get_local_ip() -> str:
    """Attempts to find the Raspberry Pi's local network IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


async def async_main():
    parser = argparse.ArgumentParser(description="Tankbot Raspberry Pi Controller & Mobile UI Server")
    parser.add_argument("--host", default=SERVER_CONFIG["host"], help="Web server host IP")
    parser.add_argument("--port", type=int, default=SERVER_CONFIG["port"], help="Web server port")
    parser.add_argument("--servo-type", choices=["BUS_SERVO", "PWM_SERVO"], default="BUS_SERVO", help="Servo control protocol")
    args = parser.parse_args()

    local_ip = get_local_ip()

    print("\n" + "=" * 65)
    print("      🤖  TANKBOT 2025 - RASPBERRY PI EDITION  🤖")
    print("=" * 65)
    print(f"  Local Access:      http://127.0.0.1:{args.port}")
    print(f"  Mobile Network UI: http://{local_ip}:{args.port}")
    print("=" * 65 + "\n")

    # Instantiate Robot & Web Server
    robot = Tankbot()
    await robot.start()

    server = TankbotWebServer(robot, {
        "host": args.host,
        "port": args.port,
        "broadcast_interval_hz": SERVER_CONFIG.get("broadcast_interval_hz", 20)
    })
    runner = await server.start()

    # Graceful shutdown event
    stop_event = asyncio.Event()

    def signal_handler():
        logger.info("Shutdown signal received. Stopping Tankbot...")
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, signal_handler)
        except NotImplementedError:
            # Windows fallback (if ever run on Windows)
            pass

    try:
        await stop_event.wait()
    finally:
        logger.info("Cleaning up server and hardware...")
        await runner.cleanup()
        await robot.stop()
        logger.info("Tankbot shutdown complete.")


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
