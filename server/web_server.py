"""
Tankbot Asynchronous Web & WebSocket Server
Built on aiohttp for low-latency full-duplex mobile communication.
"""

import json
import logging
import asyncio
from pathlib import Path
from aiohttp import web
from config import SERVER_CONFIG, WEB_DIR
from core.tankbot import Tankbot

logger = logging.getLogger("Tankbot.Server")


class TankbotWebServer:
    """Hosts Mobile UI and WebSocket command & telemetry channel"""

    def __init__(self, robot: Tankbot, config: dict = SERVER_CONFIG):
        self.robot = robot
        self.config = config
        self.app = web.Application()
        self.connected_websockets = set()
        self._broadcast_task: asyncio.Task = None

        self._setup_routes()

    def _setup_routes(self):
        self.app.router.add_get("/", self._handle_index)
        self.app.router.add_get("/ws", self._handle_websocket)
        self.app.router.add_get("/video_feed", self._handle_video_feed)
        self.app.router.add_get("/api/status", self._handle_api_status)
        self.app.router.add_post("/api/emergency_stop", self._handle_api_estop)
        self.app.router.add_post("/api/mode", self._handle_api_mode)
        self.app.router.add_post("/api/vision/color", self._handle_api_vision_color)
        self.app.router.add_post("/api/vision/preset", self._handle_api_vision_preset)

        # Serve static assets (CSS, JS, icons)
        self.app.router.add_static("/static/", path=str(WEB_DIR), name="static")

    async def _handle_index(self, request: web.Request):
        index_file = WEB_DIR / "index.html"
        if not index_file.exists():
            return web.Response(text="Tankbot Web UI index.html not found.", status=404)
        return web.FileResponse(index_file)

    async def _handle_video_feed(self, request: web.Request):
        """Streams live annotated MJPEG video over HTTP for Mobile UI"""
        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0",
            }
        )
        await response.prepare(request)

        try:
            while True:
                frame = self.robot.camera.get_frame()
                if frame is not None:
                    # Run color tracking & annotation
                    annotated_frame, _ = self.robot.color_tracker.process_frame(frame, annotate=True)
                    try:
                        import cv2
                        ret, buf = cv2.imencode(".jpg", annotated_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                        if ret:
                            jpeg_data = buf.tobytes()
                            header = (
                                f"--frame\r\n"
                                f"Content-Type: image/jpeg\r\n"
                                f"Content-Length: {len(jpeg_data)}\r\n\r\n"
                            ).encode("utf-8")
                            await response.write(header + jpeg_data + b"\r\n")
                    except Exception as e:
                        logger.debug(f"Video frame encode error: {e}")

                await asyncio.sleep(0.04)  # ~25 FPS stream
        except (asyncio.CancelledError, ConnectionResetError):
            pass

        return response

    async def _handle_api_status(self, request: web.Request):
        return web.json_response(self.robot.get_telemetry())

    async def _handle_api_estop(self, request: web.Request):
        self.robot.emergency_stop()
        return web.json_response({"status": "EMERGENCY_STOP_ACTIVATED"})

    async def _handle_api_mode(self, request: web.Request):
        try:
            data = await request.json()
            mode = data.get("mode")
            if mode:
                self.robot.set_mode(mode)
                return web.json_response({"status": "OK", "mode": self.robot.mode})
        except Exception as e:
            return web.json_response({"error": str(e)}, status=400)
        return web.json_response({"error": "Invalid mode request"}, status=400)

    async def _handle_api_vision_color(self, request: web.Request):
        try:
            data = await request.json()
            color = data.get("color", "red")
            self.robot.color_tracker.set_target_color(color)
            return web.json_response({"status": "OK", "target_color": self.robot.color_tracker.target_color})
        except Exception as e:
            return web.json_response({"error": str(e)}, status=400)

    async def _handle_api_vision_preset(self, request: web.Request):
        try:
            data = await request.json()
            action = data.get("action", "vision_pick_and_place")
            self.robot.trigger_preset(action)
            return web.json_response({"status": "OK", "action": action})
        except Exception as e:
            return web.json_response({"error": str(e)}, status=400)

    async def _handle_websocket(self, request: web.Request):
        ws = web.WebSocketResponse(heartbeat=10.0)
        await ws.prepare(request)

        self.connected_websockets.add(ws)
        client_ip = request.remote
        logger.info(f"Mobile UI connected via WebSocket from {client_ip}")

        # Send immediate greeting with current state
        await ws.send_json({
            "type": "init",
            "telemetry": self.robot.get_telemetry(),
            "config": {
                "servos": self.robot.servos.servo_defs,
                "is_simulation": self.robot.hal.is_simulation
            }
        })

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        cmd = json.loads(msg.data)
                        await self._process_command(cmd)
                    except json.JSONDecodeError:
                        logger.warning(f"Invalid JSON payload: {msg.data}")
                elif msg.type == web.WSMsgType.ERROR:
                    logger.error(f"WebSocket connection closed with error: {ws.exception()}")
        finally:
            self.connected_websockets.discard(ws)
            logger.info(f"Mobile UI disconnected from {client_ip}")
            # If no clients remain, ensure motors stop
            if not self.connected_websockets and self.robot.mode == "MANUAL":
                self.robot.motors.stop()

        return ws

    async def _process_command(self, cmd: dict):
        """Dispatches incoming mobile UI actions"""
        cmd_type = cmd.get("type")
        self.robot.heartbeat()

        if cmd_type == "ping":
            return

        elif cmd_type == "joystick":
            throttle = float(cmd.get("throttle", 0.0))
            steering = float(cmd.get("steering", 0.0))
            self.robot.drive_joystick(throttle, steering)

        elif cmd_type == "dpad":
            direction = cmd.get("direction", "stop")
            speed = int(cmd.get("speed", 80))
            if direction == "forward":
                self.robot.drive_direct(speed, speed)
            elif direction == "backward":
                self.robot.drive_direct(-speed, -speed)
            elif direction == "left":
                self.robot.drive_direct(-speed, speed)
            elif direction == "right":
                self.robot.drive_direct(speed, -speed)
            elif direction == "stop":
                self.robot.drive_direct(0, 0)

        elif cmd_type == "servo":
            servo_id = int(cmd.get("id"))
            pos = int(cmd.get("pos"))
            duration = int(cmd.get("duration", 60))
            self.robot.set_servo(servo_id, pos, duration)

        elif cmd_type == "preset":
            name = cmd.get("name")
            if name:
                self.robot.trigger_preset(name)

        elif cmd_type == "mode":
            mode = cmd.get("mode")
            if mode:
                self.robot.set_mode(mode)

        elif cmd_type == "vision_color":
            color = cmd.get("color", "red")
            self.robot.color_tracker.set_target_color(color)

        elif cmd_type == "vision_mode":
            mode = cmd.get("mode")
            if mode:
                self.robot.set_mode(mode)

        elif cmd_type == "vision_pick":
            self.robot.trigger_preset("vision_pick_and_place")

        elif cmd_type == "emergency_stop":
            self.robot.emergency_stop()

        elif cmd_type == "reset_stop":
            self.robot.reset_emergency_stop()

    async def _broadcast_telemetry_loop(self):
        """Streams live telemetry at regular intervals to all connected phones"""
        interval = 1.0 / self.config.get("broadcast_interval_hz", 20)
        while True:
            try:
                if self.connected_websockets:
                    payload = json.dumps({
                        "type": "telemetry",
                        "data": self.robot.get_telemetry()
                    })
                    # Fan out to all connected mobile browsers
                    dead_ws = []
                    for ws in self.connected_websockets:
                        try:
                            await ws.send_str(payload)
                        except Exception:
                            dead_ws.append(ws)
                    for ws in dead_ws:
                        self.connected_websockets.discard(ws)

                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Telemetry broadcast error: {e}")
                await asyncio.sleep(0.5)

    async def start(self):
        self._broadcast_task = asyncio.create_task(self._broadcast_telemetry_loop())
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, self.config["host"], self.config["port"])
        await self.site.start()
        logger.info(f"Tankbot Mobile Web UI server running at http://{self.config['host']}:{self.config['port']}")
        return self.runner

    async def stop(self):
        if self._broadcast_task:
            self._broadcast_task.cancel()
        if hasattr(self, "runner") and self.runner:
            await self.runner.cleanup()
        logger.info("Tankbot Web UI server stopped.")
