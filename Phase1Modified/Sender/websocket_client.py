# websocket_client.py

import json
import websockets
from webrtc_manager import WebRTCManager
from robot_controller import RobotController

class WebsocketClient:
    def __init__(self, uri, video1_path, video2_path, data_channel_required):
        self.uri = uri
        self.websocket = None
        self.webrtc_manager = WebRTCManager(self.send_message, video1_path, video2_path, data_channel_required)
        if data_channel_required:
            self.robot_controller = RobotController(self, self.webrtc_manager)

    async def connect(self):
        while True:
            try:
                self.websocket = await websockets.connect(self.uri)
                async for message in self.websocket:
                    data = json.loads(message)
                    if data['type'] == 'offer':
                        print("Received offer", data)
                        await self.webrtc_manager.handle_offer(data)
            except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK):
                print("Connection lost. Reconnecting...")
                await asyncio.sleep(3)  # wait for 3 seconds before reconnecting

    async def send_message(self, message):
        if self.websocket:
            try:
                await self.websocket.send(message)
            except (websockets.ConnectionClosedError, websockets.ConnectionClosedOK):
                print("Failed to send message. Connection is closed.")
        else:
            print("Websocket connection is not established.")
