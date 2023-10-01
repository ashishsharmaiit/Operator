import asyncio
import rospy
from websocket_client import WebsocketClient


async def main():
    uri = "ws://34.221.226.79:8084/"
    # you should leave as blank -> "" if there is only one or zero video
    video1_path = "/dev/video0"
    # you should leave as blank -> "" if there is only one or zero video
    video2_path = "/dev/video1"
    # you should leave as blank -> "" if data channel is not needed. Don't mention "no". Any value in that variable indicates data channel is needed. Also note that robot_controller code won't be also run in loop if data channel is not there, so it is assumed robot code is not needed.
    data_channel_required = "no"
    websocket_client = WebsocketClient(
        uri=uri, video1_path=video1_path, video2_path=video2_path, data_channel_required=data_channel_required)

    #if data_channel_required:
    #    await asyncio.gather(websocket_client.connect(), websocket_client.robot_controller.control_robot())
    #else:
    #    await websocket_client.connect()
    await websocket_client.connect()


if __name__ == "__main__":
    asyncio.run(main())
