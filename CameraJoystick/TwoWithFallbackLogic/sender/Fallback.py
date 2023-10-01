import argparse
import asyncio
import json
import logging
import os
import platform
import ssl
import time
import websockets
from functools import partial, wraps

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCOutboundRtpStreamStats
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

ROOT = os.path.dirname(__file__)

options = {"framerate": "30", "video_size": "640x480"}
offercount = 0
webcam1 = MediaPlayer("/dev/video0", format="v4l2", options=options)
webcam2 = MediaPlayer("/dev/video2", format="v4l2", options=options)



def create_local_tracks(play_from, decode):
    global relay1, relay2, webcam1, webcam2

    if play_from:
        player = MediaPlayer(play_from, decode=decode)
        return player.audio, player.video
    else:
        relay1 = MediaRelay()
        relay2 = MediaRelay()
        return None, relay1.subscribe(webcam1.video), relay2.subscribe(webcam2.video)

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

pcs = set()

async def on_channel_message(data_channel, message):
    print('Received message on data channel:', message)
    data_channel.send('Hello Back from Python!')

def partial_async(func, *args, **kwargs):
    async def wrapped(*args2, **kwargs2):
        return await func(*args, *args2, **kwargs, **kwargs2)
    return wrapped

async def websocket_client():
    global offercount, webcam1, webcam2
    uri = "ws://54.218.1.229:8083/"  # replace with your WebSocket server's URI
    async with websockets.connect(uri) as websocket:
        async for message in websocket:
            data = json.loads(message)
            if data['type'] == 'offer':
                offercount += 1
                print ("This offer's count is ",offercount)
                if offercount > 1:
                        print ("triggering offer count > 1 function")
                        coros = [pc.close() for pc in pcs]
                        await asyncio.gather(*coros)
                        pcs.clear()
                print("Received offer", data)
                sdp = data['sdp']
                offer = RTCSessionDescription(sdp=sdp, type='offer')

                pc = RTCPeerConnection()
                pcs.add(pc)

                # Create a data channel and add it to the connection
                data_channel = pc.createDataChannel('dataChannel')
                data_channel.on("message", partial(on_channel_message, data_channel))
                data_channel.on("open", lambda: print("Data channel opened"))

                @pc.on("connectionstatechange")
                async def on_connectionstatechange():
                    print("Connection state is %s" % pc.connectionState)
                    if pc.connectionState == "failed":
                        await pc.close()
                        pcs.discard(pc)

                # open media source
                audio, video1, video2 = create_local_tracks(
                    args.play_from, decode=not args.play_without_decoding
                )

                if audio:
                    audio_sender = pc.addTrack(audio)
                    if args.audio_codec:
                        force_codec(pc, audio_sender, args.audio_codec)
                    elif args.play_without_decoding:
                        raise Exception("You must specify the audio codec using --audio-codec")

                if video1:
                    print("Adding video1")
                    video1_sender = pc.addTrack(video1)
                    if args.video_codec:
                        force_codec(pc, video1_sender, args.video_codec)
                    elif args.play_without_decoding:
                        raise Exception("You must specify the video codec using --video-codec")

                if video2:
                    print("Adding video2")
                    video2_sender = pc.addTrack(video2)
                    if args.video_codec:
                        force_codec(pc, video2_sender, args.video_codec)
                    elif args.play_without_decoding:
                        raise Exception("You must specify the video codec using --video-codec")

                await pc.setRemoteDescription(offer)

                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)

                # send the answer back to the WebSocket server
                answer = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
                await websocket.send(json.dumps(answer))

async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC webcam demo")
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument("--play-from", help="Read the media from a file and sent it.")
    parser.add_argument(
        "--play-without-decoding",
        help=(
            "Read the media without decoding it (experimental). "
            "For now it only works with an MPEGTS container with only H.264 video."
        ),
        action="store_true",
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument(
        "--audio-codec", help="Force a specific audio codec (e.g. audio/opus)"
    )
    parser.add_argument(
        "--video-codec", help="Force a specific video codec (e.g. video/H264)"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    # Start the WebSocket client
    asyncio.get_event_loop().run_until_complete(websocket_client())
