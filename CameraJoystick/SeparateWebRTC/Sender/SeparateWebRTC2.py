#!/usr/bin/env python3
import argparse
import asyncio
import json
import logging
'''import os
import platform
import ssl'''
import time
import websockets
from functools import partial, wraps

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCOutboundRtpStreamStats
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

'''
# **** ROS ****
import rospy
from web_joystick_control.msg import JoystickData
# **** ROS ****

import rospy
import sys
sys.path.append(os.path.dirname(__file__))
from port_setup import setup
from web_joystick_control.msg import JoystickData
'''
speed = 50
max_speed = 100

home_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, -45]
joy_joint_values = home_joint_values
gripper_command = 0 # OPEN
# global g = gripper_command
dclose = 5
dummy_get_gripper_value = 1
#ROOT = os.path.dirname(__file__)
offercount = 0

#mc = setup()


'''
def joy_callback(joy):
	global joy_joint_values
	global gripper_command
	increment = 5
	gripper_command = joy.buttons[0]
	joy_joint_values[0] -= joy.axes[0] * increment # J1 -> Left: counter-clockwise, Right: Clockwise 
	joy_joint_values[1] += joy.axes[1] * increment # J2
	joy_joint_values[2] += joy.axes[3] * increment # J3
	joy_joint_values[3] += joy.axes[4] * increment # J4
	if (joy.buttons[1]):
		joy_joint_values[4] += 0.5*(joy.axes[2] + 1)  * increment # J5 initial value = -1 (-1 -> +1)
		joy_joint_values[5] += 0.5*(joy.axes[5] + 1) * increment # J6  initial value = -1 (-1 -> +1)
	else:
		joy_joint_values[4] -= 0.5*(joy.axes[2] + 1)  * increment # J5 initial value = -1 (-1 -> +1)
		joy_joint_values[5] -= 0.5*(joy.axes[5] + 1) * increment # J6  initial value = -1 (-1 -> +1)	


def get_gripper_value(mc):
	while dummy_get_gripper_value <0:
	#while mc.get_gripper_value() < 0:
		# print("GRIPPER IS SENDING BAD DATA (-1)")
		send_over_data_channel(json.dumps("[ERROR] GRIPPER BAD DATA -1"))
		#rospy.logerr("GRIPPER IS SENDING BAD DATA (-1)")
	#return mc.get_gripper_value()
	return dummy_get_gripper_value
'''

def create_local_tracks():
    global relay2, webcam2
    relay2 = MediaRelay()
    return None, relay2.subscribe(webcam2.video)

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )

pcs = set()

    

def partial_async(func, *args, **kwargs):
    async def wrapped(*args2, **kwargs2):
        return await func(*args, *args2, **kwargs, **kwargs2)
    return wrapped

async def websocket_client():
    global offercount, webcam2
    uri = "ws://34.221.226.79:8084/"  # replace with your WebSocket server's URI
    async with websockets.connect(uri) as websocket:
        async for message in websocket:
            data = json.loads(message)
            if data['type'] == 'offer':
                print("Received offer", data)
                offercount += 1
                print ("This offer's count is ",offercount)
                if offercount == 1:
                    print ("triggering offer count = 0 function")
                    options2 = {"framerate": data['frame2'], "video_size": data['video_size2']}
                    webcam2 = MediaPlayer("/dev/video0", format="v4l2", options=options2)
                    print("frame2",data['frame2'],"video_size2",data['video_size2'],"codec2",data['codec2'])
                if offercount > 1:
                        print ("triggering offer count > 1 function")
                        coros = [pc.close() for pc in pcs]
                        await asyncio.gather(*coros)
                        pcs.clear()
                sdp = data['sdp']
                offer = RTCSessionDescription(sdp=sdp, type='offer')

                pc = RTCPeerConnection()
                pcs.add(pc)


                @pc.on("connectionstatechange")
                async def on_connectionstatechange():
                    print("Connection state is %s" % pc.connectionState)
                    if pc.connectionState == "failed":
                        await pc.close()
                        pcs.discard(pc)

                # open media source
                _, video2 = create_local_tracks()


                if video2:
                    print("Adding video2")
                    video2_sender = pc.addTrack(video2)
                    if data['codec2']:
                        force_codec(pc, video2_sender, data['codec2'])
                    else:
                        print("No Video Encoding 2")

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


async def robot_control():
	# ROS Node Initialization
	#rospy.init_node("arm_joystick_joint_node")
	#rospy.loginfo("Arm Node Initialized.")
	'''
	rate = rospy.Rate(10)
	mc.sync_send_angles(home_joint_values, speed)
	await asyncio.sleep(1)
	#mc.set_gripper_state(0, 100)
	gripper_value = 100
	await asyncio.sleep(2)
	# print("*** ROBOT IS IN HOME CONFIGURATION WITH GRIPPER OPEN***")
	send_over_data_channel(json.dumps("ROBOT IS IN HOME CONFIGURATION WITH GRIPPER OPEN"))								
	# Subscribe to joystick data
	rospy.Subscriber("/joystick_data", JoystickData, joy_callback)
	'''
	# Control Loop
	g = 0

	
	while g <1000:
		print("this is robot code running",g)
		g=g+1
		await asyncio.sleep(2)


	'''
	while not rospy.is_shutdown():
		j = joy_joint_values
		# rospy.loginfo("Joy Joystick Value: %d, %d, %d, %d, %d, %d", j[0], j[1], j[2], j[3], j[4], j[5])
		# rospy.loginfo("Gripper command: %d", gripper_command)
		# rospy.loginfo("Gripper value: %d", gripper_value)
		send_over_data_channel(json.dumps("Joystick Value: J1:{}, J2{}, J3{}, J4{}, J5{}, J6{}".format(j[0], j[1], j[2], j[3], j[4], j[5])))	
		send_over_data_channel(json.dumps("Gripper command: {}".format(gripper_command)))	
		send_over_data_channel(json.dumps("Gripper value: {}".format(gripper_value)))	
						
		# Joint Control
		if ( (j[0] < -170) or (j[0] > +170) or (j[1] < -170) or (j[1] > +170) or (j[2] < -170) or (j[2] > +170) or (j[3] < -170) or (j[3] > +170) or (j[4] < -170) or (j[4] > +170) or (j[5] < -180) or (j[5] > +180)):
			# rospy.logerr("*** JOINT LIMIT VIOLATION ***")
			send_over_data_channel(json.dumps("[ERROR] JOINT LIMIT VIOLATION"))				
		else:
			mc.send_angles(joy_joint_values, speed)
			# print("MOVE ROBOT")
			await asyncio.sleep(0.1)
		
		
		# Gripper Control
		if gripper_command:
			if g == 0:
				# Close Gripper
				gripper_value = get_gripper_value(mc)
				while gripper_command and gripper_value > 0:
					mc.set_gripper_value(gripper_value, 100)				
					gripper_value -= 2
					# print("CLOSING GRIPPER ...", gripper_value)
					send_over_data_channel(json.dumps("CLOSSING GRIPPER. Gripper Value: {}".format(gripper_value)))									
					await asyncio.sleep(0.1)
										
				rospy.logwarn("[WARNING] Please wait for 3 seconds!")								
				time.sleep(3)
				g = 1
			elif g == 1:
				# Open Gripper
				mc.set_gripper_state(0, 100)							
				# print("OPENING GRIPPER ...")
				send_over_data_channel(json.dumps("OPENING GRIPPER"))									
				
				rospy.logwarn("[WARNING] Please wait for 3 seconds!")								
				await asyncio.sleep(3)
				g = 0				
			else:
				continue				
		
		await asyncio.sleep(0.1)
		'''

async def main():	
    # asyncio.get_event_loop().run_until_complete(websocket_client())
    await asyncio.gather(websocket_client(), robot_control())
    #*** ROS ***
    #rospy.spin()


 
if __name__ == "__main__":
    
    # ************* ROS **********
    #rospy.init_node("web_joystick_node")
    #rospy.loginfo("Websocket client node initialized")
    #joystick_data_pub = rospy.Publisher("/joystick_data", JoystickData, queue_size= 10)
    # ****************************
    
    asyncio.run(main())
