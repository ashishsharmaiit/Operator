import asyncio
import json
#Ros comment from here
'''
import rospy
from std_msgs.msg import String
'''
#Ros comment till here


class RobotController:
    def __init__(self, websocket_client, webrtc_manager):
        self.websocket_client = websocket_client
        self.webrtc_manager = webrtc_manager
        #self.publisher = rospy.Publisher('chatter', String, queue_size=10)
    
    #Ros comment till here

    '''
    async def talker(self):
        while not rospy.is_shutdown():
            hello_str = "hello ROS %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.publisher.publish(hello_str)
            await asyncio.sleep(1)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    async def listener(self):
        rospy.Subscriber('chatter', String, self.callback)
        while not rospy.is_shutdown():
            await asyncio.sleep(1)
            '''
    #Ros comment till here


    async def control_robot(self):
        #asyncio.create_task(self.talker())
        #asyncio.create_task(self.listener())
        g = 0
        while g < 1000:
            print("this is robot code running", g)
            g=g+1
            await self.webrtc_manager.send_data("[WARNING]This is robot code running")
            await asyncio.sleep(2)
            await self.webrtc_manager.send_data("[ERROR] ROBOT IS IN HOME CONFIGURATION WITH GRIPPER OPEN")								
            await asyncio.sleep(2)
            await self.webrtc_manager.send_data("JOINT LIMIT VIOLATION")				
            await asyncio.sleep(2)
            await self.webrtc_manager.send_data("[WARNING]CLOSSING GRIPPER. ")									
            await asyncio.sleep(2)
            await self.webrtc_manager.send_data("[ERROR]OPENING GRIPPER")									
            await asyncio.sleep(2)
            await self.webrtc_manager.send_data(" JOINT LIMIT VIOLATION")
