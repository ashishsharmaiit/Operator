# Phase1PointNshootMVP Instructions

1. No change in python code. You can use the last functional code that you have running. Run the webserver.js and main.py. Please make suitable changes to websocket address and video ID etc in main.py. Run the appropriate webserver. 
2. Copy and paste the Receiver folder in a separate folder. Please note that the steps to run the browser files have changed because of modularization of code. Modularized code can't run by just opening the file in a browser, due to security reasons imposed by browsers.
3. Go inside that folder in a terminal.
4. Run $ python3 -m http.server
   Keep this server running.
6. Open the link that will be published in the terminal after running above command (something like http://0.0.0.0:8000) in a browser (preferably chrome). 
7. Now, click on the html file in the menu shown in broswer. After that, you should be able to run the earlier functions as-is. Ignore the red dot in the middle. That is just temporary. Will remove it later.
8. To try point-and-shoot functionality, keep the view as the default one. With both videos side-by-side without any rototaion. Now, click on the right-most small button in the middle that has 3 horizontal bars/menu image on it.
9. Once you press the button, you should see the text in the yellow box at the bottom-right changing from ManualMode to AutoMode. Now, if you press any button in the joystick except the right-most small button, then the joystick data won't be sent to robot at all. You can press the right-most small button again to go back to manual mode, and it will start sending the data again to robot.
10. In AutoMode, if you move the left joystick, then red dot should start from the middle of left-video (video1) and will move across the video. If you press button A, then it will send the video coordinates (x and y) over webrtc. Please note that this functionality only works for video1.



# Common Instructions (might be outdated)

Steps to use code:

1.Select the folder which fits your use case - one video or two video streams.

2. Run the server file from the EC2 server with the steps below:

a. Go to EC2 console from web browser, and ensure that the 'picky' name server is running. If it is not running, run it using 'instance state'. After using, please switch off to save the cost.

b. Once it is running, go to terminal and go the folder where picky.pem file is saved.

c. type this command from that folder - chmod 400 picky.pem

d. Go to the EC2 instance and copy the Public DNS server address. It will read like ec2-34-218-222-3.us-west-2.compute.amazonaws.com, and will change with the Public IP address.

e. Then type this command -> ssh -i robotics.pem ubuntu@<replace with public ip address> e.g. ssh -i robotics.pem ubuntu@ec2-34-218-222-3.us-west-2.compute.amazonaws.com -> please note that "ubuntu" in this command will need to be replaced with corresponding value for windows or mac. You might want to google that if you are not using ubuntu.

f. press "yes" in terminal if asked.

g. now you should be have ssh into the EC2 server.

h. Go to signaling-server folder


3. Type the command node webcamServer.js for 8083, node webcamserver2.js for 8084 and node webcamServer.js for 8085, to run the server file. The file should be already saved in the server. 

4. From the downloaded folder, change the server IP address in sender and client.js codes in receiever files as per the new IP from EC2. IP refreshes every time server is switched on, if it was off earlier. If it continued to run, then IP address shouldn't have changed. Search for the code that has 'ws:' and change the ip address there.

5. Copy the python file into your src folder (no need to have roscore and catkin_make like in TCP since this is not on ROS but ROS could be integrated easily). If you are testing with webcam, no change is needed in python file in onewebcam folder. However, if you are not using twowebcam folder, or not using webcam, then you will need to change the video numbers. Search for '/dev/video' in the python code and replace video0 and video2 with relevant values. You can just try video0, video1, video2 etc and check. Or you can go to terminal and type these two commands
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
and you will know the camera ID lists.

6. Run the python file using python3 webcamJS.py. Wait for it to establish connection with server. You will see client connected message in the EC2 server terminal
Now, you can move to the next step. Please Note that you should open the python and wait till you see connected sign in terminal before you open the html. It will make a difference for UDP if you don't do this. If you opened python later, just refresh the html once you get websocket connected message in python.

7. Save all the sender files in the same folder in desktop. Open the sender html file in chrome. Open Inspect/Console to check the logs. 

8. Select Stun server check box and click on Start. You should see the image now from the other peer. If you are working on twowebcam version, then you will need to scroll a little to go to bottom to see both the video files. Once you see videos, the connection is established. Now you can try moving the Joystick, and it should send the joystick data to the python client as well.

9. To exit python code, you will need to press ctrl-c twice.
  
10. Debug Tip - If you exit with ctrl-z or any other process, the camera might still be accessible by python script, and if you run this script again you might get the error, that camera is not accessile. To disable camera being handled by previous process, type lsof /dev/video0 and then you will get process ID (PID), then type kill -9 <PID> and it will kill the process. Then you can do this again.
