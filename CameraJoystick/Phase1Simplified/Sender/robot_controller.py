from port_setup import setup
from web_joystick_control.msg import JoystickData
import rospy
import time
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BCM)


class RobotController:
    def __init__(self):
        self.joystick_data = None
        self.joy_joint_values = [0]*6
        self.gripper_command = 0
        self.gripper_value = 1
        self.speed = 50
        self.increment = 5
        self.pump_pin = 20
        self.vent_pin = 21
        self.mc = setup()
        # Either pin 20/21 can control the switch of the suction pump.
        # Note: the switch should use the same pin Foot control
        GPIO.setup(self.pump_pin, GPIO.OUT)
        GPIO.setup(self.vent_pin, GPIO.OUT)
        self.joystick_subscriber = rospy.Subscriber(
            "/joystick_data", JoystickData, self.joy_callback)
        print("Arm is Ready to Move ...")

    def joy_callback(self, JoystickData):
        self.joystick_data = JoystickData

    def get_gripper_value(self):
        while self.mc.get_gripper_value() < 0:
            print("[ERROR] Gripper Bad Data -1!")
        return self.mc.get_gripper_value()

    # Turn on the suction pump
    def pump_on(self):
        # open suction pump
        GPIO.output(self.pump_pin, 0)

    # Stop suction pump
    def pump_off(self):
        # Shut down the suction pump
        GPIO.output(self.pump_pin, 1)
        # await asyncio.sleep(0.05)
        GPIO.output(self.vent_pin, 0)
        # await asyncio.sleep(1)
        GPIO.output(self.vent_pin, 1)
        # await syncio.sleep(0.05)

    def control_robot(self):
        # DEBUG
        while not rospy.is_shutdown():
            if self.joystick_data is not None:
                self.gripper_command = self.joystick_data.buttons[0]
                # J1
                self.joy_joint_values[0] -= self.joystick_data.axes[0] * \
                    self.increment
                # J2
                self.joy_joint_values[1] += self.joystick_data.axes[1] * \
                    self.increment
                # J3
                self.joy_joint_values[2] += self.joystick_data.axes[3] * \
                    self.increment
                # J4
                self.joy_joint_values[3] += self.joystick_data.axes[4] * \
                    self.increment
                if (self.joystick_data.buttons[1]):
                    # J5 initial value = -1 (-1 -> +1)
                    self.joy_joint_values[4] += 0.5 * \
                        (self.joystick_data.axes[2] + 1) * self.increment
                    # J6  initial value = -1 (-1 -> +1)
                    self.joy_joint_values[5] += 0.5 * \
                        (self.joystick_data.axes[5] + 1) * self.increment
                else:
                    # J5 initial value = -1 (-1 -> +1)
                    self.joy_joint_values[4] -= 0.5 * \
                        (self.joystick_data.axes[2] + 1) * self.increment
                    # J6  initial value = -1 (-1 -> +1)
                    self.joy_joint_values[5] -= 0.5 * \
                        (self.joystick_data.axes[5] + 1) * self.increment

            # Joint Control
            j = self.joy_joint_values
            if ((j[0] < -170) or (j[0] > +170) or (j[1] < -170) or (j[1] > +170) or (j[2] < -170) or (j[2] > +170) or (j[3] < -170) or (j[3] > +170) or (j[4] < -170) or (j[4] > +170) or (j[5] < -180) or (j[5] > +180)):
                print("Error Joint Limit Violation.")
            else:
                self.mc.send_angles(self.joy_joint_values, self.speed)
                time.sleep(0.1)

            # Gripper Control
            if self.gripper_command:
                if self.gripper_value == 0:
                    # Turn OFF suction.
                    self.pump_off()
                    print("[WARNING] SUCTION IS OFF!")
                    print("[WARNING] Please wait for 1 second.")
                    time.sleep(1)
                    self.gripper_value = 1
                elif self.gripper_value == 1:
                    # Turn ON suction.
                    self.pump_on()
                    print("[WARNING] SUCTION IS ON!")
                    print("[WARNING] Please wait for 1 second.")
                    time.sleep(1)
                    self.gripper_value = 0
                else:
                    continue

            # await asyncio.sleep(0.1)
            time.sleep(0.1)


def main():
    # Create a robot controller class instance
    rospy.init_node("joystick_subscriber_node", anonymous=True)
    robot_controller = RobotController()
    robot_controller.control_robot()


if __name__ == "__main__":
    main()
