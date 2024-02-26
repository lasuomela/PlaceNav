import yaml
import argparse
from pathlib import Path

# ROS
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Joy2CmdNode:
    def __init__(self, robot, robot_config_path, joy_config_path):
        self.vel_msg = Twist()
        self.subgoal_msg = Int16()
        self.load_config(robot, robot_config_path, joy_config_path)
        self.RATE = 9 # Velocity command publish rate

        self.init_ros()
        self.button = None
        self.button_sg_toggle = None

    def load_config(self, robot, robot_config_path, joy_config_path):

        with robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        robot_config = robot_configs[robot]

        self.MAX_V = robot_config["max_v"]
        self.MAX_W = robot_config["max_w"]
        self.VEL_TOPIC = robot_config["vel_teleop_topic"]

        with open(joy_config_path, "r") as f:
            joy_config = yaml.safe_load(f)

        self.DEADMAN_SWITCH = joy_config["deadman_switch"]
        self.LIN_VEL_BUTTON = joy_config["lin_vel_button"]
        self.ANG_VEL_BUTTON = joy_config["ang_vel_button"]

    def init_ros(self):
        rospy.init_node("Joy2RobotCmd", anonymous=False)
        self.vel_pub = rospy.Publisher(self.VEL_TOPIC, Twist, queue_size=1)
        self.subgoal_pub = rospy.Publisher('/subgoal_idx', Int16, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.callback_joy)
        self.rate = rospy.Rate(self.RATE)
        rospy.loginfo("Registered with master node. Waiting for joystick input...")

    def callback_joy(self, data: Joy):
        self.button = data.buttons[self.DEADMAN_SWITCH]
        self.button_sg_toggle = data.buttons[0]

        if self.button_sg_toggle is not None:
            self.subgoal_msg.data += int(data.axes[7])

        if self.button is not None:
            self.vel_msg.linear.x = self.MAX_V * data.axes[self.LIN_VEL_BUTTON]
            self.vel_msg.angular.z = self.MAX_W * data.axes[self.ANG_VEL_BUTTON]
        else:
            self.vel_msg = Twist()
    
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.button:
                self.vel_pub.publish(self.vel_msg)
                self.subgoal_pub.publish(self.subgoal_msg)
                self.rate.sleep()

def parse_args():
    parser = argparse.ArgumentParser(description="ROS node to transform joystick commands to linear and angular velocities for robot control")
    parser.add_argument(
        "--robot",
        type=str,
        help="name of the robot to control",
    )
    parser.add_argument(
        "--robot-config-path",
        type=Path,
        help="path to config of the robot to control",
    )
    parser.add_argument(
        "--joy-config-path",
        type=Path,
        help="path to config of the joystick to operate",
    )
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    joy_node = Joy2CmdNode(args.robot, args.robot_config_path, args.joy_config_path)
    joy_node.main_loop()