import numpy as np
import yaml
from typing import Tuple
import argparse
from pathlib import Path

# ROS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool

class PDControllerNode:
    def __init__(self, robot: str, robot_config_path: Path):
        self.vel_msg = Twist()
        self.reached_goal = False

        with robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        robot_config = robot_configs[robot]

        self.MAX_V = robot_config["max_v"]
        self.MAX_W = robot_config["max_w"]
        self.DT = 1 / robot_config["frame_rate"]
        self.RATE = 9 # Rate for velocity publishing
        self.EPS = 1e-8

        rospy.init_node("PD_CONTROLLER", anonymous=False)
        self.waypoint_sub = rospy.Subscriber(robot_config['waypoint_topic'], Float32MultiArray, self.callback_drive, queue_size=1)
        self.reached_goal_sub = rospy.Subscriber(robot_config['reached_goal_topic'], Bool, self.callback_reached_goal, queue_size=1)
        self.stop_sub = rospy.Subscriber(robot_config['stop_topic'], Bool, self.callback_stop, queue_size=1)
        self.vel_out = rospy.Publisher(robot_config["vel_navi_topic"], Twist, queue_size=1)
        self.rate = rospy.Rate(self.RATE)
        rospy.loginfo("Registered with master node. Waiting for waypoints...")

    def clip_angle(self, theta) -> float:
        """Clip angle to [-pi, pi]"""
        theta %= 2 * np.pi
        if -np.pi < theta < np.pi:
            return theta
        return theta - 2 * np.pi

    def pd_controller(self, waypoint: np.ndarray) -> Tuple[float, float]:
        """PD controller for the robot"""
        assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
        if len(waypoint) == 2:
            dx, dy = waypoint
        else:
            dx, dy, hx, hy = waypoint

        if len(waypoint) == 4 and np.abs(dx) < self.EPS and np.abs(dy) < self.EPS:
            v = 0
            w = self.clip_angle(np.arctan2(hy, hx)) / self.DT
        elif np.abs(dx) < self.EPS:
            v = 0
            w = np.sign(dy) * np.pi / (2 * self.DT)
        else:
            v = dx / self.DT
            w = np.arctan(dy / dx) / self.DT

        v = np.clip(v, 0, self.MAX_V)
        w = np.clip(w, -self.MAX_W, self.MAX_W)

        return v, w

    def callback_drive(self, waypoint_msg: Float32MultiArray):
        """Callback function for the waypoint subscriber"""
        waypoint = waypoint_msg.data
        v, w = self.pd_controller(waypoint)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = v
        self.vel_msg.angular.z = w
        rospy.loginfo("publishing new vel")

    def callback_stop(self, stop_msg: Bool):
        if stop_msg.data == True:
            self.vel_msg = Twist()
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            rospy.loginfo("Stopped")

    def callback_reached_goal(self, reached_goal_msg: Bool):
        """Callback function for the reached goal subscriber"""
        self.reached_goal = reached_goal_msg.data

    def main_loop(self):
        while not rospy.is_shutdown():
            self.vel_out.publish(self.vel_msg)
            if self.reached_goal:
                self.vel_msg = Twist()
                self.vel_out.publish(self.vel_msg)
                rospy.loginfo("Reached goal! Stopping...")
                return
            self.rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=f"PD Controller for robot navigation"
    )
    parser.add_argument(
        "--robot",
        "-r",
        type=str,
        required=True,
        help="robot name",
    )
    parser.add_argument(
        "--robot-config-path",
        type=Path,
        required=True,
        help="path to config of the robot to control",
    )
    args = parser.parse_args()

    pd_controller_node = PDControllerNode(args.robot, args.robot_config_path)
    pd_controller_node.main_loop()