import argparse
import os
import shutil
import pickle
import copy
import numpy as np
import yaml
import time
from pathlib import Path
import cv2

# ROS
import rospy
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
from utils import bridge, odometry_msg_to_numpy, to_local_coords


class TopomapGeneratorNode:
    def __init__(self, args):
        self.obs_img = None
        self.latest_odometry = None
        self.obs_odometry = None
        self.previous_odometry = None
        self.traj_data = {'position': [], 'yaw': [], 'relative_position': [], 'relative_yaw': []}

        with args.robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        robot_config = robot_configs[args.robot]

        self.topomap_name_dir = args.topomap_directory / args.route_name
        if not os.path.isdir(self.topomap_name_dir):
            os.makedirs(self.topomap_name_dir)
        else:
            rospy.loginfo(f"{self.topomap_name_dir} already exists. Removing previous images...")
            self.remove_files_in_dir(self.topomap_name_dir)

        rospy.init_node("CREATE_TOPOMAP", anonymous=False)
        self.image_topic = robot_config["camera_topic"]
        self.odometry_topic = robot_config["odom_topic"]
    
        # Subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.callback_obs, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.callback_joy)
        self.odom_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.callback_odometry)

        assert args.dt > 0, "dt must be positive"
        self.rate = rospy.Rate(1/args.dt)
        rospy.loginfo("Registered with master node. Waiting for images...")

    def remove_files_in_dir(self, dir_path):
        for f in os.listdir(dir_path):
            file_path = os.path.join(dir_path, f)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                rospy.loginfo("Failed to delete %s. Reason: %s" % (file_path, e))

    def callback_obs(self, msg):
        if self.latest_odometry is not None:
            self.obs_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            self.obs_odometry = odometry_msg_to_numpy(self.latest_odometry)

    def callback_joy(self, msg):
        if msg.buttons[0]:
            rospy.signal_shutdown("shutdown")

    def callback_odometry(self, msg):
        self.latest_odometry = msg

    def main_loop(self):
        i = 0
        start_time = float("inf")
        while not rospy.is_shutdown():
            if self.obs_img is not None:
                odometry = copy.deepcopy(self.obs_odometry)

                if self.previous_odometry is None:
                    pose_delta = None
                else:
                    pose_delta = to_local_coords(odometry, self.previous_odometry, self.previous_odometry[2]).astype(float)
                self.previous_odometry = copy.deepcopy(odometry)

                self.traj_data['position'].append(odometry[:2])
                self.traj_data['yaw'].append(odometry[2])
                if pose_delta is not None:
                    self.traj_data['relative_position'].append(pose_delta[:2])
                    self.traj_data['relative_yaw'].append(pose_delta[2])
                else:
                    self.traj_data['relative_position'].append(np.array([0, 0]).astype(float))
                    self.traj_data['relative_yaw'].append(0)

                cv2.imwrite(os.path.join(self.topomap_name_dir, f"{i}.jpg"), cv2.cvtColor( self.obs_img, cv2.COLOR_RGB2BGR))
                rospy.loginfo(f"published image {i}")
                i += 1
                self.rate.sleep()
                start_time = time.time()
                self.obs_img = None
            if time.time() - start_time > 2 * args.dt:
                rospy.loginfo(f"Topic {self.image_topic} not publishing anymore. Shutting down...")
                rospy.signal_shutdown("shutdown")

        with open(os.path.join(self.topomap_name_dir, 'traj_data.pkl'), 'wb') as f:
            pickle.dump(self.traj_data, f)

def parse_args():
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from rosbags"
    )
    parser.add_argument(
        "--robot",
        type=str,
        help="robot name",
    )
    parser.add_argument(
        "--robot_config_path",
        type=Path,
        help="path to config of the robot to control",
    )
    parser.add_argument(
        "--route_name",
        "-r",
        default="topomap",
        type=str,
        help="path to topological map images in ../topomaps/images directory (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=3.,
        type=float,
        help=f"time between sampled images (default: 3.0)",
    )
    parser.add_argument(
        "--topomap_directory",
        type=Path,
        default="placenav/topomaps/images",
        help="path to topomap directory (default: placenav/topomaps/images)",
    )
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    topomap_node = TopomapGeneratorNode(args)
    topomap_node.main_loop()