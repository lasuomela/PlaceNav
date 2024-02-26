'''
ROS node to visualize information from the topological navigation system.
'''

# ROS
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point

import argparse
import cv2
import numpy as np
import time
import yaml

from threading import Lock
from typing import Dict
from copy import deepcopy
from pathlib import Path
from vidgear.gears import WriteGear

import placenav
from placenav.place_recognition.gallery_db import PlaceRecognitionDBHandler
from utils import project_waypoint, bridge
from placenav_viz_msgs.msg import Viz


class TopoNavVisualizationNode:

    def __init__(self, args):

        self._viz_image_size = args.viz_img_size
        self._show = args.show
        self._save = args.save
        self._display_waypoints = args.display_waypoints

        topomap_img_dir = args.topomap_base_dir / args.topomap_dir
        place_recognition_db_path = topomap_img_dir / f"global-feats-{args.pr_model}.h5"
        
        self._obs_buffer_size = 10
        self._obs_buffer: Dict[Int32, Image] = {}
        self._subgoal_idx = None
        self._query_timestamp = None

        # Load the robot config
        with args.robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        self.robot_config = robot_configs[args.robot]

        # Load the camera config
        with args.camera_config_path.open(mode="r", encoding="utf-8") as f:
            self.camera_config = yaml.safe_load(f)

        # Check that the place recognition model exists
        # If not, wait for a bit in case the main node is
        # still extracting the images
        wait_count = 0
        while not place_recognition_db_path.exists():
            wait_count += 1
            if wait_count > 10:
                raise FileNotFoundError(f"Place recognition model {place_recognition_db_path} does not exist.")
            
            rospy.loginfo(f"Place recognition model {place_recognition_db_path} does not exist. \
                          Waiting for the main node to extract the images...")
            time.sleep(1)

        # Load the gallery db
        self.db_handler = PlaceRecognitionDBHandler(
            place_recognition_db_path,
            topomap_img_dir,
        )
        # Load the gallery db images into memory
        topomap_image_filepaths = self.db_handler.get_filepaths()
        self._topomap_images = [cv2.imread(str(img_path)) for img_path in topomap_image_filepaths]

        if self._display_waypoints:
            with args.cam_cal_path.open(mode="r", encoding="utf-8") as f:
                calibration = yaml.safe_load(f)
            self._cal_K = np.array(calibration["camera_matrix"], dtype=np.float64)
            D = np.array(calibration["dist_coeff"], dtype=np.float64)

            # Usage of distortion coefficients
            # requires accurate calibration
            if args.use_distortions:
                self._cal_D = D
            else:
                self._cal_D = np.zeros_like(D)
            self._use_distortions = args.use_distortions

        if self._save:
            save_dir = args.video_save_base_path / args.topomap_dir
            if not save_dir.exists():
                save_dir.mkdir(parents=True, exist_ok=True)
            
            save_path = save_dir / f"{args.topomap_dir}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.mp4"
            output_params = {
                "-vcodec": "libx264",
                "-pix_fmt": "yuv420p",
                "-crf": 0,
                "-preset": "fast",
                "-s": f"{2*self._viz_image_size[0]}x{self._viz_image_size[1]}",
                "-input_framerate": self.robot_config['frame_rate'],
            }
            self._writer = WriteGear(
                output=save_path.as_posix(),
                compression_mode=True,
                logging=False,
                **output_params,
            )
            self._save_name = save_path.name
        
        self._init_time = time.time()
        self._setup_ros()


    def _setup_ros(self):
        self._obs_lock = Lock()
        self._info_lock = Lock()

        obs_sub = rospy.Subscriber(
            self.robot_config['camera_topic'],
            Image,
            self._obs_callback,
            queue_size=10,
            buff_size=65536*10,
        )
        viz_info_sub = rospy.Subscriber(
            self.robot_config['toponav_viz_info_topic'],
            Viz,
            self._topomap_viz_info_callback,
            queue_size=10,
        )
        self.stop_pub = rospy.Publisher(
            self.robot_config['stop_topic'],
            Bool,
            queue_size=1)
        rospy.init_node('toponav_visualization_node', anonymous=False)

    def _obs_callback(self, msg: Image):
        with self._obs_lock:
            if len(self._obs_buffer) > self._obs_buffer_size:
                # Remove the oldest observation
                del self._obs_buffer[(next(iter(self._obs_buffer)))]

            self._obs_buffer[msg.header.stamp.to_sec()] = msg

    def _topomap_viz_info_callback(self, msg: Viz):
        with self._info_lock:
            self._subgoal_idx = msg.subgoal_idx
            self._query_timestamp = msg.query_timestamp.to_sec()
            self._waypoints = msg.waypoints

    def _overlay_waypoints(self, img, waypoints):

        # Project the waypoints onto the image coords
        waypoints_projected = []
        for waypoint in waypoints:
            x, y = project_waypoint(waypoint, self.robot_config['camera_height'], self._cal_K, self._cal_D)
            waypoints_projected.append((x, y))
        
        # Additional point to enable line drawing
        # between the waypoints
        if self._use_distortions:
            # Add the bottom middle point as the first waypoint
            pt = (self.camera_config['image_width']//2, self.camera_config['image_height'])
            waypoints_projected = [pt] + waypoints_projected
        else:
            # Add point close to the origo as the first waypoint
            # and project it onto the image plane
            pt = project_waypoint(Point(x=0.05, y=0.0, z=0.0), self.robot_config['camera_height'], self._cal_K, self._cal_D)
            waypoints_projected = [pt] + waypoints_projected

        # Plot the waypoints as lines
        x_prev, y_prev = None, None
        for x, y in waypoints_projected:
            try:
                if x_prev is not None and y_prev is not None:
                    cv2.line(img, (x_prev, y_prev), (x, y), (0, 255, 255), 6)
            except:
                raise Exception(f"type x: {type(x)}, type y: {type(y)}")
            x_prev, y_prev = x, y

        # Plot the waypoints as circles
        for x, y in waypoints_projected[1:]:
            cv2.circle(img, (x, y), 7, (0, 128, 255), -1)
        return img

    def run(self):
        rospy.loginfo("Starting visualization node. Press 'Q' to stop if the visualization is shown.")
        start_time = time.time()
        started_writing = False
        while not rospy.is_shutdown():
            # Wait for 1sec to fill the buffer
            if time.time() - self._init_time > 1:

                if self._obs_buffer and (self._subgoal_idx is not None) and (self._query_timestamp is not None):
                    start_time = time.time()
                    started_writing = True
                    with self._info_lock:
                        with self._obs_lock:
                            # Get the latest observation
                            latest_obs_msg = self._obs_buffer[next(iter(reversed(self._obs_buffer)))]
                            # Get the latest query image
                            query_obs_msg = self._obs_buffer[self._query_timestamp]

                        # Get the topomap image corresponding to the query image
                        subgoal_img = self._topomap_images[self._subgoal_idx]
                        waypoints = deepcopy(self._waypoints)

                        # Make sure we don't use the same query image again
                        self._subgoal_idx = None
                        self._query_timestamp = None
                        self._waypoints = None
                    
                    query_obs_img = bridge.imgmsg_to_cv2(query_obs_msg, desired_encoding="bgr8")

                    if self._display_waypoints:
                        query_obs_img = self._overlay_waypoints(query_obs_img, waypoints)

                    subgoal_img = cv2.resize(subgoal_img, self._viz_image_size)
                    query_obs_img = cv2.resize(query_obs_img, self._viz_image_size)

                    # Add text to the images
                    cv2.putText(subgoal_img, 'Subgoal', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(query_obs_img, 'Query', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

                    # Add borders to the images
                    subgoal_img = cv2.copyMakeBorder(subgoal_img,5,5,5,5,cv2.BORDER_CONSTANT,value=[0,0,0])
                    query_obs_img = cv2.copyMakeBorder(query_obs_img,5,5,5,5,cv2.BORDER_CONSTANT,value=[0,0,0])

                    # Concatenate the images
                    panel = cv2.hconcat([subgoal_img, query_obs_img])

                    # Save the visualization
                    # to disk as a video
                    if self._save:
                        self._writer.write(panel.astype('uint8'))

                    # Show the video on screen
                    if self._show:
                        cv2.imshow('Topological Navigation', panel)
                        key = cv2.waitKey(100) % 256
                        if key == ord('q'):
                            self.stop_pub.publish(Bool(data=True))
                            break

                # Check if the navigation node is still publishing
                if started_writing:
                    if time.time() - start_time > 1/self.robot_config['frame_rate'] * 3:
                        rospy.loginfo(f"Topic {self.robot_config['camera_topic']} not publishing anymore. Shutting down...")
                        rospy.signal_shutdown("shutdown")

        if self._save:
            self._writer.close()
            rospy.loginfo(f"Saved visualization to {self._save_name}")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        type=str,
        help="name of the robot to control",
        required=True,
    )
    parser.add_argument(
        "--robot-config-path",
        type=Path,
        help="path to config of the robot to control",
        required=True,
    )
    parser.add_argument(
        "--topomap-base-dir",
        type=Path,
        help="path to the topomap base directory",
        required=True,
    )
    parser.add_argument(
        "--topomap-dir",
        "-d",
        type=str,
        help="name of the topomap directory",
        required=True,
    )
    parser.add_argument(
        "--camera-config-path",
        type=Path,
        help="path to the camera configuration file",
    )
    parser.add_argument(
        "--cam-cal-path",
        type=Path,
        help="path to the camera calibration file",
    )
    parser.add_argument(
        "--pr-model",
        default="cosplace",
        type=str,
        help="place recognition model name (hint: check config/models.yaml) (default: cosplace)",
    )
    parser.add_argument(
        "--viz-img-size",
        type=int,
        nargs=2,
        default=[640, 480],
    )
    parser.add_argument(
        "--display-waypoints",
        action="store_true",
        help="If the waypoints should be displayed",
    )
    parser.add_argument(
        "--use-distortions",
        action="store_true",
        help="If the camera distortions should be \
        used when displaying the waypoints",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="If the visualization should be shown",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="If the visualization should be saved",
    )
    parser.add_argument(
        "--video-save-base-path",
        type=Path,
        help="Base path to save the visualization video",
        default=Path(placenav.__path__[0]).parent.parent / "visualization_videos",
    )
    args, _ = parser.parse_known_args()

    topo_viz_node = TopoNavVisualizationNode(args)
    topo_viz_node.run()
    cv2.destroyAllWindows()
    rospy.signal_shutdown("shutdown")
    exit(0)



                


