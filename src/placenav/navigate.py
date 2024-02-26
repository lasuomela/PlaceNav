import numpy as np
import torch
import yaml
import copy
import argparse
import time

from pathlib import Path
from typing import List, Tuple, Dict
from threading import Lock

# ROS
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, Time

# Place recognition
from place_recognition.bayesian_querier import PlaceRecognitionTopologicalFilter
from place_recognition.sliding_window_querier import PlaceRecognitionSlidingWindowFilter
from place_recognition.feature_extractor import FeatureExtractor
from place_recognition import extract_database

import parser
from utils import to_numpy, load_model, read_image, get_image_transform, waypoints_to_ros_msg, bridge

# Visualization
from placenav_viz_msgs.msg import Viz

class PlaceNavNode:

    img_suffix = ['png', 'jpg', 'jpeg']

    def __init__(
            self,
            args: argparse.Namespace,
        ):
        # Set device to cuda if requested and available
        if args.device == "cuda" and torch.cuda.is_available():
            rospy.loginfo("Using CUDA")
            self.device = torch.device("cuda")
        else:
            rospy.loginfo("Using CPU")
            self.device = torch.device("cpu")

        # Get the image preprocessing transform for the neural network models
        self._image_transform = get_image_transform(args.img_size)

        # Load the topological map
        self._load_topomap(args.topomap_base_dir, args.topomap_dir)

        # Load robot config
        with args.robot_config_path.open(mode="r", encoding="utf-8") as f:
            robot_configs = yaml.safe_load(f)
        self.robot_config = robot_configs[args.robot]

        # Initialize place recognition
        if args.subgoal_mode == 'place_recognition':
            self._setup_place_recognition(
                args.model_config_path,
                args.model_weight_dir,
                args.pr_model,
                filter_mode=args.filter_mode,
                transition_model_window_lower=args.transition_model_window_lower,
                transition_model_window_upper=args.transition_model_window_upper,
                bayesian_filter_delta=args.filter_delta,
                recompute_db=args.recompute_place_recognition_db,
                )
            
        # Initialize the waypoint (and temporal distance) model
        self._setup_waypoint_model(
            args.model_config_path,
            args.model_weight_dir,
            args.wp_model,
            args.img_size,
            )
        
        self._setup_ros()

    def _load_topomap(self, topomap_images_base_dir, topomap_dir: Path):
        # List the topomap images with suffix img_suffix in the directory,
        # extract the filenames and sort them        
        self.topomap_img_dir = topomap_images_base_dir / topomap_dir

        topomap_images = []
        for img_suffix in self.img_suffix:
            topomap_images.extend(list(self.topomap_img_dir.glob(f"*.{img_suffix}")))
        topomap_filenames = [img.name for img in topomap_images]
        topomap_filenames = sorted(topomap_filenames, key=lambda filename: int(filename.split(".")[0]))

        # Load the topomap images
        map_size = len(topomap_filenames)
        topomap_images = []
        for i in range(map_size):
            image_path = self.topomap_img_dir / topomap_filenames[i]
            img = read_image(image_path)
            img = self._image_transform(img.astype(np.uint8)).unsqueeze(0)
            topomap_images.append(img)
        
        self.map_size = map_size
        self.topomap_images = topomap_images

    def _setup_place_recognition(
            self,
            model_config_path: Path,
            model_weight_dir: Path,
            place_recognition_model: str,
            filter_mode: str,
            transition_model_window_lower: int,
            transition_model_window_upper: int,
            bayesian_filter_delta: int,
            recompute_db: bool = False,
        ):
        # Load the place recognition model config
        with model_config_path.open(mode="r", encoding="utf-8") as f:
            confs = yaml.safe_load(f)
            conf = confs[place_recognition_model]
        conf['model']["checkpoint_path"] = model_weight_dir / conf['model']["checkpoint_path"]

        # Extract the global descriptors from the topomap images
        place_recognition_db_path = self.topomap_img_dir / f"global-feats-{place_recognition_model}.h5"

        if not place_recognition_db_path.exists():
            rospy.loginfo(f"Extracting features from topomaps in {self.topomap_img_dir}")
            extract_database.main(
                conf,
                self.topomap_img_dir,
                self._image_transform,
                self.topomap_img_dir,
                as_half=False,
                )
            
        elif recompute_db:
            rospy.loginfo(f"Recomputing features from topomaps in {self.topomap_img_dir}")
            place_recognition_db_path.unlink()
            extract_database.main(
                conf,
                self.topomap_img_dir,
                self._image_transform,
                self.topomap_img_dir,
                as_half=False,
                )

        extractor = FeatureExtractor(conf, self.device)

        if filter_mode == 'bayesian':
            self.place_recognition_querier = PlaceRecognitionTopologicalFilter(
                extractor,
                place_recognition_db_path,
                self.topomap_img_dir,
                delta=bayesian_filter_delta,
                window_lower=transition_model_window_lower,
                window_upper=transition_model_window_upper,
                )
            
        elif filter_mode == 'sliding_window':
            self.place_recognition_querier = PlaceRecognitionSlidingWindowFilter(
                extractor,
                place_recognition_db_path,
                str(self.topomap_img_dir))
        else:
            raise ValueError(f"Filter mode {filter_mode} not recognized")
        
    def _setup_waypoint_model(
            self,
            model_config_path: Path,
            model_weight_dir: Path,
            model: str,
            img_size: Tuple[int, int],
            ):
        # Load waypoint model config
        with model_config_path.open(mode="r", encoding="utf-8") as f:
            model_configs = yaml.safe_load(f)
        model_config = model_configs[model]
        model_config["img_size"] = img_size

        model_path = model_weight_dir / model_config["checkpoint_path"]
        if not model_path.exists():
            raise FileNotFoundError(f"Model weights not found at {model_path}")
        rospy.loginfo(f"Loading model weights from {model_path}")

        waypoint_model = load_model(
            model_path,
            model_config,
            self.device)
        waypoint_model.eval()
        self.waypoint_model = waypoint_model

        # Initialize the context queue
        self.context_size = model_config["context_size"]
        self.context_queue: List[Dict[Time, torch.Tensor]] = []

    def _setup_ros(self):
        self.obs_lock = Lock()

        obs_sub = rospy.Subscriber(
            self.robot_config['camera_topic'],
            Image,
            self.obs_callback,
            queue_size=1)
        
        self.waypoint_pub = rospy.Publisher(
            self.robot_config['waypoint_topic'],
            Float32MultiArray,
            queue_size=1)
        
        self.stop_sub = rospy.Subscriber(
            self.robot_config['stop_topic'],
            Bool,
            self.stop_callback,
            queue_size=1)
        
        self.goal_pub = rospy.Publisher(
            self.robot_config['reached_goal_topic'],
            Bool,
            queue_size=1)
        
        self.viz_pub = rospy.Publisher(
            self.robot_config['toponav_viz_info_topic'],
            Viz,
            queue_size=1)

        rospy.init_node("PlaceNavNode", anonymous=False)

        
    def obs_callback(self, msg: Image):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        with self.obs_lock:
            img = self._image_transform(img).unsqueeze(0)
            if len(self.context_queue) < self.context_size + 1:
                self.context_queue.append( {'timestamp': msg.header.stamp, 'img': img})
            else:            
                self.context_queue.pop(0)
                self.context_queue.append({'timestamp': msg.header.stamp, 'img': img})

    def stop_callback(self, msg: Bool):
        if msg.data:
            rospy.loginfo("Stopping...")
            rospy.signal_shutdown("shutdown")

    def navigate(self, args: argparse.Namespace):
        rospy.loginfo("Starting navigation")
        
        closest_node_idx = args.start_node_idx
        assert -1 <= args.goal_node_idx < self.map_size, "Invalid goal index"
        if args.goal_node_idx == -1:
            args.goal_node_idx = self.map_size - 1

        # Initialize iterative average calculation for runtime
        avg_runtime = 0
        avg_runtime_count = 0

        first_pass = True
        reached_goal = False
        rate = rospy.Rate(self.robot_config['frame_rate'])
        while not rospy.is_shutdown():

            start_time = time.time()

            with self.obs_lock:
                context = copy.deepcopy(self.context_queue)
            
            if len(context) >= self.context_size + 1:

                current_obs = context[-1]['img'].to(self.device)
                current_obs_timestamp = context[-1]['timestamp']
                # Cat the context images along the channel dimension
                context = torch.cat([obs['img'] for obs in context], dim=1).to(self.device)

                if (
                    first_pass and
                    args.subgoal_mode == 'place_recognition' and
                    args.filter_mode == 'bayesian'
                    ):
                    # Initialize the belief distribution of the
                    # Bayesian filter prior to first query
                    self.place_recognition_querier.initialize_model(current_obs)
                    first_pass = False
                
                if args.subgoal_mode == 'place_recognition':
                    
                    # Place recognition with Bayesian filter
                    if args.filter_mode == 'bayesian':
                        closest_node_idx, score = self.place_recognition_querier.match(current_obs)

                    # Place recognition with sliding window filter
                    elif args.filter_mode == 'sliding_window':
                        start = max(closest_node_idx - args.window_radius, 0)
                        end = min(closest_node_idx + args.window_radius +1, args.goal_node_idx +1)
                        closest_node_idx = self.place_recognition_querier.match(current_obs, start, end)

                    else:
                        raise ValueError(f"Filter mode {args.filter_mode} not recognized for subgoal mode {args.subgoal_mode}")
                    
                    # How many nodes ahead from closest node to choose the subgoal
                    subgoal_idx = min(closest_node_idx + args.lookahead, args.goal_node_idx)
                    sg_img = self.topomap_images[subgoal_idx].to(self.device)

                    # Predict the waypoints to subgoal
                    with torch.no_grad():
                        _, waypoints = self.waypoint_model(context, sg_img)
                    waypoints = to_numpy(waypoints[0])

                elif args.subgoal_mode == 'temporal_distance':
                    if args.filter_mode == 'bayesian':
                        raise NotImplementedError("Temporal distance subgoal mode cannot be used with the Bayesian filter")

                    # The regular GNM approach
                    elif args.filter_mode == 'sliding_window':
                        start = max(closest_node_idx - args.window_radius, 0)
                        end = min(closest_node_idx + args.window_radius +1, args.goal_node_idx +1)

                        distances = []
                        waypoints_candidates = []      
                        for sg_idx in range(start, end):
                            sg_img = self.topomap_images[sg_idx].to(self.device)
                            with torch.no_grad():
                                    dist, waypoint = self.waypoint_model(context, sg_img)

                            distances.append(to_numpy(dist[0]))
                            waypoints_candidates.append(to_numpy(waypoint[0]))

                        # look for closest node
                        closest_node = np.argmin(distances)

                        # choose subgoal and output waypoints
                        if distances[closest_node] > args.close_threshold:
                            sg_idx = closest_node
                        else:
                            sg_idx = min(closest_node + 1, len(waypoints_candidates) - 1)

                        waypoints = waypoints_candidates[sg_idx]
                            
                        # 'closest_node' is the index of the subgoal in the window,
                        # so we need to add 'start' to get the index in the topomap
                        closest_node_idx = start + closest_node
                        subgoal_idx = start + sg_idx
                    else:
                        raise ValueError(f"Filter mode {args.filter_mode} not recognized for subgoal mode {args.subgoal_mode}")
                else:
                    raise ValueError(f"Subgoal mode {args.subgoal_mode} not recognized")

                # Check if the goal has been reached
                reached_goal = closest_node_idx == args.goal_node_idx

                # Publish the waypoint
                chosen_waypoint = waypoints[args.target_waypoint_idx]
                waypoint_msg = Float32MultiArray()
                waypoint_msg.data = chosen_waypoint
                self.waypoint_pub.publish(waypoint_msg)

                # Update the average runtime
                loop_duration = time.time() - start_time
                avg_runtime = (avg_runtime * avg_runtime_count + loop_duration) / (avg_runtime_count + 1)
                avg_runtime_count += 1
                rospy.loginfo(f"Average runtime: {avg_runtime:.3f} s, Runtime: {loop_duration:.3f} s")

                # Publish information for visualization
                viz_msg = Viz()
                viz_msg.query_timestamp = current_obs_timestamp
                viz_msg.subgoal_idx = subgoal_idx
                viz_msg.waypoints = waypoints_to_ros_msg(waypoints)
                self.viz_pub.publish(viz_msg)

                self.goal_pub.publish(reached_goal)
                if reached_goal:
                    rospy.loginfo("Reached goal. Stopping...")
                    return
                
                rate.sleep()

if __name__ == '__main__':
    args = parser.parse_args()

    placenav_node = PlaceNavNode(args)
    placenav_node.navigate(args)


        





        
        