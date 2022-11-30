#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LA
from launch.substitutions import LaunchConfiguration as LC


def generate_launch_description():
    largs = list()

    lidar_height = LA('lidar_height', default_value='-2000.0')

    yolo_input_topics = str([
        '/kinect_master/rgb_rotated_rectified/image_raw',
        # TODO: The current YOLO model is trained on distorted images
        #       Should be trained on rectified images
        #       Then these topics would be /multicam/back/image_rect
        '/multicam/back/image_color/compressed',
        '/multicam/left/image_color/compressed',
        '/multicam/right/image_color/compressed'
    ])

    yolo_weights = 'models/ABB.pt'
    yolo_data = 'models/abb_model.yaml'

    cameras = [
        '/kinect_master/rgb_rotated',
        '/multicam/back',
        '/multicam/right',
        '/multicam/left'
    ]

    debug_topics = [
        'none',
        # '/kinect_master/rgb_rotated_rectified/image_raw',
        # '/multicam/back/image_color/compressed',
        # '/multicam/right/image_color/compressed',
        # '/multicam/left/image_color/compressed',
    ]
    camera_links = [
        'kinect_master_rgb_camera_link_rotated',
        'flir_back_link',
        'flir_right_link',
        'flir_left_link'
    ]

    # --- YOLO NODE
    launch_args = {
        "weights": yolo_weights,
        "data_yaml": yolo_data,
        "input_topics": yolo_input_topics,
        "name": "yolo"
    }
    launch_path = PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('yolo_eth_ros'), 'detector.launch.py'])])
    node_yolo = IncludeLaunchDescription(launch_path, launch_arguments=launch_args.items())
    largs.append(node_yolo)

    # --- SEMANTIC TRACKING NODE
    largs.append(lidar_height)
    
    node_semantics = Node(
        package='semantic_tracking',
        executable='node',
        name='semantic_tracking',
        output='screen',
        parameters=[
            {'cameras': cameras},
            {'camera_links': camera_links},
            # see semantic_tracking repo for explanation of arguments
            {'image_floor_margin': 0.9},
            {'intersection_threshold': 0.5},
            # value < -1000 to tell the node not to overwrite the z-coordinate of obstacles
            # and instead use the z in the message
            {'lidar_height': LC('lidar_height')},
            {'image_plane_threshold': 0.3},
            {'yolo_pixel_confidence_margin': 20},
            {'debug_topics': debug_topics},
            {'rectified_input': False},

        ],
        remappings=[
            ('yolov5', 'yolo/yolo_eth_ros/detections'),
            ('tracked_obstacles', 'tracked_obstacles'),
            ('tracked_obstacles_visualization', 'tracked_obstacles_visualization'),
            ('tracked_semantic_obstacles', '/tracked_semantic_obstacles'),
            ('tracked_semantic_obstacles_visualization', '/tracked_semantic_obstacles_visualization'),
        ]
    )
    largs.append(node_semantics)

    return LaunchDescription(largs)