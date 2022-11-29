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


def generate_launch_description():
    largs = list()

    yolo_input_topics = str([
        '/multicam/back/image_color/compressed',
        '/multicam/left/image_color/compressed',
        '/multicam/right/image_color/compressed'
    ])

    yolo_weights = 'models/ABB.pt'
    yolo_data = 'models/abb_model.yaml'

    cameras = [
        '/multicam/back',
        '/multicam/right',
        '/multicam/left'
    ]
    camera_links = [
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
            {'lidar_height': 0.32},
            {'image_plane_threshold': 0.3},
            {'yolo_pixel_confidence_margin': 20},
            {'debug_vis_projection': False},
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