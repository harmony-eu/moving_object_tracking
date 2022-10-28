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

import os
import launch_ros.actions

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    node_output = 'screen'

    laser_scan_merger = launch_ros.actions.Node(
            package='laser_scan_merger',
            executable='scans_merger_node',
            output = node_output,
            name='scans_merger',
            remappings=[
               ('/front_scan', '/front/scan'),
               ('/rear_scan', '/rear/scan'),
               ('/pcl', '/merged_pcl_2'),
            ],
            parameters=[
                {'active': True},
                {'publish_scan': False},
                {'publish_pcl2': True},
                {'ranges_num': 1000},
                {'min_scanner_range': 0.05},
                {'max_scanner_range': 10.0},
                {'min_x_range': -10.0},
                {'max_x_range': 10.0},
                {'min_y_range': -10.0},
                {'max_y_range': 10.0},
                {'fixed_frame_id':'map'},
                {'target_frame_id':'map'},
            ])

    # Nodes launching commands
    static_map_filter = launch_ros.actions.Node(
            package='laser_static_map_filter',
            namespace='',
            executable='laser_static_map_filter',
            name='laser_static_map_filter',
            remappings=[('/map', '/map_inflated'),
                        ('/map_updates', '/map_inflated_updates'),
                        ('/map_server/map', '/map_server_inflated/map'),
                        ('/scan', '/merged_pcl_2'),
                        ('/scan_filtered', '/scan_filtered_2'),]
        )

    obstacle_extractor = launch_ros.actions.Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            output = node_output,
            name='obstacle_extractor',
            remappings=[
               ('/scan', '/front/scan'),
               ('/pcl', '/scan_filtered'),
               ('/pcl2', '/scan_filtered_2'),
            ],
            parameters=[
                {'active': True},
                {'use_scan': False},
                {'use_pcl': False},
                {'use_pcl2': True},
                {'use_split_and_merge': True},
                {'circles_from_visibles': True},
                {'discard_converted_segments': True},
                {'transform_coordinates': True},

                {'min_group_points': 10},
                {'max_group_distance': 0.1},
                {'distance_proportion': 0.00628},
                {'max_split_distance': 0.2},
                {'max_merge_separation': 0.2},
                {'max_merge_spread': 0.2},
                {'max_circle_radius': 0.6},
                {'radius_enlargement': 0.3},
                {'frame_id':'map'},
            ])

    obstacle_tracker = launch_ros.actions.Node(
            package='obstacle_detector',
            executable='obstacle_tracker_node',
            output = node_output,
            name='obstacle_tracker',
            remappings=[
               ('/scan', '/front/scan'),
               ('/pcl', '/scan_filtered'),
            ],
            parameters=[
                {'active': True},
                {'loop_rate': 100.0},
                {'tracking_duration': 2.0},
                {'min_correspondence_cost': 0.6},
                {'std_correspondence_dev': 0.15},
                {'process_variance': 0.1},
                {'process_rate_variance': 0.1},
                {'measurement_variance': 1.0},

                {'frame_id':'map'},
                {'tracked_obstacles':'obstacles'},
                {'tracked_obstacles_visualization':'obstacles_visualization'},
            ])
  
    ld = LaunchDescription()

    ld.add_action(laser_scan_merger)
    ld.add_action(static_map_filter)
    ld.add_action(obstacle_extractor)
    ld.add_action(obstacle_tracker)
    return ld