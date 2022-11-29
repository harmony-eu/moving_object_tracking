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


import launch_ros.actions

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument as LA
from launch.substitutions import LaunchConfiguration as LC

def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server', 'map_server_inflated']
    use_sim_time = False
    autostart = True
    node_output = 'screen'

    ll = list()

    location = LA('location', default_value="abb")  # prepended to map name
    location_conf = LC('location')
    rdir = str(get_package_share_directory("moving_object_tracking"))

    map_yaml = LA('map_yaml', default_value=[rdir, '/maps/', location_conf, '_map.yaml'])
    map_edited_yaml = LA('map_edited_yaml', default_value=[rdir, '/maps/', location_conf, '_map_edited.yaml'])


    ll.append(location)
    ll.append(map_yaml)
    ll.append(map_edited_yaml)

    # Nodes launching commands
    start_map_saver_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output = node_output,
            name='map_server',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'yaml_filename': LC('map_yaml')},
                        {'topic_name': 'map'},
                        {'frame_id': 'map'}])

    start_map_saver_inflated_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output = node_output,
            name='map_server_inflated',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'yaml_filename': LC('map_edited_yaml')},
                        {'topic_name': 'map_inflated'},
                        {'frame_id': 'map_inflated'}])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output = node_output,
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    static_tf_publisher = launch_ros.actions.Node(package = "tf2_ros", 
            output = node_output,
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "map", "map_inflated"])

    # static_tf_publisher_2 = launch_ros.actions.Node(package = "tf2_ros", 
    #         output = node_output,
    #         executable = "static_transform_publisher",
    #         arguments = ["-4.65", "-2.74", "0", "0", "0", "0.0", "map", "odom"])

    # iris_lama_ros = launch_ros.actions.Node(
    #         package='iris_lama_ros2',
    #         namespace='iris_lama_ros2',
    #         executable='loc2d_ros',
    #         name='loc2d_ros',
    #         remappings=[
    #            ('/scan', '/merged_scan'),
    #         ],
    #         output='screen',
    #         parameters=[{'use_map_topic': True}],
    #     )

    ll.append(start_map_saver_server_cmd)
    ll.append(start_map_saver_inflated_server_cmd)
    ll.append(start_lifecycle_manager_cmd)
    ll.append(static_tf_publisher)
    
    # ld.add_action(static_tf_publisher_2)
    # ld.add_action(iris_lama_ros)

    return LaunchDescription(ll)