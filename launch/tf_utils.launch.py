
import launch_ros.actions

from launch import LaunchDescription

def generate_launch_description():
    """
    Publish some static TFs
    Useful in case some links are missing from a rosbag
    """
    ll = list()
    static_tf_publisher = launch_ros.actions.Node(package = "tf2_ros", 
            output = 'screen',
            executable = "static_transform_publisher",
            arguments = ["0", "-0.032", "0", "1.5707963", "0.", "1.5707963", 
            "kinect_master_camera_base", 
            "kinect_master_rgb_camera_link_rotated"])

    ll.append(static_tf_publisher)
    return LaunchDescription(ll)
