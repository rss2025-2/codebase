from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    safety_controller_node = Node(
        package = "controllers",
        executable = "safety_controller",
        output = "screen",
        name = "safety_controller",
        parameters=[
            {"stopping_distance": 0.5},
            {"cutoff_distance": 0.5},
        ],
        remappings=[
            ("scan_topic", "/scan"),
            # for real racecar
            # ("drive_input_topic", "/vesc/low_level/ackermann_cmd"),
            # ("safety_drive_topic", "/vesc/low_level/input/safety"),
            # for testing in sim
            ("drive_input_topic", "/input_drive"),
            ("safety_drive_topic", "/drive"),
        ]
    )
    return LaunchDescription([
        safety_controller_node,
    ])