from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    safety_controller_node = Node(
        package = "controllers",
        executable = "safety_controller",
        output = "screen",
        name = "safety_controller",
        parameters=[
            {"safety_cutoff_distance": 0.5},
            {"danger_zone_points_thres": 10},
            {"max_steering_angle": 2.0},
            {"max_deceleration": 10.0},
            {"max_danger_velocity", 1.0},
            {"car_length": 1.0},
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