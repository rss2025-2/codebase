from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    safety_controller_node = Node(
        package = "controllers",
        executable = "safety_controller",
        output = "screen",
        name = "safety_controller",
        parameters=[
            {"safety_cutoff_distance": 0.1},
            {"safety_scan_distance": 1.0},
            {"safety_scan_angle_pi": 0.5},
            {"point_num_thres": 5},
            {"car_length": 0.5},
            {"max_steering_angle": 2.5},
            {"max_deceleration": 5.0},
            {"max_danger_velocity": 0.5},
            {"forward_message": True},
            {"safety_time": 0.001}
        ],
        remappings=[
            ("scan_topic", "/scan"),
            # for real racecar
            # ("drive_input_topic", "/vesc/high_level/ackermann_cmd"),
            # ("safety_drive_topic", "/vesc/low_level/input/safety"),
            # for testing in sim
            ("drive_input_topic", "/input_drive"),
            ("safety_drive_topic", "/drive"),
        ]
    )
    keyboard_controller_node = Node(
        package = "controllers",
        executable = "keyboard_controller",
        output = "screen",
        name = "keyboard_controller",
        parameters=[
            {"max_steering_angle": 2.0},
            {"max_speed": 1.0},
            {"keypress_time": 0.001},
            {"window_size": 100},
            # {"persistent": True}
        ],
        remappings=[
            ("scan_topic", "/scan"),
            # for real racecar
            # ("keyboard_drive_topic", "/vesc/high_level/input/nav_0"),
            # for testing in sim
            ("keyboard_drive_topic", "/input_drive"),
        ]
    )
    return LaunchDescription([
        safety_controller_node,
        keyboard_controller_node
    ])
