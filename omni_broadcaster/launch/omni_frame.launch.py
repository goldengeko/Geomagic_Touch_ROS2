from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory("omni_description")

    # Load URDF from file
    with open(os.path.join(package_dir, "urdf", "omni.urdf"), 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Set an environment variable (optional, specific to your setup)
        SetEnvironmentVariable(
            name="GTDD_HOME",
            value="/home/fh-aachen/.3dsystems"
        ),
        
        # Omni state node
        Node(
            package="omni_common",
            executable="omni_state",
            name="omni_state_node",
            output="screen",
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "/world"},
                {"units": "mm"}
            ],
            # remappings=[
            #     ("/phantom/joint_states", "/joint_states"),
            # ],
            # arguments=["--ros-args", "--log-level", "debug"]
        ),
        
        # Omni broadcaster node
        Node(
            package='omni_broadcaster',
            executable='omni_broadcaster',
            name='broadcaster1',
            parameters=[
                {'framename': 'frame1'}
            ],
            # arguments=["--ros-args", "--log-level", "debug"]
        ),
        
        # Robot state publisher with URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output="screen",
            parameters=[{'robot_description': robot_desc}],
            namespace="phantom",
            # remappings=[
            #     ("joint_states", "phantom/joint_states")  # fixed typo: 'joit_states'
            # ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_omni',
            output="screen",
            arguments=[
                '1', '0', '0',  # XYZ
                '0', '0', '0', '1',  # Quaternion (WXYZ)
                'world', 'base'
            ]
        )
    ])
