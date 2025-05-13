from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

package_dir = get_package_share_directory("omni_description")
def generate_launch_description():
    # Define the path to the URDF file
    
    with open(os.path.join(package_dir, "urdf", "omni.urdf")) as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Set an environment variable (if necessary)
        SetEnvironmentVariable(
            name="GTDD_HOME",
            value="/home/fh-aachen/.3dsystems"
        ),
        
        # Omni state node
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "/world"},
                {"units": "mm"}
            ],
            remappings=[
            ("/phantom/joint_states", "/joint_states"),
        ],
            arguments=["--log-level", "debug"]
        ),
        
        # Omni broadcaster node
        Node(
            package='omni_broadcaster',
            executable='omni_broadcaster',
            name='broadcaster1',
            parameters=[
                {'framename': 'frame1'}
            ],
            arguments=["--log-level", "debug"]
        ),
        
        # Robot state publisher with URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output="screen",
            parameters=[{'robot_description': robot_desc}],
            #namespace="omni_robot" 
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_omni',
        #     output="screen",
        #     arguments=['0', '0', '0.5', '0', '0', '0', '1', 'base', 'omni_robot/base'])
    ])
