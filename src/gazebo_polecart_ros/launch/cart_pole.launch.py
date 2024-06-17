import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_polecart_ros'), 'launch')

    cart_pole_model_path = os.path.join(
        get_package_share_directory('gazebo_polecart_ros'), 'models', 'model.sdf'
    )

    ld = LaunchDescription([
        gazebo,
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'cart_pole', '-file', cart_pole_model_path, '-sdf']
        ),
        Node(
            package='cart_pole',
            executable='cartpole_controller',
            name='cartpole_controller',
            output='screen'
        )
    ])

    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_polecart.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld.add_action(spawn_cmd)
    return ld;

