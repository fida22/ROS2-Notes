
import os
from launch.substitutions import Command
from launch.actions import SetEnvironmentVariable

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    model_path = '/usr/share/gazebo-11/models'
    set_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path)
    pkg_name = 'four_wheel_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'final_wall.world')
    xacro_path = os.path.join(pkg_share,'urdf','four_wheel_robot.urdf.xacro')

    # 1. Start Gazebo with your custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # 2. Robot State Publisher (loads the xacro)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_path
            ])
        }]
    )

    # 3. Spawn robot in Gazebo (delayed to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                arguments=[
                    '-entity', 'four_wheel_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    #4 teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in a separate terminal window
        remappings=[
            ('/cmd_vel', '/cmd_vel_input')  # So obstacle node can intercept
        ]   
    )
    # 5. Obstacle Stop Node (custom Python node)
    obstacle_stop_node = Node(
        package='four_wheel_robot',
        executable='obstacle_stop',
        name='obstacle_stop_node',
        output='screen'
    )


    return LaunchDescription([
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        teleop_node,
        obstacle_stop_node
    ])
