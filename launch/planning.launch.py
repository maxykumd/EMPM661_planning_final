import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_wa     = get_package_share_directory('wa_birrt_star')
    pkg_gz_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg_wa, 'worlds', 'planning_world.world')

    urdf_file = os.path.join(pkg_nav2, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 1. Gazebo server with our world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # 2. Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        # 3. Robot state publisher with URDF
        Node(
            package    = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name       = 'robot_state_publisher',
            output     = 'screen',
            parameters = [{
                'use_sim_time':      True,
                'robot_description': robot_description
            }],
        ),

        # 4. Spawn TurtleBot3 using TurtleBot3's own spawn launch
        # This correctly sets up the differential drive controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gz, 'launch',
                             'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '0.3',
                'y_pose': '5.5',
            }.items()
        ),

        # 5. Static obstacle publisher — feeds obstacle positions to planner
        Node(
            package    = 'wa_birrt_star',
            executable = 'moving_obstacle_pub',
            name       = 'moving_obstacle_publisher',
            output     = 'screen',
            parameters = [{'use_sim_time': True}]
        ),

        # 6. planner node
        Node(
            package    = 'wa_birrt_star',
            executable = 'planner_node',
            name       = 'birrt_star_planner',
            output     = 'screen',
            parameters = [{'use_sim_time': True}]
        ),

        # 7. Path follower node
        Node(
            package    = 'wa_birrt_star',
            executable = 'path_follower',
            name       = 'path_follower',
            output     = 'screen',
            parameters = [{'use_sim_time': True}]
        ),

        #7. RViz with pre-configured layout
        Node(
            package    = 'rviz2',
            executable = 'rviz2',
            name       = 'rviz2',
            arguments  = ['-d', os.path.join(pkg_wa, 'rviz', 'planning.rviz')],
            output     = 'screen',
            parameters = [{'use_sim_time': True}]
        ),
        Node(
            package='wa_birrt_star', executable='viz_node',
            name='viz_node', output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        

    ])