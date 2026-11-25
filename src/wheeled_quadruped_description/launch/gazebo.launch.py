import os, re
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('wheeled_quadruped_description')
    world_file = os.path.join(pkg_share, 'worlds', 'obstacle_course.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_xml = re.sub(r'<!--.*?-->', '', robot_description_config.toxml(), flags=re.DOTALL)
    robot_xml = ' '.join(robot_xml.split())
    robot_description = {'robot_description': robot_xml}

    set_model_uri = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    node_robot_state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description])
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'wheeled_quadruped', '-z', '0.410', '-timeout', '60.0'], output='screen')

    load_joint_state_broadcaster = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    load_leg_controller = Node(package='controller_manager', executable='spawner', arguments=['leg_controller'], output='screen')
    load_wheel_controller = Node(package='controller_manager', executable='spawner', arguments=['wheel_controller'], output='screen')

    script_dir = os.path.expanduser('~/wheeled_quadruped_ws/src/wheeled_quadruped_description/scripts')
    start_posture = ExecuteProcess(cmd=['python3', os.path.join(script_dir, 'leg_command_bridge.py')], output='screen')

    return LaunchDescription([
        set_model_uri, gazebo, node_robot_state_publisher, spawn_entity,
        RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster, load_leg_controller, load_wheel_controller, start_posture])),
    ])
