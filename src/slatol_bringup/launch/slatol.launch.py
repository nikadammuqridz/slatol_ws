import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slatol = get_package_share_directory('slatol_description')
    
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', 'empty.sdf'], output='screen')
    
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'slatol', '-topic', 'robot_description', '-z', '0.65'],
                 output='screen')

    xacro_file = os.path.join(pkg_slatol, 'urdf', 'slatol.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': os.popen(f'xacro {xacro_file}').read()}],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                   '/slatol/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[os.path.join(pkg_slatol, 'config', 'slatol_controllers.yaml')],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )
    
    spawner = Node(package="controller_manager", executable="spawner", arguments=["slatol_position_controller", "--controller-manager", "/controller_manager"])
    broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    
    planner = Node(package='slatol_bringup', executable='slatol_planner', output='screen')

    return LaunchDescription([
        gazebo, robot_state_publisher, spawn, bridge, 
        controller_manager, spawner, broadcaster, planner
    ])