import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('bp001_sim')
    control_pkg = get_package_share_directory('bp001_control')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(sim_pkg, 'urdf', 'bp001_sim.xacro')
    world_path = os.path.join(sim_pkg, 'worlds', 'empty.sdf')
    controller_config = os.path.join(
        control_pkg,
        'config',
        'balance_controller.yaml',
    )
    robot_description = xacro.process_file(urdf_path).toxml()
    gz_args = LaunchConfiguration('gz_args')
    dt = LaunchConfiguration('dt')
    lqr_g_over_l = LaunchConfiguration('lqr_g_over_l')
    lqr_theta_damping = LaunchConfiguration('lqr_theta_damping')
    lqr_vel_damping = LaunchConfiguration('lqr_vel_damping')
    lqr_input_gain = LaunchConfiguration('lqr_input_gain')
    lqr_motor_gain = LaunchConfiguration('lqr_motor_gain')
    q_pos = LaunchConfiguration('q_pos')
    q_vel = LaunchConfiguration('q_vel')
    q_pitch = LaunchConfiguration('q_pitch')
    q_pitch_rate = LaunchConfiguration('q_pitch_rate')
    r_input = LaunchConfiguration('r_input')
    max_wheel_speed = LaunchConfiguration('max_wheel_speed')
    command_alpha = LaunchConfiguration('command_alpha')
    max_cmd_step = LaunchConfiguration('max_cmd_step')
    pitch_deadband = LaunchConfiguration('pitch_deadband')
    pitch_alpha = LaunchConfiguration('pitch_alpha')
    pitch_rate_alpha = LaunchConfiguration('pitch_rate_alpha')
    wheel_vel_alpha = LaunchConfiguration('wheel_vel_alpha')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/default/model/bp001/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/bp001/joint/joint_lw/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/bp001/joint/joint_rw/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/default/model/bp001/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{
            'name': 'bp001',
            'topic': 'robot_description',
            'x': 0.0,
            'y': 0.0,
            'z': 0.15,
        }],
        output='screen',
    )

    controller = Node(
        package='bp001_control',
        executable='balance_controller',
        parameters=[{
            'use_sim_time': True,
        }, controller_config, {
            'dt': dt,
            'lqr_g_over_l': lqr_g_over_l,
            'lqr_theta_damping': lqr_theta_damping,
            'lqr_vel_damping': lqr_vel_damping,
            'lqr_input_gain': lqr_input_gain,
            'lqr_motor_gain': lqr_motor_gain,
            'q_pos': q_pos,
            'q_vel': q_vel,
            'q_pitch': q_pitch,
            'q_pitch_rate': q_pitch_rate,
            'r_input': r_input,
            'max_wheel_speed': max_wheel_speed,
            'command_alpha': command_alpha,
            'max_cmd_step': max_cmd_step,
            'pitch_deadband': pitch_deadband,
            'pitch_alpha': pitch_alpha,
            'pitch_rate_alpha': pitch_rate_alpha,
            'wheel_vel_alpha': wheel_vel_alpha,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value=f'-r {world_path}',
            description='Arguments forwarded to gz sim',
        ),
        DeclareLaunchArgument('dt', default_value='0.01'),
        DeclareLaunchArgument('lqr_g_over_l', default_value='22.0'),
        DeclareLaunchArgument('lqr_theta_damping', default_value='2.8'),
        DeclareLaunchArgument('lqr_vel_damping', default_value='3.0'),
        DeclareLaunchArgument('lqr_input_gain', default_value='28.0'),
        DeclareLaunchArgument('lqr_motor_gain', default_value='10.0'),
        DeclareLaunchArgument('q_pos', default_value='2.0'),
        DeclareLaunchArgument('q_vel', default_value='0.8'),
        DeclareLaunchArgument('q_pitch', default_value='80.0'),
        DeclareLaunchArgument('q_pitch_rate', default_value='8.0'),
        DeclareLaunchArgument('r_input', default_value='1.4'),
        DeclareLaunchArgument('max_wheel_speed', default_value='3.0'),
        DeclareLaunchArgument('command_alpha', default_value='0.06'),
        DeclareLaunchArgument('max_cmd_step', default_value='0.08'),
        DeclareLaunchArgument('pitch_deadband', default_value='0.03'),
        DeclareLaunchArgument('pitch_alpha', default_value='0.25'),
        DeclareLaunchArgument('pitch_rate_alpha', default_value='0.20'),
        DeclareLaunchArgument('wheel_vel_alpha', default_value='0.15'),
        gz_sim,
        rsp,
        bridge,
        controller,
        TimerAction(period=2.0, actions=[spawn]),
    ])
