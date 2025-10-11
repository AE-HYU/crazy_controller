from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    controller_mode_arg = DeclareLaunchArgument(
        'controller_mode',
        default_value='MAP',
        description='Mode of operation: MAP only (simplified for time trial racing)'
    )

    mod_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description='Launch mode: real (use /pf/pose/odom), sim (use /ego_racecar/odom), sim_pf (use /pf/pose/odom)'
    )

    # Dynamic parameter paths based on mod
    l1_params_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/l1_params_sim.yaml' if '", 
        LaunchConfiguration('mod'), "' in ['sim', 'sim_pf'] else '",
        FindPackageShare('crazy_controller'), "/config/l1_params.yaml'"
    ])
    
    lookup_table_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/SIM_linear_lookup_table.csv' if '", 
        LaunchConfiguration('mod'), "' in ['sim', 'sim_pf'] else '",
        FindPackageShare('crazy_controller'), "/config/NUC4_pacejka_lookup_table.csv'"
    ])

    # Dynamic odom topic based on mod
    odom_topic = PythonExpression([
        "'/ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else '/pf/pose/odom'"
    ])

    # Controller node
    controller_node = Node(
        package='crazy_controller',
        executable='controller_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('controller_mode'),
            'l1_params_path': l1_params_path,
            'lookup_table_path': lookup_table_path,
            'use_sim_time': PythonExpression([
                "'true' if '", LaunchConfiguration('mod'), "' in ['sim', 'sim_pf'] else 'false'"
            ])
        }],
        remappings=[
            ('/planned_path', '/planned_waypoints'),
            ('/odom', odom_topic),
            ('/frenet/odom', '/car_state/frenet/odom'),
        ]
    )

    return LaunchDescription([
        controller_mode_arg,
        mod_arg,
        controller_node,
    ])
