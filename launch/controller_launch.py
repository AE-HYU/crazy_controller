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
        description='Mode of operation: MAP, PP, or AUG'
    )

    mod_arg = DeclareLaunchArgument(
        'mod',
        default_value='real',
        description='Launch mode: real (TF map->base_link + /odom), sim (TF map->ego_racecar/base_link + /ego_racecar/odom)'
    )

    # Dynamic parameter paths based on mod
    l1_params_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/l1_params_sim.yaml' if '",
        LaunchConfiguration('mod'), "' == 'sim' else '",
        FindPackageShare('crazy_controller'), "/config/l1_params.yaml'"
    ])

    pp_params_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/pp_params_sim.yaml' if '",
        LaunchConfiguration('mod'), "' == 'sim' else '",
        FindPackageShare('crazy_controller'), "/config/pp_params.yaml'"
    ])

    aug_params_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/aug_params_sim.yaml' if '",
        LaunchConfiguration('mod'), "' == 'sim' else '",
        FindPackageShare('crazy_controller'), "/config/aug_params.yaml'"
    ])

    lookup_table_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/SIM_linear_lookup_table.csv' if '", 
        LaunchConfiguration('mod'), "' in ['sim', 'sim_pf'] else '",
        FindPackageShare('crazy_controller'), "/config/NUC4_pacejka_lookup_table.csv'"
    ])

    # Dynamic odom topic based on mod
    odom_topic = PythonExpression([
        "'/ego_racecar/odom' if '", LaunchConfiguration('mod'), "' == 'sim' else '/odom'"
    ])

    # Dynamic TF map and base_link frames based on mod
    map_frame = PythonExpression([
        "'map' if '", LaunchConfiguration('mod'), "' == 'sim' else 'mcl_map'"
    ])

    base_link_frame = PythonExpression([
        "'ego_racecar/base_link' if '", LaunchConfiguration('mod'), "' == 'sim' else 'base_link'"
    ])

    # Controller node (MAP)
    map_controller_node = Node(
        package='crazy_controller',
        executable='controller_node',
        name='controller_manager',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller_mode'), "' == 'MAP'"])
        ),
        parameters=[{
            'mode': LaunchConfiguration('controller_mode'),
            'l1_params_path': l1_params_path,
            'lookup_table_path': lookup_table_path,
            'map_frame': map_frame,
            'base_link_frame': base_link_frame,
            'use_sim_time': PythonExpression([
                "'true' if '", LaunchConfiguration('mod'), "' == 'sim' else 'false'"
            ])
        }],
        remappings=[
            ('/planned_path', '/planned_waypoints'),
            ('/odom', odom_topic),
            ('/frenet/odom', '/car_state/frenet/odom'),
        ]
    )

    # Pure Pursuit Controller node (PP)
    pp_controller_node = Node(
        package='crazy_controller',
        executable='pp_controller_node',
        name='pp_controller_manager',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller_mode'), "' == 'PP'"])
        ),
        parameters=[{
            'mode': LaunchConfiguration('controller_mode'),
            'pp_params_path': pp_params_path,
            'lookup_table_path': lookup_table_path,
            'map_frame': map_frame,
            'base_link_frame': base_link_frame,
            'use_sim_time': PythonExpression([
                "'true' if '", LaunchConfiguration('mod'), "' == 'sim' else 'false'"
            ])
        }],
        remappings=[
            ('/planned_path', '/planned_waypoints'),
            ('/odom', odom_topic),
            ('/frenet/odom', '/car_state/frenet/odom'),
        ]
    )

    # AUG Controller node
    aug_controller_node = Node(
        package='crazy_controller',
        executable='aug_controller_node',
        name='aug_controller_manager',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('controller_mode'), "' == 'AUG'"])
        ),
        parameters=[{
            'mode': LaunchConfiguration('controller_mode'),
            'aug_params_path': aug_params_path,
            'lookup_table_path': lookup_table_path,
            'map_frame': map_frame,
            'base_link_frame': base_link_frame,
            'use_sim_time': PythonExpression([
                "'true' if '", LaunchConfiguration('mod'), "' == 'sim' else 'false'"
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
        map_controller_node,
        pp_controller_node,
        aug_controller_node,
    ])
