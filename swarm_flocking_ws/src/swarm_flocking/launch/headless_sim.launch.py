import ast
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _parse_float_list(text: str):
    """Parse a launch argument list from Python-literal or CSV form."""
    if text is None:
        return []

    text = str(text).strip()
    if not text:
        return []

    try:
        parsed = ast.literal_eval(text)
        if isinstance(parsed, (list, tuple)):
            return [float(v) for v in parsed]
    except Exception:
        pass

    return [float(v.strip()) for v in text.split(',') if v.strip()]


def _spawn_coords_from_flat(flat, num_robots):
    coords = []
    for i in range(num_robots):
        idx = 2 * i
        if idx + 1 < len(flat):
            coords.append((flat[idx], flat[idx + 1]))
        else:
            coords.append((0.0, 0.0))
    return coords


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='6', description='Number of robots in headless sim'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use ROS sim time in headless mode'),
        DeclareLaunchArgument('dt', default_value='0.1', description='Headless physics integration time step'),
        DeclareLaunchArgument('enable_rviz', default_value='false', description='Launch RViz for visual demo'),
        DeclareLaunchArgument('success_timeout_s', default_value='300.0', description='Monitor timeout in seconds before completion'),
        DeclareLaunchArgument('auto_shutdown_on_completion', default_value='true', description='Auto-stop nodes when monitor completes'),
        DeclareLaunchArgument(
            'waypoints',
            default_value='[8.0, 7.5, 15.0, 7.5, 22.0, 7.5, 28.0, 7.5]',
            description='Flat waypoint list [x0,y0,x1,y1,...]',
        ),
        DeclareLaunchArgument(
            'spawn_coords',
            default_value='[-2.0, -0.5, -2.0, 0.5, -3.0, -0.5, -3.0, 0.5, -4.0, -0.5, -4.0, 0.5]',
            description='Flat spawn offsets [x0,y0,x1,y1,...] used by boid world-frame conversion',
        ),
        OpaqueFunction(function=_build_headless_graph),
    ])


def _build_headless_graph(context, *args, **kwargs):
    pkg_flocking = get_package_share_directory('swarm_flocking')
    params_file = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')
    rviz_cfg = os.path.join(pkg_flocking, 'config', 'rviz_config.rviz')

    num_robots = int(context.launch_configurations.get('num_robots', '6'))
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false').lower() == 'true'
    dt = float(context.launch_configurations.get('dt', '0.1'))
    enable_rviz = context.launch_configurations.get('enable_rviz', 'false').lower() == 'true'
    success_timeout_s = float(context.launch_configurations.get('success_timeout_s', '300.0'))
    auto_shutdown_on_completion = (
        context.launch_configurations.get('auto_shutdown_on_completion', 'true').lower() == 'true'
    )

    waypoints = _parse_float_list(context.launch_configurations.get('waypoints', ''))
    spawn_flat = _parse_float_list(context.launch_configurations.get('spawn_coords', ''))
    spawn_coords = _spawn_coords_from_flat(spawn_flat, num_robots)

    actions = []

    # 1. Physics engine (headless)
    actions.append(
        Node(
            package='swarm_flocking',
            executable='physics_node',
            name='physics_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'num_robots': num_robots,
                'dt': dt,
                'spawn_coords': spawn_flat,
            }],
        )
    )

    # 2. Experiment manager
    actions.append(
        Node(
            package='swarm_flocking',
            executable='flock_monitor_node',
            name='flock_monitor_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'num_robots': num_robots,
                    'waypoints': waypoints,
                    'success_timeout_s': success_timeout_s,
                    'auto_shutdown_on_completion': auto_shutdown_on_completion,
                },
            ],
        )
    )

    if enable_rviz:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        )

    # 3. Decentralized boid nodes
    for i in range(num_robots):
        sx, sy = spawn_coords[i]
        actions.append(
            Node(
                package='swarm_flocking',
                executable='boid_node',
                name=f'boid_node_{i}',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'robot_id': i,
                        'num_robots': num_robots,
                        'spawn_x': sx,
                        'spawn_y': sy,
                        'waypoints': waypoints,
                    },
                ],
            )
        )

    return actions
