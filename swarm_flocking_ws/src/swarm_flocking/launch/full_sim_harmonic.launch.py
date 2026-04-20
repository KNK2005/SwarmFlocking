#!/usr/bin/env python3
"""
full_sim_harmonic.launch.py - Gazebo Harmonic (ros_gz) multi-robot simulation.

Starts:
  1. Gazebo Sim world via ros_gz_sim
  2. N differential-drive robots with lidar (SDF generated per robot)
  3. ros_gz_bridge for /clock and per-robot cmd_vel/odom/scan topics
  4. N boid_node instances and one flock_monitor_node
  5. RViz
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_flocking = get_package_share_directory('swarm_flocking')
    pkg_gazebo = get_package_share_directory('swarm_flocking_gazebo')

    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    except Exception as exc:
        raise RuntimeError(
            "Could not find 'ros_gz_sim'. Install Gazebo Harmonic integration packages."
        ) from exc

    world_file = PathJoinSubstitution([
        pkg_gazebo,
        'worlds',
        PythonExpression(["'", LaunchConfiguration('world_name'), "' + '.world'"]),
    ])

    rviz_cfg = os.path.join(pkg_flocking, 'config', 'rviz_config.rviz')

    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='6', description='Number of robots to spawn')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock')
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='obstacle_course', description='World basename from swarm_flocking_gazebo/worlds')

    # Harmonic uses GZ_SIM_RESOURCE_PATH for resolving model:// resources.
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # Keep legacy variable populated as some setups still read it.
    ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models') + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
        }.items(),
    )

    spawn_and_boids = OpaqueFunction(function=_spawn_all_harmonic)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_XCB_GL_INTEGRATION': 'none',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            'MESA_GLSL_VERSION_OVERRIDE': '330',
        },
        output='screen',
    )

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        world_name_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim_launch,
        spawn_and_boids,
        rviz_node,
    ])


def _spawn_all_harmonic(context, *args, **kwargs):
    from launch.actions import TimerAction
    from launch_ros.actions import Node as RosNode

    # Runtime requirements for Harmonic path.
    try:
        get_package_share_directory('ros_gz_sim')
        get_package_share_directory('ros_gz_bridge')
    except Exception as exc:
        raise RuntimeError(
            "Gazebo Harmonic path requires both 'ros_gz_sim' and 'ros_gz_bridge'."
        ) from exc

    pkg_flocking = get_package_share_directory('swarm_flocking')
    params_file = os.path.join(pkg_flocking, 'config', 'flocking_params.yaml')

    num_robots = int(context.launch_configurations.get('num_robots', '6'))
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
    start_x, start_y = 2.0, 4.5
    spacing = 0.7

    # One bridge handles clock + all robot topics.
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    for i in range(num_robots):
        ns = f'robot_{i}'
        bridge_args.extend([
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ])

    actions = [
      # RViz often uses map as fixed frame while Gazebo odom streams are in odom.
      # Publish an identity transform so odom messages can be transformed to map.
      RosNode(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
      ),
        RosNode(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_swarm',
            arguments=bridge_args,
            output='screen',
        )
    ]

    tmp_dir = tempfile.mkdtemp(prefix='swarm_harmonic_')

    WORLD_READY_DELAY = 12.0
    SPAWN_INTERVAL = 1.2
    BOID_DELAY_AFTER_SPAWN = 2.5

    for i in range(num_robots):
        row = i // 3
        col = i % 3
        x = start_x + col * spacing
        y = start_y + row * spacing
        ns = f'robot_{i}'

        sdf_path = os.path.join(tmp_dir, f'{ns}.sdf')
        with open(sdf_path, 'w', encoding='utf-8') as f:
            f.write(_harmonic_robot_sdf(ns))

        spawn_time = WORLD_READY_DELAY + i * SPAWN_INTERVAL
        boid_time = spawn_time + BOID_DELAY_AFTER_SPAWN

        spawn_action = TimerAction(
            period=float(spawn_time),
            actions=[
                RosNode(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', ns,
                      '-allow_renaming', 'true',
                        '-x', str(x),
                        '-y', str(y),
                      '-z', '0.0',
                        '-file', sdf_path,
                    ],
                    output='screen',
                ),
            ],
        )

        boid_action = TimerAction(
            period=float(boid_time),
            actions=[
                RosNode(
                    package='swarm_flocking',
                    executable='boid_node',
                    name=f'boid_{i}',
                    namespace=ns,
                    parameters=[
                        params_file,
                        {
                            'robot_id': i,
                            'num_robots': num_robots,
                            'spawn_x': x,
                            'spawn_y': y,
                            'use_sim_time': use_sim_time == 'true',
                        },
                    ],
                    output='screen',
                ),
            ],
        )

        actions.extend([spawn_action, boid_action])

    monitor = RosNode(
        package='swarm_flocking',
        executable='flock_monitor_node',
        name='flock_monitor',
        parameters=[
            params_file,
            {
                'num_robots': num_robots,
                'use_sim_time': use_sim_time == 'true',
            },
        ],
        output='screen',
    )
    actions.append(monitor)

    return actions


def _harmonic_robot_sdf(robot_ns: str) -> str:
    """Create a compact differential-drive SDF with lidar and ROS-friendly topics."""
    return f"""<?xml version=\"1.0\"?>
<sdf version=\"1.9\">
  <model name=\"{robot_ns}\">
    <pose>0 0 0 0 0 0</pose>
    <static>false</static>

    <link name=\"base_footprint\">
      <pose>0 0 0.06 0 0 0</pose>
      <inertial>
        <mass>1.2</mass>
        <inertia>
          <ixx>0.01</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.01</iyy><iyz>0.0</iyz><izz>0.02</izz>
        </inertia>
      </inertial>
      <collision name=\"base_collision\">
        <geometry>
          <box><size>0.18 0.16 0.06</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.02</mu>
              <mu2>0.02</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"base_visual\">
        <geometry>
          <box><size>0.18 0.16 0.06</size></box>
        </geometry>
        <material>
          <ambient>0.2 0.6 0.9 1</ambient>
          <diffuse>0.2 0.6 0.9 1</diffuse>
        </material>
      </visual>

      <sensor name=\"laser\" type=\"lidar\">
        <pose>0.08 0 0.06 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>12</update_rate>
        <topic>/{robot_ns}/scan</topic>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>3.5</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
      </sensor>
    </link>

    <link name=\"left_wheel\">
      <pose>0 0.085 0.033 1.5708 0 0</pose>
      <inertial>
        <mass>0.15</mass>
        <inertia>
          <ixx>0.0002</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.0002</iyy><iyz>0.0</iyz><izz>0.0002</izz>
        </inertia>
      </inertial>
      <collision name=\"left_wheel_collision\">
        <geometry>
          <cylinder><radius>0.033</radius><length>0.02</length></cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>2.0</mu>
              <mu2>2.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"left_wheel_visual\">
        <geometry>
          <cylinder><radius>0.033</radius><length>0.02</length></cylinder>
        </geometry>
      </visual>
    </link>

    <link name=\"right_wheel\">
      <pose>0 -0.085 0.033 1.5708 0 0</pose>
      <inertial>
        <mass>0.15</mass>
        <inertia>
          <ixx>0.0002</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.0002</iyy><iyz>0.0</iyz><izz>0.0002</izz>
        </inertia>
      </inertial>
      <collision name=\"right_wheel_collision\">
        <geometry>
          <cylinder><radius>0.033</radius><length>0.02</length></cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>2.0</mu>
              <mu2>2.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"right_wheel_visual\">
        <geometry>
          <cylinder><radius>0.033</radius><length>0.02</length></cylinder>
        </geometry>
      </visual>
    </link>

    <link name=\"caster_front\">
      <pose>0.08 0 0.012 0 0 0</pose>
      <inertial><mass>0.02</mass></inertial>
      <collision name=\"caster_front_collision\">
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.01</mu>
              <mu2>0.01</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"caster_front_visual\">
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
      </visual>
    </link>

    <link name=\"caster_back\">
      <pose>-0.08 0 0.012 0 0 0</pose>
      <inertial><mass>0.02</mass></inertial>
      <collision name=\"caster_back_collision\">
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.01</mu>
              <mu2>0.01</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name=\"caster_back_visual\">
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
      </visual>
    </link>

    <joint name=\"left_wheel_joint\" type=\"revolute\">
      <parent>base_footprint</parent>
      <child>left_wheel</child>
      <axis>
        <xyz expressed_in=\"__model__\">0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <joint name=\"right_wheel_joint\" type=\"revolute\">
      <parent>base_footprint</parent>
      <child>right_wheel</child>
      <axis>
        <xyz expressed_in=\"__model__\">0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <joint name=\"caster_front_joint\" type=\"fixed\">
      <parent>base_footprint</parent>
      <child>caster_front</child>
    </joint>

    <joint name=\"caster_back_joint\" type=\"fixed\">
      <parent>base_footprint</parent>
      <child>caster_back</child>
    </joint>

    <plugin filename=\"gz-sim-diff-drive-system\" name=\"gz::sim::systems::DiffDrive\">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.17</wheel_separation>
      <wheel_radius>0.033</wheel_radius>
      <topic>/{robot_ns}/cmd_vel</topic>
      <odom_topic>/{robot_ns}/odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>{robot_ns}/base_footprint</child_frame_id>
      <max_linear_velocity>0.22</max_linear_velocity>
      <max_angular_velocity>2.8</max_angular_velocity>
    </plugin>

    <plugin filename=\"gz-sim-joint-state-publisher-system\" name=\"gz::sim::systems::JointStatePublisher\"/>
  </model>
</sdf>
"""
