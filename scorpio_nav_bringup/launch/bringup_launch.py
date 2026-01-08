# Copyright 2026 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("scorpio_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    map_cloud_dir = LaunchConfiguration("map_cloud_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    simulation = LaunchConfiguration("simulation")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    # Only it applies when `namespace` is not empty.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_map_cloud_dir_cmd = DeclareLaunchArgument(
        "map_cloud_dir", description="Full path to prior PCD file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "reality", "nav_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_simulation_cmd = DeclareLaunchArgument(
        "simulation",
        default_value="False",
        description="simulation mode will disable plain_slam_ros2 nodes",
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                condition=IfCondition(use_composition),
                name="scorpio_nav_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                # condition=IfCondition(slam),
                condition=IfCondition(
                    AndSubstitution(NotSubstitution(simulation), slam)
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "scorpio_nav_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(
                    AndSubstitution(NotSubstitution(simulation), NotSubstitution(slam))
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "map_cloud_dir": map_cloud_dir,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "scorpio_nav_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "scorpio_nav_container",
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_map_cloud_dir_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_simulation_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
