"""Launch the self-contained EmberTrace forest and one Husky UGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("embertrace_sim")
    config_dir = PathJoinSubstitution([package_share, "config"])
    model_dir = PathJoinSubstitution([package_share, "models"])
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([package_share, "urdf", "husky.urdf.xacro"]),
                " prefix:=husky1_ gz_model_name:=husky1",
            ]
        ),
        value_type=str,
    )

    def launch_world(context):
        scenario = LaunchConfiguration("scenario").perform(context)
        gui = LaunchConfiguration("gazebo_gui").perform(context).lower() == "true"
        gui_engine = LaunchConfiguration("gui_render_engine").perform(context)
        if scenario not in (
            "fire",
            "no_fire",
            "dust",
            "hot_surface",
            "visual_decoy",
            "dropout",
        ):
            raise ValueError("Unknown scenario: " + scenario)
        world = (
            Path(get_package_share_directory("embertrace_sim"))
            / "worlds"
            / LaunchConfiguration("world_file").perform(context)
        )
        tree = ET.parse(world)
        if scenario in ("no_fire", "dust", "hot_surface"):
            parent = tree.getroot().find("world")
            for item in list(parent.findall("include")):
                if item.findtext("name") == "fire_source_visual":
                    parent.remove(item)
        if scenario == "visual_decoy":
            parent = tree.getroot().find("world")
            for item in list(parent.findall("include")):
                if item.findtext("name") == "fire_source_visual":
                    pose = item.findtext("pose", "12 7 0 0 0 0")
                    parent.remove(item)
                    decoy = (
                        ET.parse(
                            Path(get_package_share_directory("embertrace_sim"))
                            / "models/fire_source/model.sdf"
                        )
                        .getroot()
                        .find("model")
                    )
                    decoy.set("name", "visual_decoy")
                    for visual in decoy.findall(".//visual"):
                        for plugin in list(visual.findall("plugin")):
                            if plugin.get("name", "").endswith("::Thermal"):
                                plugin.find("temperature").text = "295.15"
                    ET.SubElement(decoy, "pose").text = pose
                    parent.append(decoy)
        if scenario == "hot_surface":
            # Existing rock geometry, heated without smoke/CO2 or an RGB flame cue.
            model = (
                ET.parse(
                    Path(get_package_share_directory("embertrace_sim"))
                    / "models/ember_rock/model.sdf"
                )
                .getroot()
                .find("model")
            )
            model.set("name", "heated_rock")
            ET.SubElement(model, "pose").text = "12 7 0 0 0 0"
            for visual in model.findall(".//visual"):
                plugin = ET.SubElement(
                    visual,
                    "plugin",
                    {
                        "filename": "ignition-gazebo-thermal-system",
                        "name": "ignition::gazebo::systems::Thermal",
                    },
                )
                ET.SubElement(plugin, "temperature").text = "353.15"
            tree.getroot().find("world").append(model)
        with tempfile.NamedTemporaryFile(
            prefix="embertrace_", suffix=".sdf", delete=False
        ) as handle:
            tree.write(handle, encoding="utf-8", xml_declaration=True)
            runtime_world = handle.name

        def cleanup(_context):
            Path(runtime_world).unlink(missing_ok=True)
            return []

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_ign_gazebo"),
                            "launch",
                            "ign_gazebo.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ign_args": [runtime_world, " -r --render-engine-gui " + gui_engine if gui else " -r -s -v 3"]
                }.items(),
            ),
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])
            ),
        ]

    gazebo = OpaqueFunction(function=launch_world)

    drone_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace="drone1",
        name="gazebo_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_drone")),
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [config_dir, "gazebo_bridge_drone1.yaml"]
                ),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    spawn_drone = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_ign_gazebo",
                executable="create",
                name="spawn_drone1",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_drone")),
                arguments=[
                    "-file",
                    PathJoinSubstitution([model_dir, "scout_drone", "model.sdf"]),
                    "-name",
                    "drone1",
                    "-x",
                    LaunchConfiguration("drone_x"),
                    "-y",
                    LaunchConfiguration("drone_y"),
                    "-z",
                    LaunchConfiguration("drone_z"),
                ],
            )
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="husky1",
        name="robot_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_rover")),
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    spawn_husky = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_ign_gazebo",
                executable="create",
                name="spawn_husky1",
                condition=IfCondition(LaunchConfiguration("use_rover")),
                output="screen",
                arguments=[
                    "-topic",
                    "/husky1/robot_description",
                    "-name",
                    "husky1",
                    "-x",
                    LaunchConfiguration("rover_x"),
                    "-y",
                    LaunchConfiguration("rover_y"),
                    "-z",
                    "0.4",
                    "-Y",
                    "0.7854",
                ],
            )
        ],
    )

    robot_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace="husky1",
        name="gazebo_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rover")),
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [config_dir, "gazebo_bridge_husky1.yaml"]
                ),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    clock_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [config_dir, "gazebo_bridge_clock.yaml"]
                ),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace="husky1",
        name="robot_localization",
        condition=IfCondition(LaunchConfiguration("use_rover")),
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir, "robot_localization_husky1.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("odometry/filtered", "odom"),
        ],
    )

    navigation = TimerAction(
        period=7.0,
        condition=IfCondition(PythonExpression(["'", use_nav2, "'.lower() == 'true' and '",
                                                 LaunchConfiguration("use_rover"), "'.lower() == 'true'"])),
        actions=[
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [package_share, "launch", "navigation.launch.py"]
                            )
                        ),
                        launch_arguments={"use_sim_time": use_sim_time}.items(),
                    )
                ],
            )
        ],
    )

    rviz = TimerAction(
        period=8.0,
        condition=IfCondition(use_rviz),
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                namespace="husky1",
                name="embertrace_rviz",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-d",
                    PathJoinSubstitution([package_share, "rviz", "embertrace.rviz"]),
                ],
                remappings=[("/tf", "/husky1/tf"), ("/tf_static", "/husky1/tf_static")],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="embertrace_forest.sdf"),
            DeclareLaunchArgument("use_rover", default_value="true"),
            DeclareLaunchArgument("use_drone", default_value="true"),
            DeclareLaunchArgument("drone_z", default_value="0.6"),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument("gui_render_engine", default_value="ogre" if Path('/mnt/wslg').exists() else "ogre2",
                                  choices=["ogre", "ogre2"]),
            DeclareLaunchArgument("rover_x", default_value="-14.0"),
            DeclareLaunchArgument("rover_y", default_value="-14.0"),
            DeclareLaunchArgument("drone_x", default_value="-14.0"),
            DeclareLaunchArgument("drone_y", default_value="-12.0"),
            DeclareLaunchArgument("scenario", default_value="fire"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            SetEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                [
                    model_dir,
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    model_dir,
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "IGN_GAZEBO_SERVER_CONFIG_PATH",
                PathJoinSubstitution([config_dir, "ignition_server.config"]),
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SERVER_CONFIG_PATH",
                PathJoinSubstitution([config_dir, "ignition_server.config"]),
            ),
            gazebo,
            clock_bridge,
            robot_state_publisher,
            robot_bridge,
            ekf,
            spawn_husky,
            drone_bridge,
            spawn_drone,
            navigation,
            rviz,
        ]
    )
