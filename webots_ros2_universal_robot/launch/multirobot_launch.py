#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Universal Robot and ABB Robot simulation."""

import os
import xacro
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


PACKAGE_NAME = "webots_ros2_universal_robot"


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    # 读取URDF文件
    package_dir = get_package_share_directory(PACKAGE_NAME)
    ur5e_xacro_path = os.path.join(
        package_dir, "resource", "ur5e_with_gripper.urdf.xacro"
    )
    ur5e_description = xacro.process_file(
        ur5e_xacro_path, mappings={"name": "UR5eWithGripper"}
    ).toxml()
    abb_description = pathlib.Path(
        os.path.join(package_dir, "resource", "webots_abb_description.urdf")
    ).read_text()
    ur5e_control_params = os.path.join(
        package_dir, "resource", "ros2_control_config.yaml"
    )
    abb_control_params = os.path.join(
        package_dir, "resource", "ros2_control_abb_config.yaml"
    )

    spawn_URDF_ur5e = URDFSpawner(
        name="UR5e",
        robot_description=ur5e_description,
        relative_path_prefix=os.path.join(package_dir, "resource"),
        translation="0 0 0.62",
        rotation="0 0 1 -1.5708",
    )

    ur5e_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        # 包含了需要添加到节点中的额外环境变量。这个添加了一个名为WEBOTS_CONTROLLER_URL的环境变量。
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "UR5e"},
        # 在ROS中，命名空间用于将相关的节点和主题组织在一起
        namespace="ur5e",
        parameters=[
            {"robot_description": ur5e_description},
            {"use_sim_time": True},
            ur5e_control_params,
        ],
    )

    abb_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        additional_env={
            "WEBOTS_CONTROLLER_URL": controller_url_prefix() + "abbirb4600"
        },
        namespace="abb",
        parameters=[
            {"robot_description": abb_description},
            {"use_sim_time": True},
            abb_control_params,
        ],
    )

    controller_manager_timeout = ["--controller-manager-timeout", "75"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    # 这里因为我们的系统是humble，因此这个值为False
    use_deprecated_spawner_py = (
        "ROS_DISTRO" in os.environ and os.environ["ROS_DISTRO"] == "foxy"
    )
    ur5e_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner" if not use_deprecated_spawner_py else "spawner.py",
        output="screen",
        # prefix=这是要添加到命令行前的前缀
        prefix=controller_manager_prefix,
        # 传递给可执行文件的参数列表
        arguments=["ur_joint_trajectory_controller", "-c", "ur5e/controller_manager"]
        + controller_manager_timeout,
    )
    ur5e_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner" if not use_deprecated_spawner_py else "spawner.py",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["ur_joint_state_broadcaster", "-c", "ur5e/controller_manager"]
        + controller_manager_timeout,
    )
    abb_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner" if not use_deprecated_spawner_py else "spawner.py",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["abb_joint_trajectory_controller", "-c", "abb/controller_manager"]
        + controller_manager_timeout,
    )
    abb_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner" if not use_deprecated_spawner_py else "spawner.py",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["abb_joint_state_broadcaster", "-c", "abb/controller_manager"]
        + controller_manager_timeout,
    )

    # 控制节点
    ur5e_controller = Node(
        package=PACKAGE_NAME,
        executable="ur5e_controller",
        namespace="ur5e",
        output="screen",
    )
    abb_controller = Node(
        package=PACKAGE_NAME,
        executable="abb_controller",
        namespace="abb",
        output="screen",
    )

    return [
        spawn_URDF_ur5e,
        abb_driver,
        ur5e_trajectory_controller_spawner,
        ur5e_joint_state_broadcaster_spawner,
        abb_trajectory_controller_spawner,
        abb_joint_state_broadcaster_spawner,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF_ur5e,
                on_stdout=lambda event: get_webots_driver_node(
                    event, [ur5e_driver, ur5e_controller, abb_controller]
                ),
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    # 创建了一个名为world的LaunchConfiguration对象。这个对象的值可以在运行时通过命令行参数或者环境变量来设置。例如:
    # `ros2 launch your_package your_launch_file.launch.py world:=your_world`
    world = LaunchConfiguration("world")

    # Starts Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]), ros2_supervisor=True
    )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="robotic_arms.wbt",
                description="Choose one of the world files from `/webots_ros2_universal_robot/worlds` directory",
            ),
            webots,
            webots._supervisor,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.UnregisterEventHandler(
                            event_handler=reset_handler.event_handler
                        ),
                        launch.actions.EmitEvent(event=launch.events.Shutdown()),
                    ],
                )
            ),
            # Add the reset event handler
            reset_handler,
        ]
        + get_ros2_nodes()
    )
