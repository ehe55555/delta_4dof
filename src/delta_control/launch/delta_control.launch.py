#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    workspace = os.path.join(os.path.expanduser("~"), "delta_4dof")
    world = os.path.join(
        workspace,
        "src",
        "descripe",
        "worlds",
        "descripe_test.world",
    )
    model_path = os.path.join(workspace, "src", "descripe", "models")
    controller_path = os.path.join(
        workspace,
        "install",
        "gz_delta_controller2",
        "lib",
        "gz_delta_controller2",
    )

    old_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    plugin_path = ":".join(
        value for value in (controller_path, old_plugin_path) if value
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=model_path,
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=plugin_path,
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-v", "4", world],
                output="screen",
            ),
            Node(
                package="delta_bridge2",
                executable="delta_bridge2_node",
                output="screen",
            ),
            Node(
                package="delta_bridge2",
                executable="feedback_bridge_node",
                output="screen",
            ),
            Node(
                package="delta_control",
                executable="delta_gui",
                output="screen",
            ),
        ]
    )
