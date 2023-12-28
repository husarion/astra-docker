from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    device_namespace = LaunchConfiguration("device_namespace").perform(context)

    remapping = []
    if robot_namespace:
        remapping.append(("/tf", f"/{robot_namespace}/tf"))
        remapping.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    params_file = "/husarion_utils/astra_params.yaml"

    astra_node = Node(
        package="astra_camera",
        executable="astra_camera_node",
        name="camera",
        namespace=device_namespace,
        parameters=[
            params_file,
            {
                "camera_name": device_namespace,
                "camera_link_frame_id": device_namespace + "_link",
            },
        ],
        remappings=remapping,
        output="screen",
    )

    healthcheck_node = Node(
        package="healthcheck_pkg",
        executable="healthcheck_node",
        name="healthcheck_astra",
        namespace=device_namespace,
        output="screen",
    )

    return [PushRosNamespace(robot_namespace), astra_node, healthcheck_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value=EnvironmentVariable("ROS_NAMESPACE", default_value=""),
                description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
            ),
            DeclareLaunchArgument(
                "device_namespace",
                default_value="camera",
                description="Sensor namespace that will appear after all topics and TF frames, used for distinguishing multiple cameras on the same robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
