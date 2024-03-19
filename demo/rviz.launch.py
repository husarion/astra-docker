from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def replace_rviz_fixed_frame(input_file, output_file, fix_frame):
    with open(input_file, "r") as file:
        rviz_content = file.read()

    rviz_content = rviz_content.replace("Fixed Frame: camera_link", f"Fixed Frame: {fix_frame}")

    with open(output_file, "w") as file:
        file.write(rviz_content)


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    device_namespace = LaunchConfiguration("device_namespace").perform(context)
    config = LaunchConfiguration("config").perform(context)

    modify_config = "/var/tmp/modified_default.rviz"
    replace_rviz_fixed_frame(config, modify_config, f"{device_namespace}_link")

    tf_remap = []
    if robot_namespace:
        tf_remap.append(("/tf", f"/{robot_namespace}/tf"))
        tf_remap.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=device_namespace,
        remappings=tf_remap,
        arguments=["-d", modify_config],
        output="screen",
    )

    decoder = Node(
        package="image_transport",
        executable="republish",
        name="republish",
        namespace=device_namespace,
        arguments=[
            "ffmpeg",
            "in/ffmpeg:=color/image_raw/ffmpeg",
            "raw",
            "out:=color/image_uncompressed",
        ],
        remappings=tf_remap,
        output="screen",
    )

    return [PushRosNamespace(robot_namespace), rviz, decoder]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_namespace",
                default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
                description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
            ),
            DeclareLaunchArgument(
                "device_namespace",
                default_value="camera",
                description="Sensor namespace that will appear after all topics and TF frames, used for distinguishing multiple cameras on the same robot.",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="/default.rviz",
                description="Path to rviz configuration.",
            ),
            OpaqueFunction(function=launch_setup),
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
        ]
    )
