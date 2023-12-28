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

    default_rviz_conf = "/root/.rviz2/default.rviz"
    rviz_conf = "/root/.rviz2/modified_default.rviz"
    replace_rviz_fixed_frame(default_rviz_conf, rviz_conf, f"{device_namespace}_link")

    remapping = []
    if robot_namespace:
        remapping.append(("/clicked_point", f"/{robot_namespace}/clicked_point"))
        remapping.append(("/goal_pose", f"/{robot_namespace}/goal_pose"))
        remapping.append(("/initialpose", f"/{robot_namespace}/initialpose"))
        remapping.append(("/tf", f"/{robot_namespace}/tf"))
        remapping.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=device_namespace,
        remappings=remapping,
        arguments=["-d", rviz_conf],
        output="screen",
    )

    return [PushRosNamespace(robot_namespace), rviz]


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
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
        ]
    )
