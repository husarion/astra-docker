from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    sensor_namespace = LaunchConfiguration("sensor_namespace").perform(context)

    remapping = []
    if robot_namespace:
        remapping.append(("/tf", f"/{robot_namespace}/tf"))
        remapping.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    astra_camera_pkg = get_package_share_directory("astra_camera")
    params_file = PathJoinSubstitution([astra_camera_pkg, "params", "astra_mini_params.yaml"])

    astra_container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='camera',
                namespace=sensor_namespace,
                parameters=[
                    params_file,
                    {
                        "camera_name": sensor_namespace,
                        "camera_link_frame_id": sensor_namespace + "_link",
                    },
                ],
                remappings=remapping,
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                name='point_cloud_xyz',
                namespace=sensor_namespace,
                remappings=remapping,
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb',
                namespace=sensor_namespace,
                remappings=remapping,
            ),
        ],
        output='screen',
    )

    healthcheck_node = Node(
        package='healthcheck_pkg',
        executable='healthcheck_node',
        name='healthcheck_astra',
        namespace=sensor_namespace,
        output='screen',
    )

    return [PushRosNamespace(robot_namespace), astra_container, healthcheck_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_namespace',
                default_value=EnvironmentVariable('ROBOT_NAMESPACE', default_value=""),
                description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
            ),
            DeclareLaunchArgument(
                'sensor_namespace',
                default_value="camera",
                description="Sensor namespace that will appear after all topics and TF frames, used for distinguishing multiple cameras on the same robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
