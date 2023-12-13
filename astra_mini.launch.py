from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    sensor_name = LaunchConfiguration("sensor_name").perform(context)

    remapping = []
    if robot_namespace:
        remapping.append(("/tf", f"/{robot_namespace}/tf"))
        remapping.append(("/tf_static", f"/{robot_namespace}/tf_static"))

    astra_camera_pkg = get_package_share_directory("astra_camera")
    params_file = PathJoinSubstitution([astra_camera_pkg, "params", "astra_mini_params.yaml"])

    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='camera',
                namespace=sensor_name,
                parameters=[
                    params_file,
                    {
                        "camera_name": sensor_name,
                        "camera_link_frame_id": sensor_name + "_link",
                    },
                ],
                remappings=remapping,
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                namespace=sensor_name,
                name='point_cloud_xyz',
                remappings=remapping,
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                namespace=sensor_name,
                name='point_cloud_xyzrgb',
                remappings=remapping,
            ),
        ],
        output='screen',
    )

    return [PushRosNamespace(robot_namespace), container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_namespace',
                default_value=EnvironmentVariable('ROBOT_NAMESPACE'),
                description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
            ),
            DeclareLaunchArgument(
                'sensor_name',
                default_value=EnvironmentVariable('SENSOR_NAME', default_value="camera"),
                description="Name of the sensor that will appear after all topics and TF frames, used for distinguishing multiple cameras on the same robot.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
