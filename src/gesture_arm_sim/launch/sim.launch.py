from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    pkg = FindPackageShare("gesture_arm_sim")
    world = PathJoinSubstitution([pkg, "worlds", "pick_place.sdf"])
    description = PathJoinSubstitution([pkg, "urdf", "ur_gripper.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
        launch_arguments={"ur_type": ur_type, "world_file": world, "launch_rviz": "false",
                          "description_file": description, "controllers_file": controllers}.items())

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={"ur_type": ur_type, "use_sim_time": "true", "launch_rviz": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("moveit")))

    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": True}])

    cam_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        parameters=[{"use_sim_time": True}])

    cam_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0.6", "--y", "0", "--z", "1.0",
                   "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", "camera_link"],
        parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("moveit", default_value="true"),
        control, moveit, gripper_spawner, cam_bridge, cam_tf])
