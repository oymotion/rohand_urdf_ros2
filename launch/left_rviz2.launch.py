import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "rohand_urdf_ros2"
    urdf_name = "rohand_left.urdf"
    rviz_name = "rohand_left_urdf.rviz"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    rviz_path = os.path.join(pkg_share, f"rviz/{rviz_name}")


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rohand_left_state_publisher",
        namespace="rohand_left",
        parameters=[{
            "frame_prefix": "rohand_left/",
            }],
        remappings=[("/joint_states", "/rohand_left/joint_states")],
        arguments=[urdf_model_path]
    )

    joint_state_publisher_node = Node(
        package=package_name,
        executable="rohand_joint_state_gui",
        name="rohand_left_joint_state_gui",
        # namespace="rohand_left",
        remappings=[("/joint_states", "/rohand_left/rohand_left_urdf_node/joint_states")],
        arguments=[urdf_model_path]
    )

    finger_sync_node = Node(
        package=package_name,
        executable="rohand_urdf_node",
        name="rohand_left_urdf_node",
        namespace="rohand_left",
        remappings=[("/joint_states", "/rohand_left/joint_states")],
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rohand_left_rviz2",
        output="screen",
        namespace="rohand_left",
        arguments=["-d", rviz_path]
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(finger_sync_node)
    ld.add_action(rviz2_node)

    return ld
