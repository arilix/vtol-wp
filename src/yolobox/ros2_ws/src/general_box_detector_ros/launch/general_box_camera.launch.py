import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("general_box_detector_ros")
    rviz_config = os.path.join(package_share, "config", "general_box.rviz")

    arguments = [
        # Topic image yang sudah dipublish node lain (capture_node /
        # realsense2_camera_node dari package fiducial_detector) — node ini
        # tidak membuka device kamera sendiri, cuma subscribe. Default sama
        # dengan output_topic capture_node supaya bisa pakai kamera nadir
        # yang sama dengan aruco_node (dipakai gantian, lihat PROGRAM_OVERVIEW.md).
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/vtol/wayfix_ws/src/yolobox/models/model.hef",
        ),
        DeclareLaunchArgument(
            "output",
            default_value="/home/vtol/wayfix_ws/src/yolobox/general_box_ros_camera.mp4",
        ),
        DeclareLaunchArgument("confidence", default_value="0.70"),
        DeclareLaunchArgument("max_frames", default_value="0"),
        DeclareLaunchArgument("save_video", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("preprocess_mode", default_value="letterbox"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_child_frame", default_value="yolo_bbox_target"),
        DeclareLaunchArgument("tf_depth_m", default_value="1.0"),
    ]

    detector = Node(
        package="general_box_detector_ros",
        executable="yolo_camera_node",
        name="general_box_camera",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "output": LaunchConfiguration("output"),
                "confidence": LaunchConfiguration("confidence"),
                "max_frames": LaunchConfiguration("max_frames"),
                "save_video": LaunchConfiguration("save_video"),
                "publish_topic": "/general_box/image_annotated",
                "count_topic": "/general_box/detection_count",
                "boxes_topic": "/general_box/detections",
                "center_topic": "/general_box/target_center",
                "frame_id": "general_box_camera",
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "tf_child_frame": LaunchConfiguration("tf_child_frame"),
                "tf_depth_m": LaunchConfiguration("tf_depth_m"),
                "target_class_id": 3,
                "preprocess_mode": LaunchConfiguration("preprocess_mode"),
                "input_color_order": "rgb",
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="general_box_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(arguments + [detector, rviz])
