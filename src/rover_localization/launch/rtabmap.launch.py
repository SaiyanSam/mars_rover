from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Whether to launch rtabmapviz",
    )

    parameters = [
        {
            "frame_id": "base_link",
            "odom_frame_id": "odom", 
            "Mem/IncrementalMemory": "true",     
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": False,
            "approx_sync": True,
            "publish_tf": True,           # <-- KEEP TRUE: RTAB-Map master-controls map -> odom
            "use_sim_time": use_sim_time,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_imu": 2,
            "qos_odom": 1,
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "RGBD/Enabled": "true",
            "RGBD/OptimizeMaxError": "0.5",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/Sensor": "1",
            "Grid/RangeMax": "5.0",
            "Grid/CellSize": "0.1",
        }
    ]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        # --- THE SYNCHRONIZATION LINK ---
        # Remap this to your EKF output topic so RTAB-Map knows where the robot is locally
        ("odom", "odometry/filtered"),    
        ("goal", "goal_pose"),
        ("grid_map", "map")
    ]

    return LaunchDescription([
        use_sim_time_cmd,
        launch_rtabmapviz_cmd,
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=["-d", "--ros-args", "--log-level", "Error"],
        ),
        Node(
            condition=IfCondition(launch_rtabmapviz),
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
    ])