from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    parameters = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "approx_sync": True,
            "approx_sync_max_interval": 0.01,
            "publish_tf": False,          
            "wait_imu_to_init": False,
            "publish_null_when_lost": False,
            "qos": 2,
            "qos_camera_info": 2,
            "use_sim_time": use_sim_time,
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "1",
            "Odom/Holonomic": "false",
            "Odom/FilteringStrategy": "1",
            "Odom/ParticleSize": "500",
            "Odom/GuessMotion": "true",
            "Odom/AlignWithGround": "false",
            "OdomF2M/MaxSize": "5000",
            "OdomF2M/ScanMaxSize": "5000",
            "GFTT/MinDistance": "7.0",
            "GFTT/QualityLevel": "0.001",
            "GFTT/BlockSize": "3",
            "GFTT/UseHarrisDetector": "true",
            "GFTT/K": "0.04",
            "Vis/EstimationType": "1",
            "Vis/ForwardEstOnly": "true",
            "Vis/FeatureType": "8",
            "Vis/DepthAsMask": "true",
            "Vis/CorGuessWinSize": "40",
            "Vis/MaxFeatures": "0",
            "Vis/MinDepth": "0.0",
            "Vis/MaxDepth": "0.0",
            "Vis/CorType": "0",
            "Vis/CorNNType": "1",
        }
    ]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        ("odom", "odom_rgbd"),            
    ]

    return LaunchDescription([
        use_sim_time_cmd,
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "Error"],
        ),
    ])