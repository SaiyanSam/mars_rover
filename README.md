# Project Submission for Summer of Robotics

To run the final demo:

Terminal 1:
cd ~/mars_rover
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_gazebo mars.launch.py

Terminal 2: (from same directory)
ros2 run rover_exploration visual_rock_explorer_node \
  --ros-args \
  -p rgb_topic:=/camera/image_raw \
  -p depth_topic:=/camera/depth/image_raw \
  -p segmentation_topic:=/rock_segmentation/debug_image \
  -p max_goals:=5 \
  -p dark_value_threshold:=130 \
  -p rock_saturation_max:=150 \
  -p rock_red_dominance_max:=35 \
  -p min_blob_area:=1000 \
  -p relative_blob_area:=0.15 \
  -p min_rock_depth:=0.6 \
  -p max_rock_depth:=6.5 \
  -p min_detection_hits:=2 \
  -p capture_distance:=1.25 \
  -p approach_linear_speed:=0.07 \
  -p approach_slow_linear_speed:=0.04 \
  -p approach_angular_gain:=0.9 \
  -p max_approach_angular:=0.45 \
  -p explore_linear_speed:=0.1 \
  -p explore_turn_linear_speed:=0.065 \
  -p explore_angular_speed:=0.42 \
  -p front_blocked_depth:=2.6 \
  -p sector_safe_depth:=3.2 \
  -p roughness_weight:=1.0 \
  -p slope_weight:=2.0 \
  -p close_penalty_weight:=4.0 \
  -p close_fraction_blocked:=0.18 \
  -p backtrack_depth:=1.7 \
  -p backtrack_slope_threshold:=0.75 \
  -p backtrack_duration_sec:=1.8 \
  -p backtrack_linear_speed:=-0.06 \
  -p backtrack_angular_speed:=0.35 \
  -p backtrack_cooldown_sec:=4.0 \
  -p draw_depth_sectors:=false \
  -p output_dir:=captured_images
