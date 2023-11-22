Step 1 - Launch Robot
    ros2 launch sam_bot_description display.launch.py

Step 2 - Launch Localization
    ros2 launch nav2_bringup localization_launch.py map:=./map/two_rooms_map.yaml use_sim_time:=true

Step 3 - Launch Navigation
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
