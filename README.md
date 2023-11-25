Step 1 - Launch Robot
    ros2 launch sam_bot_description display.launch.py

Step 2 - Launch Localization
    ros2 launch nav2_bringup localization_launch.py map:=./map/two_rooms_map.yaml use_sim_time:=true params_file:=/home/trsa2024/ros2_lab2/src/sam_bot_description/config/nav2_params.yaml

Step 3 - Launch Navigation
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/trsa2024/ros2_lab2/src/sam_bot_description/config/nav2_params.yaml

Step 4 - Launch Node
    ros2 run robot_control goal_send

Step 5 - Request Goal
    ros2 service call /start_navigation std_srvs/Trigger "{}"