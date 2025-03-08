#!/bin/bash

echo "=== ROS Nodes ==="
docker exec -it lsuwb_simulator bash -c "source /opt/ros/noetic/setup.bash && rosnode list" 2>/dev/null

echo -e "\n=== ROS Topics ==="
docker exec -it lsuwb_simulator bash -c "source /opt/ros/noetic/setup.bash && rostopic list" 2>/dev/null

echo -e "\n=== Tag1 Info ==="
docker exec -it lsuwb_simulator bash -c "source /opt/ros/noetic/setup.bash && rostopic info /lsuwb/Tag1" 2>/dev/null

echo -e "\n=== Tag2 Info ==="
docker exec -it lsuwb_simulator bash -c "source /opt/ros/noetic/setup.bash && rostopic info /lsuwb/Tag2" 2>/dev/null

echo -e "\n=== Tag3 Info ==="
docker exec -it lsuwb_simulator bash -c "source /opt/ros/noetic/setup.bash && rostopic info /lsuwb/Tag3" 2>/dev/null