#!/bin/bash

echo "Topic Name : $1"
echo "Power Rate : $2"

ros2 topic pub $1 std_msgs/msg/Float32 "data: $2"