#!/bin/bash
# Part 2: grade a recorded lap against your map. Usage: lab3/grade.sh lab3/bags/<bag>
cd "$(git rev-parse --show-toplevel)"
ros2 run autosdv_lab3 lab3_grade "$1" --map lab3/map
