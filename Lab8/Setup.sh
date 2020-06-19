#!/bin/bash

# Лабораторная работа №4
# Настройка окружающей среды

path_choice()
{
	# Проверяет введенный пользователем путь до целевого каталога

	if [ "${1:0:1}" = "/" ] || [ "${1:0:1}" = "./" ] ; then
		
		if [ -d "$1" ] ; then
			goal_dir="$1"
		
		elif [ -f "$1" ] ; then
			echo "Error: Expected directory, not file"
			exit 1

		else
			echo "Error: No such directory"
			exit 1
		fi
	else
		goal_dir=$("pwd")
	fi
}

######################
# ОСНОВНАЯ ПРОГРАММА #
######################

cur_dir="$(pwd)" # Рабочая директория
path_choice "$1" 
cd "$goal_dir/maze_ws"

# Настройка окружения и запуск ROS
source devel/setup.bash
roscd wall_explorer/
chmod u+x src/id_painter_node/main.py
roslaunch wall_explorer maze.launch

cd "$cur_dir" > /dev/null
exit 0
