#!/bin/bash

# Лабораторная работа №3
# Создание рабочего окружения ROS

package_check()
{	
	# Проверяет, установлен ли пакет в системе

	if [ -z "$(dpkg --get-selections | grep "$package")" ] ; then
		echo "Error: Package $package not found. Go to http://wiki.ros.org/melodic/Installation/Ubuntu to install it"
		exit 1
	fi
}

dir_check()
{
	# Проверяет введенный пользователем путь до целевого каталога

	if [ "$empty_arg" -eq 1 ] ; then
		
		if [ -d "$wrk_dir" ] ; then
			wrk_dir="$wrk_dir"
		
		elif [ -f "$wrk_dir" ] ; then
			echo "Error: Expected directory, not file"
			exit 1

		else
			echo "Error: No such directory"
			exit 1
		fi
	else
		wrk_dir=$("pwd")
	fi
}

env_build()
{
	mkdir -p "$dpath/$pkg/src" && cd "$_"
catkin_create_pkg turtle_move geometry_msgs rospy

cd ..
catkin_make
touch ./src/turtle_move/src/p_id_paint.py
}
######################
# ОСНОВНАЯ ПРОГРАММА #
######################

package="ros"
cur_dir=$("pwd")
wrk_dir=""
emty_arg=0

package_check $package

while getopts ":b" option ; do
	case "$option" in
		"b")
			echo "Option find"
			;;
		"?")
			echo "Not option"
			;;
		*)
			echo "Error"
			;;
	esac
done

for usr_dir in "${@:$OPTIND}" ; do
	wrk_dir=$usr_dir
	empty_arg=1
	break 
done

dir_check "$wrk_dir" $empty_arg

echo "$wrk_dir"
#chmod u+x "~/$pkg_name/src/turtle_move/src/p_id_paint.py"

#source devel/setup.bash

cd "$cur_dir" > /dev/null
exit 0
